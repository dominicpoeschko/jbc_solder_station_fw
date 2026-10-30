#!/usr/bin/env bash
# jbc_tui --dump and --render against jbc_fake (the firmware's command handler and a simulated tip
# behind a unix socket): tunables set, clamped and refused, the tip heating to its setpoint, the
# holder, a station reset, the latch, and the screen.
#
# No step waits for a time: each waits for the line that shows the state it needs (`--dump until=`,
# waitLine), so a slow machine only takes longer. A line that never comes is a hang, ended by
# ctest's TIMEOUT; the "== step" lines say where. (psu302's tui/e2e_test.sh, the same rules.)
#
#   e2e_test.sh <jbc_fake> <jbc_tui> <work dir>
set -uo pipefail

fake=$1
tui=$2
work=$3
mkdir -p "$work"
sock=$work/duplex.0.sock
# a unix socket path holds 107 bytes
temp=""
if [ ${#sock} -gt 80 ]; then
    temp=$(mktemp -d)
    work=$temp
    sock=$work/duplex.0.sock
fi

failures=0
fail() {
    echo "FAIL: $*"
    failures=$((failures + 1))
}
expect() { # <file> <extended regex> <what>
    grep -qE "$2" "$1" || fail "$3 (no /$2/ in $1)"
}

fakePid=""
startFake() { # <args...>
    rm -f "$sock"
    "$fake" "$sock" "$@" >>"$work/fake.log" 2>&1 &
    fakePid=$!
    until [ -S "$sock" ]; do sleep 0.05; done
}
stopFake() {
    [ -n "$fakePid" ] && kill "$fakePid" 2>/dev/null && wait "$fakePid" 2>/dev/null
    fakePid=""
}
cleanup() {
    stopFake
    [ -n "$temp" ] && rm -rf "$temp"
}
trap cleanup EXIT

# <file> <extended regex>: until a line of the (growing) file matches
waitLine() {
    until grep -qE "$2" "$1" 2>/dev/null; do sleep 0.05; done
}

dump() { # <out file> <args...>: exit code in $?
    local out=$1
    shift
    "$tui" --dump "$sock" "$@" >"$out" 2>&1
}

echo "== no socket: says so, exit 1"
rm -f "$sock"
dump "$work/missing.txt"
[ $? -eq 1 ] || fail "exit code without a socket"
expect "$work/missing.txt" "no link" "the message without a socket"

startFake

echo "== the station says what it uses"
dump "$work/get.txt" until='^tunables solder temp=320 sleep temp=210 kp=0.300 ki=0.000 kd=0.200 tau=700' \
    until='^status state=' until='^diagnostics mains=10000/10000'
[ $? -eq 0 ] || fail "tunables, status and diagnostics"

echo "== a tunable set: acked, taken"
dump "$work/set.txt" set=0:350 until='^ack seq=100 ok' until='^tunables solder temp=350'
[ $? -eq 0 ] || fail "solder temperature to 350"

echo "== out of range: clamped, and the clamped value is what is used"
dump "$work/clamp.txt" set=2:99999 until='^ack seq=100 clamped to range' until='^tunables .* kp=5.000'
[ $? -eq 0 ] || fail "kp clamped"
dump "$work/kp.txt" set=2:300 until='^tunables .* kp=0.300'
[ $? -eq 0 ] || fail "kp back"

echo "== no such tunable"
dump "$work/index.txt" set=99:1 until='^ack seq=100 unknown tunable'
[ $? -eq 0 ] || fail "bad index"

echo "== the tip heats up to its setpoint"
dump "$work/heat.txt" until='^telemetry tip=3[2-9][0-9]\.[0-9] .* set=350\.0'
[ $? -eq 0 ] || fail "tip at 350"

echo "== into the holder: Sleep at the sleep temperature, out again: Solder"
kill -USR1 "$fakePid"
dump "$work/sleep.txt" until='^status state=SLEEP .*flags=0x0[23]' until='^telemetry .* set=210\.0'
[ $? -eq 0 ] || fail "sleep"
kill -USR1 "$fakePid"
dump "$work/solder.txt" until='^status state=SOLDER'
[ $? -eq 0 ] || fail "solder again"

echo "== stored: kp and the solder temperature survive a reset, an unstored kd does not"
dump "$work/store.txt" set=2:450 store until='^ack seq=101 ok' until='^tunables .* kp=0.450 .* unsaved=-'
[ $? -eq 0 ] || fail "kp stored"
dump "$work/unsaved.txt" set=4:900 until='^tunables .* kd=0.900 .* unsaved=kd'
[ $? -eq 0 ] || fail "kd in use, not stored"

echo "== a station reset: seen, and the station starts with what is stored"
"$tui" --dump "$sock" until='^station reset' >"$work/reset.txt" 2>&1 &
dumpPid=$!
waitLine "$work/reset.txt" '^status '
kill -USR2 "$fakePid"
wait "$dumpPid"
[ $? -eq 0 ] || fail "reset seen"
dump "$work/stored.txt" until='^tunables solder temp=350 .* kp=0.450 .* kd=0.200 .* unsaved=-'
[ $? -eq 0 ] || fail "the stored set after the reset"

echo "== a stored bad set undone: the defaults into the flash survive a reset, the solder temp stays"
dump "$work/bad.txt" set=4:4000 store until='^ack seq=101 ok' until='^tunables .* kd=4.000 .* unsaved=-'
[ $? -eq 0 ] || fail "a bad kd stored"
dump "$work/factory.txt" defaults store until='^ack seq=101 ok' \
    until='^tunables solder temp=350 .* kp=0.300 .* kd=0.200 .* unsaved=-'
[ $? -eq 0 ] || fail "the defaults stored"
"$tui" --dump "$sock" until='^station reset' >"$work/reset2.txt" 2>&1 &
dumpPid=$!
waitLine "$work/reset2.txt" '^status '
kill -USR2 "$fakePid"
wait "$dumpPid"
[ $? -eq 0 ] || fail "second reset seen"
dump "$work/after.txt" until='^tunables solder temp=350 .* kp=0.300 .* kd=0.200 .* unsaved=-'
[ $? -eq 0 ] || fail "the defaults after the reset"

echo "== the screen, rendered once"
"$tui" --render "$sock" 140x40 until='^telemetry' >"$work/screen.txt" 2>&1
[ $? -eq 0 ] || fail "render"
expect "$work/screen.txt" "JBC station remote" "the title"
expect "$work/screen.txt" "live" "the link live"
expect "$work/screen.txt" "TUNING" "the tuning card"
expect "$work/screen.txt" "solder temp +350 °C" "a tunable row"
expect "$work/screen.txt" "all stored" "the store state"
expect "$work/screen.txt" "DIAGNOSTICS" "the diagnostics card"
expect "$work/screen.txt" "mains +50.00 Hz" "the mains frequency"
stopFake

echo "== latched: SHUTDOWN, every change refused, Get still answered"
startFake latch=1
dump "$work/latch.txt" until='^status state=SHUTDOWN .*flags=0x0[89].* latchIn=0'
[ $? -eq 0 ] || fail "latched"
dump "$work/refused.txt" set=0:300 until='^ack seq=100 refused \(latched\)'
[ $? -eq 0 ] || fail "set refused while latched"
stopFake

if [ $failures -ne 0 ]; then
    echo "$failures failure(s); logs in $work"
    exit 1
fi
echo "all passed"
