# jbc_solder_station_fw: notes for Claude

The soldering station firmware (ATSAMD21, src/). Read kvasir_work/CLAUDE.md first; this file holds what is true for
this firmware only. No board on the bench: every run of it is owed (plans/refactor_analysis/sam_board_runs.md).

## The safety latch (the stated exception to "every error recovers by itself")

- **One hour after the boot the station turns the heater off and stays off until a person power-cycles it - on
  purpose** (written down 2026-10-21, refactor step 132; the behaviour predates the refactor). Two places hold it, both
  1 h (src/TimingConfig.hpp): the state machine's `Shutdown` (`failTimeOut`, src/Application.hpp: heater 0, LED red,
  "To leave this state you must reboot the system"), and `main()` after `maxRuntime`: it leaves its loop, disables the
  ADC interrupt (it is what switches the heater on, `Temp::adcCallback`), runs the safe state
  (`HW::PinConfig::safeState`, every heater and tip switch off, the `Hook::SafeState` entry every fatal path runs too),
  logs `Failed...` and then only shows the error and feeds the watchdog. The reason is the heater: a reset - by the
  watchdog, a reboot or a retry - would start heating again with nobody at the iron ("a reset would heat again", the
  comment at that loop). The README's "Safety features" list it as "station is on longer than 1 hour -> turn off".
  kvasir_work/CLAUDE.md's rule wants such a latch stated; this is it. Do not "fix" it into a recovery.
- **Every build before 2cc02a7 (2026-10-30) can leave the heater ON for good at the 1 h exit** (found 2026-10-30
  from a field report, "blows the fuse, stuck with the heater on"): its main ran `HW::Fault_CleanUpAction();` -
  a struct, so a temporary and no call - and left the ADC interrupt (which switches the heater on) running while
  nothing polled `Temp::handler()` (the only switch-off) any more, with no watchdog. Heater on at that moment, or
  turned on by the next conversion, stays on: odds about the duty, LED red. A station that fails ~1 h after power-on
  runs such a build. 2cc02a7 disables the ADC interrupt and really runs the safe state. The friend's station fails FAST, though, so
  this is not the whole story; the guards added the same day for the other ways (constants: `safetyConfig` in
  src/TimingConfig.hpp, first guesses, no station run - sam_board_runs.md "jbc heater guards"): `safeState` clears
  `b10` too (`heat<true>()` drives it with `tipComSw`); the switch-off no longer depends on the main loop alone -
  the ADC interrupt switches off when the next crossing's conversion finds the heater on (`lateSwitchOffs`); a
  current conversion that never comes times out (`currentTimeOuts`); the README's transformer limit holds the heater
  off above 50 °C, back below 45 °C (recovers by itself; no MAX31820 reading holds nothing).
- **The second stated latch: "tip did not warm"** (2026-10-30): at full power the reading must rise 20 °C in every
  15 s (`Temp::watchFullPower`), else `runaway` -> `Shutdown` until a power cycle, status flag `tipNotHeating`.
  A sensor that reads low asked for full power for ever before (until the 5 min standby); a retry would do it again,
  so it latches like the 1 h limit. Still open: the ADC runs at 48 MHz / 8 = 6 MHz, D21 max f_CLK_ADC 2.1 MHz
  (DS40001882L 37.11.4) - the temperature factor was tuned on it, so changing it needs a station; the measured
  current (`lastCurrentVal`) is checked against nothing.
- The remote (below) cannot reach the latch either: `failTimeOut`/`maxRuntime` are not tunables, a latched station
  refuses every set (`Ack refused`), and no command switches the heater.
- Everything else recovers by itself: a panic or a fault runs the safe state, records, and the watchdog (2 s, no longer
  fed - `Panic::PanicReaction` is `OnPanic::Wait`, ApplicationConfig.hpp) resets the board; the idle standby
  (`standbyTimeOut`, 5 min) is left by a knob or a sleep/remove event (`Standby` -> `Sleep`), the tip's remove and sleep
  states by their signal; a refused EEPROM store is retried from the loop (`TempEeeprom::turn()`); fuse bits not written
  are tried again at the next boot.

## The remote (src/remote/, host/, tui/; written 2026-10-30)

- **Wire**: `src/remote/Protocol.hpp`, aglio `Packager` frames (start 0x1BC0, u8 length, CRC-16/CCITT-FALSE in
  software, MaxSize 96), shared by the firmware, the host and the tests. Plain integers only (0.1 °C, gains x1000).
  Commands: `Get`, `SetTunable{seq, index, value}`, `ResetTunables`, `StoreTunables`, `ClearCounters`; reports: `Telemetry` (10 Hz,
  aggregated in the ADC ISR: `Temp::Window`/`take()`), `Status` (on change, >= 1 s; `uptimeMs` going back = a reset),
  `Diagnostics` (1 s), `Tunables{values, stored}`, `Ack{seq}`. What may be set and its range: `src/remote/Tunables.hpp` `Infos`
  (static_asserts in `Remote.hpp` pin the defaults to `HeaterGains`/`TimingConfig.hpp`). **Storing** (dominic,
  2026-10-30: an explicit store): a set is used at once and lives in RAM; `StoreTunables` (the tui's `s`) writes all
  of them to the EEPROM emulation (`TempEeeprom`, record `StoredSettings{Remote::Values}`), which the station loads at
  boot (`Remote::sanitized`: clamped; no valid record = the defaults). The solder temperature alone is stored on every
  change, remote or knob press, as before. A stored bad set is undone with the tui's `F F` (or its "Defaults → flash" button,
  `--dump defaults store`): `ResetTunables` then `StoreTunables` - every tunable but the solder temperature. No keepalive: the station never depends on a host (dominic).
- **Transports, both at once**: the RTT duplex channel "control" (256 up / 128 down, `kvasir_config.hpp`; the printer
  serves it as `build/rtt_log/<target>/duplex.0.sock`) and SERCOM1 USART 115200 8N1, TX PA18 = SERCOM1/PAD[2], RX PA19
  = PAD[3] (D21 Table 7-1, mux C), DMA channel 0 (`RemoteUart`, ApplicationConfig.hpp). The SAM USART sends by DMA
  from the caller's bytes: `Remote::Link` packs the next report only when `Transport::idle()` (txIdle) - packing over
  bytes still on their way would corrupt the frame.
- **Run it**: `just log debug` + `just tui`, or `just tui /dev/ttyUSB0`; without a board `just fake` + `just tui
  /tmp/jbc_fake.sock` (SIGUSR1 = holder, SIGUSR2 = reset). Scripts: `just tui --dump [path] set=I:V until=REGEX`;
  the screen as text: `just tui --render [path] 150x44 until=REGEX`. Tests: `just test` (protocol, handler, model,
  graphs), `just tui-test` (e2e against jbc_fake).
- **Cost** (2026-10-30, clang): flash debug 30.5 -> 38.0 KB, release 22.7 -> 29.8 KB; RAM: release had no RTT block
  before (no log strings) and now holds it (~5 KB: the 512/4096 log rings + 384 duplex) - stack left 10.0 KB release,
  9.8 KB debug.
- Every hardware run of it is owed (`plans/refactor_analysis/sam_board_runs.md`, "jbc remote").
