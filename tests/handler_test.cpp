// remote/Handler.hpp and remote/Tunables.hpp: clamping, the latch refusing, what each command asks
// the firmware to do.
#include "remote/Handler.hpp"

#include "Check.hpp"

namespace { using Code = Protocol::Ack::Code; }   // namespace

int main() {
    using namespace Protocol;

    Test::testCase("defaults are inside their ranges");
    {
        auto const d  = Remote::defaults();
        bool       ok = true;
        for(std::size_t i = 0; i < Remote::Count; ++i) { ok = ok && Remote::inRange(i, d[i]); }
        Test::check(ok, "every default in range");
        Test::check(Remote::Infos[Remote::solderC].max <= 512,
                    "remote solder max within the knob's range");
    }

    Test::testCase("set, in range");
    {
        auto       v   = Remote::defaults();
        auto const out = Remote::handle(SetTunable{1, Remote::kpMilli, 450}, v, false);
        Test::check(out.ack.seq == 1 && out.ack.code == Code::ok, "ok");
        Test::check(out.changed && !out.solderChanged && v[Remote::kpMilli] == 450, "kp taken");
    }

    Test::testCase("set, clamped both ways");
    {
        auto v   = Remote::defaults();
        auto out = Remote::handle(SetTunable{2, Remote::solderC, 9'999}, v, false);
        Test::check(out.ack.code == Code::clamped && v[Remote::solderC] == Remote::MaxRemoteC,
                    "solder capped at MaxRemoteC");
        Test::check(out.changed && out.solderChanged, "solder change reported");
        out = Remote::handle(SetTunable{3, Remote::tauMs, -5}, v, false);
        Test::check(out.ack.code == Code::clamped
                      && v[Remote::tauMs] == Remote::Infos[Remote::tauMs].min,
                    "tau raised to its min");
    }

    Test::testCase("set to the value it has: ok, nothing changed");
    {
        auto       v   = Remote::defaults();
        auto const out = Remote::handle(SetTunable{4, Remote::sleepC, v[Remote::sleepC]}, v, false);
        Test::check(out.ack.code == Code::ok && !out.changed, "no change");
    }

    Test::testCase("bad index");
    {
        auto       v   = Remote::defaults();
        auto const out = Remote::handle(SetTunable{5, Remote::Count, 1}, v, false);
        Test::check(out.ack.code == Code::badIndex && !out.changed && v == Remote::defaults(),
                    "refused");
    }

    Test::testCase("latched: every change refused, Get still answered");
    {
        auto v   = Remote::defaults();
        auto out = Remote::handle(SetTunable{6, Remote::solderC, 200}, v, true);
        Test::check(out.ack.code == Code::refused && !out.changed && v == Remote::defaults(),
                    "set refused");
        out = Remote::handle(ResetTunables{7}, v, true);
        Test::check(out.ack.code == Code::refused, "reset refused");
        out = Remote::handle(Get{8}, v, true);
        Test::check(out.ack.code == Code::ok && out.sendAll, "get answered");
    }

    Test::testCase("reset: RAM tunables back, solder temperature kept");
    {
        auto v              = Remote::defaults();
        v[Remote::solderC]  = 350;
        v[Remote::kdMilli]  = 1;
        v[Remote::standbyS] = 60;
        auto const out      = Remote::handle(ResetTunables{9}, v, false);
        auto const d        = Remote::defaults();
        Test::check(out.changed && !out.solderChanged, "changed, solder untouched");
        Test::check(v[Remote::solderC] == 350 && v[Remote::kdMilli] == d[Remote::kdMilli]
                      && v[Remote::standbyS] == d[Remote::standbyS],
                    "values");
        Test::check(!Remote::handle(ResetTunables{10}, v, false).changed,
                    "second reset changes nothing");
    }

    Test::testCase("store: asked for, refused while latched");
    {
        auto       v   = Remote::defaults();
        auto const out = Remote::handle(StoreTunables{12}, v, false);
        Test::check(out.store && out.ack.code == Code::ok && !out.changed, "store asked");
        auto const latched = Remote::handle(StoreTunables{13}, v, true);
        Test::check(!latched.store && latched.ack.code == Code::refused, "refused while latched");
    }

    Test::testCase("a stored set is clamped on load");
    {
        Remote::Values v{};
        v[Remote::solderC] = 9'999;
        v[Remote::tauMs]   = -1;
        auto const s       = Remote::sanitized(v);
        Test::check(s[Remote::solderC] == Remote::MaxRemoteC
                      && s[Remote::tauMs] == Remote::Infos[Remote::tauMs].min,
                    "clamped");
        Test::check(Remote::sanitized(Remote::defaults()) == Remote::defaults(),
                    "defaults untouched");
    }

    Test::testCase("clear counters");
    {
        auto v = Remote::defaults();
        Test::check(Remote::handle(ClearCounters{11}, v, false).clearCounters, "asked");
    }
    return Test::finish();
}
