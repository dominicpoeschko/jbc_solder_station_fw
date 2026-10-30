// remote/Protocol.hpp: every command and report fits a frame and comes back through aglio's
// Packager; a corrupted, a split and a garbage-led frame.
#include "remote/Protocol.hpp"

#include "Check.hpp"

#include <vector>

namespace {
static_assert(Protocol::Packager::requireAlwaysFits<Protocol::Command>());
static_assert(Protocol::Packager::requireAlwaysFits<Protocol::Report>());

template<typename Variant>
bool roundTrip(Variant const& in) {
    std::vector<std::byte> buffer;
    if(!Protocol::Packager::pack(buffer, in)) { return false; }
    Variant    out{};
    auto const r = Protocol::Packager::unpack(buffer, out);
    return r && r->consumed == buffer.size() && out.index() == in.index();
}

template<typename Variant>
Variant through(Variant const& in) {
    std::vector<std::byte> buffer;
    static_cast<void>(Protocol::Packager::pack(buffer, in));
    Variant out{};
    static_cast<void>(Protocol::Packager::unpack(buffer, out));
    return out;
}
}   // namespace

int main() {
    using namespace Protocol;

    Test::testCase("every command");
    Test::check(roundTrip(Command{Get{1}}), "Get");
    Test::check(roundTrip(Command{
                  SetTunable{2, Remote::kdMilli, -7}
    }),
                "SetTunable");
    Test::check(roundTrip(Command{ResetTunables{3}}), "ResetTunables");
    Test::check(roundTrip(Command{ClearCounters{4}}), "ClearCounters");
    Test::check(roundTrip(Command{StoreTunables{5}}), "StoreTunables");

    Test::testCase("every report");
    Test::check(roundTrip(Report{Telemetry{}}), "Telemetry");
    Test::check(roundTrip(Report{Status{}}), "Status");
    Test::check(roundTrip(Report{Diagnostics{}}), "Diagnostics");
    Test::check(roundTrip(Report{
                  Tunables{Remote::defaults(), Remote::defaults()}
    }),
                "Tunables");
    {
        auto stored             = Remote::defaults();
        stored[Remote::kdMilli] = 1'234;
        auto const  out         = through(Report{
          Tunables{Remote::defaults(), stored}
        });
        auto const* t           = std::get_if<Tunables>(&out);
        Test::check(t != nullptr && t->stored[Remote::kdMilli] == 1'234
                      && t->values == Remote::defaults(),
                    "Tunables keeps both sets");
    }
    Test::check(roundTrip(Report{
                  Ack{9, Ack::Code::clamped}
    }),
                "Ack");

    Test::testCase("values survive");
    {
        auto const  out = through(Command{
          SetTunable{0xBEEF, Remote::standbyS, -2'147'483'647}
        });
        auto const* s   = std::get_if<SetTunable>(&out);
        Test::check(s != nullptr && s->seq == 0xBEEF && s->index == Remote::standbyS
                      && s->value == -2'147'483'647,
                    "SetTunable fields");
    }
    {
        Status in{};
        in.state        = State::shutdown;
        in.adc          = AdcState::waitHeaterTurnOff;
        in.flags        = Flag::latched | Flag::tipRemoved;
        in.ledR         = 10;
        in.presetC      = 512;
        in.uptimeMs     = 3'600'123;
        in.latchInS     = 0;
        auto const  out = through(Report{in});
        auto const* s   = std::get_if<Status>(&out);
        Test::check(s != nullptr && s->state == State::shutdown
                      && s->adc == AdcState::waitHeaterTurnOff
                      && s->flags == (Flag::latched | Flag::tipRemoved) && s->ledR == 10
                      && s->presetC == 512 && s->uptimeMs == 3'600'123,
                    "Status fields");
    }
    {
        Diagnostics in{};
        in.mainsHalfPeriodUs = {10'000, 9'990};
        in.transformerDeciC  = Diagnostics::Unknown;
        in.resetCause        = ResetCause::wdt;
        in.uart.badBytes     = 77;
        in.probe.framesIn    = 123'456;
        auto const  out      = through(Report{in});
        auto const* d        = std::get_if<Diagnostics>(&out);
        Test::check(d != nullptr && d->mainsHalfPeriodUs[1] == 9'990
                      && d->transformerDeciC == Diagnostics::Unknown
                      && d->resetCause == ResetCause::wdt && d->uart.badBytes == 77
                      && d->probe.framesIn == 123'456,
                    "Diagnostics fields");
    }
    {
        Telemetry in{};
        in.tipDeciC     = -12;
        in.tipMaxDeciC  = 4'500;
        in.pidCenti     = 1'000;
        in.heatedCycles = 7;
        auto const  out = through(Report{in});
        auto const* t   = std::get_if<Telemetry>(&out);
        Test::check(t != nullptr && t->tipDeciC == -12 && t->tipMaxDeciC == 4'500
                      && t->pidCenti == 1'000 && t->heatedCycles == 7,
                    "Telemetry fields");
    }

    Test::testCase("a flipped byte is refused, a split frame waits, garbage is skipped");
    {
        std::vector<std::byte> buffer;
        static_cast<void>(Packager::pack(buffer, Command{Get{5}}));
        auto broken = buffer;
        broken.back() ^= std::byte{0x01};
        Command out{};
        Test::check(!Packager::unpack(broken, out), "corrupt refused");
        auto const half = std::span<std::byte const>{buffer}.first(buffer.size() / 2);
        auto const r    = Packager::unpack(half, out);
        Test::check(!r && r.error().consumed == 0, "partial frame consumes nothing");

        // garbage in front of a frame is skipped as part of the frame's unpack (consumed covers both)
        std::vector<std::byte> noisy{std::byte{0x00}, std::byte{0xC0}, std::byte{0x55}};
        noisy.insert(noisy.end(), buffer.begin(), buffer.end());
        auto const res = Packager::unpack(noisy, out);
        Test::check(res && res->consumed == noisy.size() && std::get<Get>(out).seq == 5,
                    "frame found behind 3 bytes of garbage");
    }
    return Test::finish();
}
