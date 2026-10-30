#pragma once
// The station's end of the remote (tui/, jbc_tui): both links - the probe's duplex channel and
// SERCOM1's UART - served from the main loop, one report a link at a time. What goes out, by
// priority: the Ack of a command, the Tunables after a change or a Get, the Status when it changes
// (at least every StatusPeriod), the Diagnostics every DiagnosticsPeriod, the Telemetry every
// TelemetryPeriod. A link nobody reads drops its reports (Link.hpp); the station never waits for a
// host, and nothing a host sends can switch the heater on or reach the 1 h latch.
#include "../ApplicationConfig.hpp"
#include "../TimingConfig.hpp"
#include "Handler.hpp"
#include "Link.hpp"

#include <algorithm>
#include <chip/PM.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace Remote {

using ProbeLink = Link<ProbeTransport<HW::ComBackend::DuplexChannel<0>>>;
using UartLink  = Link<UartTransport<RemoteUart>>;

// The defaults the remote shows are the firmware's own.
static_assert(Infos[kpMilli].fallback == 300 && HeaterGains.kp == Float_t(0.3));
static_assert(Infos[kiMilli].fallback == 0 && HeaterGains.ki == Float_t(0));
static_assert(Infos[kdMilli].fallback == 200 && HeaterGains.kd == Float_t(0.2));
static_assert(Infos[tauMs].fallback == 700 && HeaterGains.tau == Float_t(0.7));
static_assert(std::chrono::seconds{Infos[standbyS].fallback} == timingConfig::standbyTimeOut);
static_assert(std::chrono::seconds{Infos[sleepDelayS].fallback} == timingConfig::sleepTimeOut);

inline Protocol::ResetCause resetCause() {
    using C = Kvasir::PM::ResetCause;
    switch(Kvasir::PM::resetCause()) {
    case C::por:   return Protocol::ResetCause::por;
    case C::bod12: return Protocol::ResetCause::bod12;
    case C::bod33: return Protocol::ResetCause::bod33;
    case C::ext:   return Protocol::ResetCause::ext;
    case C::wdt:   return Protocol::ResetCause::wdt;
    case C::syst:  return Protocol::ResetCause::syst;
    default:       return Protocol::ResetCause::unknown;
    }
}

template<typename Clock, typename App>
class Station {
public:
    static constexpr auto TelemetryPeriod   = std::chrono::milliseconds{100};
    static constexpr auto StatusPeriod      = std::chrono::seconds{1};
    static constexpr auto DiagnosticsPeriod = std::chrono::seconds{1};

    /// Takes what the EEPROM emulation holds (Application took the solder temperature from it already).
    Station(App& app,
            bool fusesOk)
      : values_{sanitized(TempEeeprom::value().values)}
      , fusesOk_{fusesOk}
      , resetCause_{resetCause()} {
        apply(app, false);
    }

    /// One main-loop turn. `latched`: main() left its loop at maxRuntime - commands are refused.
    void turn(App& app,
              bool latched) {
        auto const now = Clock::now();
        latched_ = latched || app.state() == static_cast<std::size_t>(Protocol::State::shutdown);

        // the knob commits the solder temperature too: the remote shows what is in use
        if(auto const c = static_cast<std::int32_t>(app.solderTemp_); c != values_[solderC]) {
            values_[solderC] = c;
            pending_ |= wantTunables;
        }

        serve(probe_, app);
        serve(uart_, app);

        if(now >= nextTelemetry_) {
            using namespace sc::literals;
            nextTelemetry_ = now + TelemetryPeriod;
            telemetry_     = telemetry(app);
            pending_ |= wantTelemetry;
            UC_LOG_T(
              "tip {} set {} duty {}/{}",
              uc_log::metric<"tip"_sc, "°C"_sc, "heater"_sc>(telemetry_.tipDeciC / 10),
              uc_log::metric<"setpoint"_sc, "°C"_sc, "heater"_sc>(telemetry_.setpointDeciC / 10),
              uc_log::metric<"heated"_sc, ""_sc, "heater"_sc>(telemetry_.heatedCycles),
              telemetry_.halfCycles);
        }
        auto const s = status(app, now);
        if(!sameStatus(s, status_) || now >= nextStatus_) {
            status_     = s;
            nextStatus_ = now + StatusPeriod;
            pending_ |= wantStatus;
        }
        if(now >= nextDiagnostics_) {
            nextDiagnostics_ = now + DiagnosticsPeriod;
            pending_ |= wantDiagnostics;
        }
        // what came up this turn goes to both links
        probe_.want |= pending_;
        uart_.want |= pending_;
        pending_ = 0;
        send(probe_, app);
        send(uart_, app);
    }

    /// The main loop's turn time, for Diagnostics.
    void noteTurn(typename Clock::duration d) {
        auto const us = static_cast<std::uint32_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(d).count());
        maxTurnUs_ = std::max(maxTurnUs_, us);
    }

private:
    enum Want : std::uint8_t {
        wantAck         = 1U << 0,
        wantTunables    = 1U << 1,
        wantStatus      = 1U << 2,
        wantDiagnostics = 1U << 3,
        wantTelemetry   = 1U << 4,
    };

    static std::int16_t deci(Float_t c) {
        return static_cast<std::int16_t>(std::lround(c * Float_t(10)));
    }

    /// A link, what it still has to send, and the answer to its last command.
    template<typename L>
    struct Side {
        L             link{};
        std::uint8_t  want{};
        Protocol::Ack ack{};
    };

    template<typename L>
    void serve(Side<L>& side,
               App&     app) {
        auto const command = side.link.receive();
        if(!command) { return; }
        auto out = handle(*command, values_, latched_);
        side.want |= wantAck;
        if(out.store) {
            TempEeeprom::value().values = values_;
            // a refusal stays pending and is retried by TempEeeprom::turn() in main's loop
            if(!TempEeeprom::writeValue()) { out.ack.code = Protocol::Ack::Code::storePending; }
            pending_ |= wantTunables;
        }
        side.ack = out.ack;
        if(out.sendAll) {
            side.want |= wantTunables | wantStatus | wantDiagnostics;
            return;
        }
        if(out.clearCounters) {
            probe_.link.clearCounters();
            uart_.link.clearCounters();
            maxTurnUs_ = 0;
        }
        if(out.changed) {
            apply(app, out.solderChanged);
            pending_ |= wantTunables;
        }
    }

    void apply(App& app,
               bool solderChanged) {
        if(solderChanged) { app.setSolderTemp(static_cast<std::uint16_t>(values_[solderC])); }
        app.setSleepTemp(static_cast<std::uint16_t>(values_[sleepC]));
        app.setTimeouts(std::chrono::seconds{values_[standbyS]},
                        std::chrono::seconds{values_[sleepDelayS]});
        if(gainsOf(values_) != gainsOf(applied_)) {
            app.heaterManager.setGains(Kvasir::Control::PidGains<Float_t>{
              .kp     = milli(values_[kpMilli]),
              .ki     = milli(values_[kiMilli]),
              .kd     = milli(values_[kdMilli]),
              .tau    = milli(values_[tauMs]),
              .outMin = HeaterGains.outMin,
              .outMax = HeaterGains.outMax,
            });
        }
        applied_ = values_;
    }

    static Float_t milli(std::int32_t v) { return static_cast<Float_t>(v) / Float_t(1000); }

    static std::array<std::int32_t,
                      4>
    gainsOf(Values const& v) {
        return {v[kpMilli], v[kiMilli], v[kdMilli], v[tauMs]};
    }

    template<typename L>
    void send(Side<L>& side,
              App&     app) {
        auto& link = side.link;
        if(side.want == 0 || !link.ready()) { return; }
        auto const next
          = static_cast<std::uint8_t>(side.want & -side.want);   // lowest bit: highest priority
        side.want = static_cast<std::uint8_t>(side.want & ~next);
        switch(next) {
        case wantAck: link.send(Protocol::Report{side.ack}); break;
        case wantTunables:
            link.send(Protocol::Report{
              Protocol::Tunables{values_, TempEeeprom::value().values}
            });
            break;
        case wantStatus:      link.send(Protocol::Report{status_}); break;
        case wantDiagnostics: link.send(Protocol::Report{diagnostics(app)}); break;
        default:              link.send(Protocol::Report{telemetry_}); break;
        }
    }

    Protocol::Telemetry telemetry(App& app) {
        auto const w = app.heaterManager.take();
        auto const n = w.samples;
        return Protocol::Telemetry{
          .tipDeciC
          = n == 0 ? deci(app.heaterManager.temp()) : deci(w.sum / static_cast<Float_t>(n)),
          .tipMinDeciC   = n == 0 ? deci(app.heaterManager.temp()) : deci(w.min),
          .tipMaxDeciC   = n == 0 ? deci(app.heaterManager.temp()) : deci(w.max),
          .setpointDeciC = deci(w.setPoint),
          .rawTemp       = w.rawTemp,
          .rawCurrent    = w.rawCurrent,
          .pidCenti      = static_cast<std::uint16_t>(
            std::lround(std::clamp(w.pidOut, Float_t(0), Float_t(10)) * 100)),
          .halfCycles   = n,
          .heatedCycles = w.heated,
        };
    }

    Protocol::Status status(App&                       app,
                            typename Clock::time_point now) const {
        auto const&  t      = app.heaterManager;
        auto const   uptime = now - typename Clock::time_point{};
        auto const   left   = timingConfig::maxRuntime - uptime;
        std::uint8_t flags  = 0;
        if(t.heaterOn) { flags |= Protocol::Flag::heaterOn; }
        if(t.tip1Sleep) { flags |= Protocol::Flag::tipInSleep; }
        if(t.tip1Remove) { flags |= Protocol::Flag::tipRemoved; }
        if(latched_) { flags |= Protocol::Flag::latched; }
        if(app.preSetTemp_ != app.solderTemp_) { flags |= Protocol::Flag::presetDirty; }
        return Protocol::Status{
          .state    = static_cast<Protocol::State>(app.state()),
          .adc      = static_cast<Protocol::AdcState>(t.s),
          .flags    = flags,
          .ledR     = app.led_.r,
          .ledG     = app.led_.g,
          .ledB     = app.led_.b,
          .presetC  = static_cast<std::uint16_t>(app.preSetTemp_),
          .uptimeMs = static_cast<std::uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(uptime).count()),
          .latchInS = latched_ || left.count() <= 0
                      ? 0U
                      : static_cast<std::uint32_t>(
                          std::chrono::duration_cast<std::chrono::seconds>(left).count()),
        };
    }

    /// Everything but the clocks: a Status goes out at once when one of these changes.
    static bool sameStatus(Protocol::Status const& a,
                           Protocol::Status const& b) {
        return a.state == b.state && a.adc == b.adc && a.flags == b.flags && a.ledR == b.ledR
            && a.ledG == b.ledG && a.ledB == b.ledB && a.presetC == b.presetC;
    }

    Protocol::Diagnostics diagnostics(App& app) const {
        auto const us = [](auto d) {
            return static_cast<std::uint16_t>(std::min<std::int64_t>(
              std::chrono::duration_cast<std::chrono::microseconds>(d).count(),
              0xFFFF));
        };
        std::int16_t trafo = Protocol::Diagnostics::Unknown;
        if(auto const t = app.tempSensTransformer.temperature()) {
            trafo = static_cast<std::int16_t>(t->numerical_value_in(t->unit) / 100);
        }
        return Protocol::Diagnostics{
          .mainsHalfPeriodUs = {us(ZCD::avgE1), us(ZCD::avgE2)},
          .transformerDeciC  = trafo,
          .resetCause        = resetCause_,
          .fusesOk           = fusesOk_,
          .eepromPending     = TempEeeprom::pending(),
          .eepromC           = static_cast<std::uint16_t>(TempEeeprom::value().values[solderC]),
          .maxTurnUs         = maxTurnUs_,
          .adcConversions    = app.heaterManager.conversions,
          .probe             = probe_.link.counters(),
          .uart              = uart_.link.counters(),
        };
    }

    Side<ProbeLink>      probe_{};
    Side<UartLink>       uart_{};
    Values               values_{defaults()};
    Values               applied_{defaults()};
    Protocol::Telemetry  telemetry_{};
    Protocol::Status     status_{};
    bool                 fusesOk_;
    Protocol::ResetCause resetCause_;
    bool                 latched_{};
    std::uint8_t         pending_{wantTunables};   // the station says what it uses at boot
    std::uint32_t        maxTurnUs_{};

    typename Clock::time_point nextTelemetry_{};
    typename Clock::time_point nextStatus_{};
    typename Clock::time_point nextDiagnostics_{};
};

}   // namespace Remote
