#include "Application.hpp"
#include "remote/Remote.hpp"

#include <kvasir/StartUp/StartUp.hpp>

int main() {
    // the boot line (version, reset cause, what the run before ended in) came from Boot::Report before main
    auto const fuses = Kvasir::Fuses::setFuseBits(HW::fuseBits(), false);
    if(!fuses) {
        UC_LOG_E("fuse bits not written: {} (tried again at the next boot)", fuses.error());
    }
    HW::Watchdog::arm();
    Application<Clock> app;
    // the remote (tui/): the probe's duplex channel and SERCOM1's UART, served from this loop
    Remote::Station<Clock, Application<Clock>> remote{app, fuses.has_value()};

    auto next = Clock::time_point{};

    while(Clock::now() < Clock::time_point{} + timingConfig::maxRuntime) {
        auto const turnStart = Clock::now();
        app.turn();
        remote.turn(app, false);
        // a write the NVM controller refused, retried from here: this loop calls its drivers' handlers itself and never
        // runs Startup::run<Kvasir::Hook::MainLoop>(), which is where a listed store's maintain() would run
        TempEeeprom::turn();
        Runtime::StackProtector::turn();
        Runtime::Guard::turn();   // the boot guard: a run this long is healthy
        HW::Watchdog::feed();
        if(Clock::now() > next) {
            next += 1s;
            UC_LOG_D("alive...");
        }
        remote.noteTurn(Clock::now() - turnStart);
    }
    // The ADC interrupt is what switches the heater on (Temp::adcCallback): stop it first, then
    // every output off.
    apply(Kvasir::Nvic::makeDisable(Kvasir::Interrupt::adc));
    Kvasir::Startup::SafeStateOf<::Startup>{}();
    UC_LOG_C("Failed...");
    while(true) {
        app.fault();
        remote.turn(app, true);   // the remote still sees the station, every command refused
        HW::Watchdog::feed();     // stay here showing the error: a reset would heat again
    }
}

template struct Kvasir::Startup::Start<Startup>;
