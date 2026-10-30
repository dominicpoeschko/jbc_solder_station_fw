// jbc_fake: a stand-in for the station on a unix socket, for running jbc_tui without hardware. It
// is the firmware's command handler (remote/Handler.hpp) and report schedule behind a socket that
// behaves like uc_log_printer's duplex.0.sock; the station around it is a simulation: a tip that
// heats with the heated half-cycles of a 50 Hz mains and cools to 25 °C, a PD loop like Temp's
// (ten half-cycles a burst), the Sleep/Solder states and the latch.
//
//   jbc_fake <socket> [latch=SECONDS (3600)] [fast=FACTOR (1)]
//
// SIGUSR1 puts the iron into its holder and takes it out again (Sleep <-> Solder); SIGUSR2 resets
// the station (uptime from 0, the tunables back to what its "flash" holds - StoreTunables writes
// it, a solder temperature change too). One client at a time, as the printer. `fast` runs the
// simulation that many times faster than the wall clock.
#include "Frames.hpp"
#include "remote/Handler.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <poll.h>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

std::sig_atomic_t volatile holderToggles = 0;
std::sig_atomic_t volatile resets        = 0;
std::sig_atomic_t volatile quit          = 0;

/// The station: its state, its tip and its loop, one mains half-cycle (10 ms) a step.
struct Station {
    Remote::Values  values{Remote::defaults()};
    Protocol::State state{Protocol::State::sleep};
    bool            inHolder{false};
    double          tip{25.0};
    double          setpoint{};
    double          pidOut{};
    double          lastTip{25.0};
    double          derivative{};
    int             onLeft{};
    int             burstLeft{};
    bool            heaterOn{};
    std::uint32_t   halfCycles{};   // since boot: the station's clock, 10 ms each
    std::uint32_t   lastHandlingAt{};
    std::uint32_t   enteredSleepAt{};
    std::uint32_t   latchAt{360'000};
    std::uint32_t   conversions{};

    // the Telemetry window
    double       sum{};
    double       min{};
    double       max{};
    std::uint8_t samples{};
    std::uint8_t heated{};

    [[nodiscard]] bool latched() const { return state == Protocol::State::shutdown; }

    [[nodiscard]] double gain(Remote::Index i) const { return values[i] / 1000.0; }

    void step() {
        ++halfCycles;
        ++conversions;
        auto const s
          = [](std::int32_t seconds) { return static_cast<std::uint32_t>(seconds) * 100U; };
        if(halfCycles >= latchAt) { state = Protocol::State::shutdown; }
        switch(state) {
        case Protocol::State::sleep:
            // before the delay the setpoint stays what it was (Application's Sleep)
            if(halfCycles - enteredSleepAt >= s(values[Remote::sleepDelayS])) {
                setpoint = values[Remote::sleepC];
            }
            if(!inHolder) {
                state          = Protocol::State::solder;
                lastHandlingAt = halfCycles;
            } else if(halfCycles - lastHandlingAt > s(values[Remote::standbyS])) {
                state = Protocol::State::standby;
            }
            break;
        case Protocol::State::solder:
            setpoint = values[Remote::solderC];
            if(inHolder) {
                state          = Protocol::State::sleep;
                enteredSleepAt = halfCycles;
                lastHandlingAt = halfCycles;
            } else if(halfCycles - lastHandlingAt > s(values[Remote::standbyS])) {
                state = Protocol::State::standby;
            }
            break;
        case Protocol::State::standby:
            setpoint = 0;   // left by a handling event (toggleHolder)
            break;
        default: setpoint = 0; break;
        }

        // Temp::adcCallback: a burst of ten half-cycles, the PD output says how many heat
        if(burstLeft == 0) {
            auto const       error = setpoint - tip;
            auto const       tau   = gain(Remote::tauMs);
            constexpr double dt    = 0.1;
            derivative
              = (-(2 * gain(Remote::kdMilli) * (tip - lastTip)) + (2 * tau - dt) * derivative)
              / (2 * tau + dt);
            lastTip   = tip;
            pidOut    = std::clamp(gain(Remote::kpMilli) * error + derivative, 0.0, 10.0);
            onLeft    = latched() ? 0 : static_cast<int>(std::ceil(pidOut - 0.2));
            burstLeft = 10;
        }
        heaterOn = onLeft > 0;
        if(heaterOn) { --onLeft; }
        --burstLeft;

        // the tip: about 4 °C a heated half-cycle into 25 °C surroundings, some 30 s to cool
        tip += (heaterOn ? 4.0 : 0.0) - (tip - 25.0) * 0.003;

        if(samples == 0) {
            min = tip;
            max = tip;
        }
        if(samples != 0xFF) {
            min = std::min(min, tip);
            max = std::max(max, tip);
            sum += tip;
            ++samples;
            if(heaterOn) { ++heated; }
        }
    }

    /// The holder signal changed (SIGUSR1): a handling event, as the firmware counts one.
    void toggleHolder() {
        inHolder       = !inHolder;
        lastHandlingAt = halfCycles;
        if(state == Protocol::State::standby) {
            state          = Protocol::State::sleep;
            enteredSleepAt = halfCycles;
        }
    }

    static std::int16_t deci(double c) { return static_cast<std::int16_t>(std::lround(c * 10)); }

    Protocol::Telemetry telemetry() {
        Protocol::Telemetry t{
          .tipDeciC      = deci(samples == 0 ? tip : sum / samples),
          .tipMinDeciC   = deci(samples == 0 ? tip : min),
          .tipMaxDeciC   = deci(samples == 0 ? tip : max),
          .setpointDeciC = deci(setpoint),
          .rawTemp       = static_cast<std::uint16_t>(std::clamp(tip / 0.01115, 0.0, 65535.0)),
          .rawCurrent    = static_cast<std::uint16_t>(heaterOn ? 31'000 : 120),
          .pidCenti      = static_cast<std::uint16_t>(std::lround(pidOut * 100)),
          .halfCycles    = samples,
          .heatedCycles  = heated,
        };
        sum     = 0;
        samples = 0;
        heated  = 0;
        return t;
    }

    [[nodiscard]] Protocol::Diagnostics diagnostics() const {
        return Protocol::Diagnostics{
          .mainsHalfPeriodUs = {10'000, 10'000},
          .transformerDeciC  = static_cast<std::int16_t>(300 + tip / 20),
          .resetCause        = Protocol::ResetCause::por,
          .fusesOk           = true,
          .eepromPending     = false,
          .eepromC           = static_cast<std::uint16_t>(
            values[Remote::solderC]), // the solder temp is stored at once
          .maxTurnUs      = 850,
          .adcConversions = conversions * 2,
          .probe          = {},
          .uart           = {},
        };
    }

    [[nodiscard]] Protocol::Status status() const {
        std::uint8_t flags = 0;
        if(heaterOn) { flags |= Protocol::Flag::heaterOn; }
        if(inHolder) { flags |= Protocol::Flag::tipInSleep; }
        if(latched()) { flags |= Protocol::Flag::latched; }
        auto const led = latched()                         ? std::array<std::uint8_t, 3>{10, 0, 0}
                       : heaterOn                          ? std::array<std::uint8_t, 3>{1, 0, 0}
                       : state == Protocol::State::sleep   ? std::array<std::uint8_t, 3>{0, 3, 0}
                       : state == Protocol::State::standby ? std::array<std::uint8_t, 3>{1, 1, 0}
                                                           : std::array<std::uint8_t, 3>{0, 0, 0};
        return Protocol::Status{
          .state    = state,
          .adc      = Protocol::AdcState::waitTemp,
          .flags    = flags,
          .ledR     = led[0],
          .ledG     = led[1],
          .ledB     = led[2],
          .presetC  = static_cast<std::uint16_t>(values[Remote::solderC]),
          .uptimeMs = halfCycles * 10U,
          .latchInS = latched() ? 0U : (latchAt - halfCycles) / 100U,
        };
    }
};

int listenOn(std::string const& path) {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if(path.size() >= sizeof(address.sun_path)) { return -1; }
    std::memcpy(address.sun_path, path.data(), path.size());
    ::unlink(path.c_str());
    int const fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if(fd < 0 || ::bind(fd, reinterpret_cast<sockaddr const*>(&address), sizeof(address)) != 0
       || ::listen(fd, 4) != 0)
    {
        return -1;
    }
    return fd;
}

void sendReport(int                     fd,
                Protocol::Report const& r) {
    if(fd < 0) { return; }
    auto const frame = Host::frame(r);
    static_cast<void>(::send(fd, frame.data(), frame.size(), MSG_NOSIGNAL));
}

}   // namespace

int main(int    argc,
         char** argv) {
    if(argc < 2) {
        std::fprintf(stderr, "usage: jbc_fake <socket> [latch=SECONDS] [fast=FACTOR]\n");
        return 2;
    }
    std::string const path{argv[1]};
    std::uint32_t     latchSeconds = 3'600;
    int               fast         = 1;
    for(int i = 2; i < argc; ++i) {
        std::string_view const arg{argv[i]};
        if(arg.starts_with("latch=")) {
            latchSeconds = static_cast<std::uint32_t>(std::atoi(arg.substr(6).data()));
        } else if(arg.starts_with("fast=")) {
            fast = std::max(1, std::atoi(arg.substr(5).data()));
        } else {
            std::fprintf(stderr, "unknown: %s\n", argv[i]);
            return 2;
        }
    }
    std::signal(SIGUSR1, [](int) { holderToggles = holderToggles + 1; });
    std::signal(SIGUSR2, [](int) { resets = resets + 1; });
    std::signal(SIGTERM, [](int) { quit = 1; });
    std::signal(SIGINT, [](int) { quit = 1; });

    if(path.size() >= sizeof(sockaddr_un::sun_path)) {
        std::fprintf(stderr,
                     "%s: longer than a unix socket path may be (%zu bytes)\n",
                     path.c_str(),
                     sizeof(sockaddr_un::sun_path) - 1);
        return 1;
    }
    int const listener = listenOn(path);
    if(listener < 0) {
        std::fprintf(stderr, "cannot listen on %s: %s\n", path.c_str(), std::strerror(errno));
        return 1;
    }
    std::printf("jbc_fake on %s\n", path.c_str());
    std::fflush(stdout);

    Remote::Values flash = Remote::defaults();   // the EEPROM emulation: survives a reset
    auto const     boot  = [&] {
        Station s;
        s.values  = flash;
        s.latchAt = latchSeconds * 100U;
        return s;
    };
    Station                              station = boot();
    int                                  client  = -1;
    Host::FrameReader<Protocol::Command> reader;
    int                                  seenHolder = 0;
    int                                  seenResets = 0;
    auto                                 nextStep   = Clock::now();
    std::uint32_t                        lastStatus = 0;
    Protocol::Status                     sentStatus{};
    bool                                 sendTunables = true;

    while(quit == 0) {
        pollfd fds[2]{
          {listener, POLLIN, 0},
          {  client, POLLIN, 0}
        };
        ::poll(fds, client >= 0 ? 2 : 1, 2);

        if(holderToggles != seenHolder) {
            seenHolder = holderToggles;
            station.toggleHolder();
            std::printf("%s\n", station.inHolder ? "in holder" : "in hand");
            std::fflush(stdout);
        }
        if(resets != seenResets) {
            seenResets   = resets;
            station      = boot();
            lastStatus   = 0;
            sendTunables = true;
            std::printf("reset\n");
            std::fflush(stdout);
        }
        if((fds[0].revents & POLLIN) != 0) {
            int const fd = ::accept4(listener, nullptr, nullptr, SOCK_CLOEXEC);
            if(client < 0) {
                client = fd;
                reader.clear();
            } else if(fd >= 0) {
                ::close(fd);   // one at a time
            }
        }
        if(client >= 0 && (fds[1].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
            std::byte  chunk[256];
            auto const n = ::recv(client, chunk, sizeof(chunk), 0);
            if(n <= 0) {
                ::close(client);
                client = -1;
            } else {
                reader.append(std::span<std::byte const>{chunk, static_cast<std::size_t>(n)});
            }
        }

        // remote/Remote.hpp's serve()
        while(auto const command = reader.next()) {
            auto const out = Remote::handle(*command, station.values, station.latched());
            if(out.solderChanged) { flash[Remote::solderC] = station.values[Remote::solderC]; }
            if(out.store) {
                flash        = station.values;
                sendTunables = true;
            }
            sendReport(client, Protocol::Report{out.ack});
            if(out.sendAll || out.changed) { sendTunables = true; }
            if(out.sendAll) {   // as the firmware: Status, Diagnostics and Tunables
                sendReport(client, Protocol::Report{station.status()});
                sendReport(client, Protocol::Report{station.diagnostics()});
            }
        }
        if(sendTunables) {
            sendReport(client,
                       Protocol::Report{
                         Protocol::Tunables{station.values, flash}
            });
            sendTunables = false;
        }

        // the simulation catches up with the wall clock, 10 ms a step (fast: shorter)
        auto const stepEvery = std::chrono::microseconds{10'000 / fast};
        auto const now       = Clock::now();
        while(nextStep <= now) {
            nextStep += stepEvery;
            station.step();
            if(station.halfCycles % 10 == 0) {
                sendReport(client, Protocol::Report{station.telemetry()});
            }
            auto const s       = station.status();
            bool const changed = s.state != sentStatus.state || s.flags != sentStatus.flags
                              || s.presetC != sentStatus.presetC;
            if(changed || station.halfCycles - lastStatus >= 100) {
                sendReport(client, Protocol::Report{s});
                sentStatus = s;
                lastStatus = station.halfCycles;
            }
            if(station.halfCycles % 100 == 50) {
                sendReport(client, Protocol::Report{station.diagnostics()});
            }
        }
    }
    if(client >= 0) { ::close(client); }
    ::close(listener);
    ::unlink(path.c_str());
    return 0;
}
