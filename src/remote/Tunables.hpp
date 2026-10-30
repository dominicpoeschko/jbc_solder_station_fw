#pragma once
// What a host may change on the station (Protocol::SetTunable), with names, units, ranges and
// defaults - one table both sides read, as water_mix's Regulator::Info. A change is used at once and
// lives in RAM; Protocol::StoreTunables writes them all to the EEPROM emulation, which is what the
// station starts with (the solder temperature is stored on every change, as a knob press).
//
// Not in here on purpose: the heater itself, and the 1 h safety latch (TimingConfig.hpp
// failTimeOut and maxRuntime, CLAUDE.md "The safety latch") - no command reaches those.
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Remote {

enum Index : std::uint8_t {
    solderC,   ///< what Solder regulates to; stored in the EEPROM, as a knob press
    sleepC,    ///< what Sleep regulates to after its delay
    kpMilli,   ///< PID gains x1000 (ApplicationConfig.hpp HeaterGains)
    kiMilli,
    kdMilli,
    tauMs,         ///< the PID's derivative filter
    standbyS,      ///< Sleep/Solder -> Standby (heater off) without a handling event
    sleepDelayS,   ///< Sleep's delay before the sleep temperature is set
    Count
};

using Values = std::array<std::int32_t, Count>;

/// One tunable. Its value is an integer in units of 10^-decimals of `unit` (kp 300 = 0.300).
struct Info {
    std::string_view name;
    std::string_view unit;
    std::uint8_t     decimals;
    std::int32_t     min;
    std::int32_t     max;
    std::int32_t     fallback;   ///< the default
    std::int32_t     step;       ///< what one arrow press changes it by in the remote
    std::string_view help;
};

/// The knob reaches 512 °C (ApplicationConfig.hpp RotaryEncoder); a remote stops below that.
inline constexpr std::int32_t MaxRemoteC = 450;

inline constexpr std::array<Info, Count> Infos{
  {
   {"solder temp", "°C", 0, 100, MaxRemoteC, 320, 5, "Solder setpoint; stored like a knob press"},
   {"sleep temp", "°C", 0, 0, 300, 210, 5, "Setpoint with the iron in its holder"},
   {"kp", "1/°C", 3, 0, 5000, 300, 10, "Proportional gain: half-cycles per °C of error"},
   {"ki", "1/°C s", 3, 0, 1000, 0, 1, "Integral gain (0: a PD loop)"},
   {"kd", "s/°C", 3, 0, 5000, 200, 10, "Derivative gain, on the measurement"},
   {"tau", "ms", 0, 10, 5000, 700, 50, "Derivative filter time constant"},
   {"standby after", "s", 0, 30, 1800, 300, 30, "Heater off after this long without handling"},
   {"sleep delay", "s", 0, 0, 600, 0, 5, "Time in the holder before the sleep temp applies"},
   }
};

constexpr Values defaults() {
    Values v{};
    for(std::size_t i = 0; i < Count; ++i) { v[i] = Infos[i].fallback; }
    return v;
}

/// A stored set as the station takes it: every value clamped into its range (a record of another
/// layout or a corrupted one reads as the defaults - its CRC fails).
constexpr Values sanitized(Values v) {
    for(std::size_t i = 0; i < Count; ++i) {
        v[i] = v[i] < Infos[i].min ? Infos[i].min : v[i] > Infos[i].max ? Infos[i].max : v[i];
    }
    return v;
}

constexpr bool inRange(std::size_t  index,
                       std::int32_t value) {
    return value >= Infos[index].min && value <= Infos[index].max;
}

constexpr std::int32_t clampTo(std::size_t  index,
                               std::int32_t value) {
    if(value < Infos[index].min) { return Infos[index].min; }
    if(value > Infos[index].max) { return Infos[index].max; }
    return value;
}

static_assert([] {
    for(std::size_t i = 0; i < Count; ++i) {
        if(!inRange(i, Infos[i].fallback) || Infos[i].step <= 0) { return false; }
    }
    return true;
}());

}   // namespace Remote
