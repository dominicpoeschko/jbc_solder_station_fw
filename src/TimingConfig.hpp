//
// Created by patrick on 1/28/22.
//

#pragma once

#include <chrono>

namespace timingConfig {
constexpr auto menuTimeOut{std::chrono::seconds{5}};
constexpr auto failTimeOut{std::chrono::hours{1}};
constexpr auto standbyTimeOut{std::chrono::minutes{5}};
constexpr auto sleepTimeOut{std::chrono::seconds{0}};
constexpr auto maxRuntime{std::chrono::hours{1}};
}   // namespace timingConfig

/// The heater's guards beyond the timeouts (Temp, Application). None is tried on a station yet: the numbers are
/// first guesses, chosen to stay clear of normal soldering (a JBC tip recovers in seconds at full power).
namespace safetyConfig {
/// At full power the tip must warm by heatingMinRiseC within every heatingRiseWindow, else the reading is not the
/// tip's (a broken or shorted sensor reads low and asks for full power for ever): heater off until a power cycle.
constexpr auto heatingRiseWindow{std::chrono::seconds{15}};
constexpr int  heatingMinRiseC{20};
/// The current conversion at the peak takes tens of microseconds; one that has not come after this switches the
/// heater off (Temp::turn, State::wait_current).
constexpr auto currentConversionTimeOut{std::chrono::milliseconds{2}};
/// A zero crossing's conversion this long after the switch-on, the heater still on: the main loop missed its
/// switch-off, the ADC interrupt does it (Temp::adcCallback). Under a half-cycle at 60 Hz (8.3 ms), far above the
/// noise right after the switch-on.
constexpr auto lateSwitchOffAfter{std::chrono::milliseconds{7}};
/// The transformer (MAX31820, README: "> 50 °C -> turn off"): heater held off above transformerMaxMilliC, on again
/// below transformerResumeMilliC. No reading (sensor failed) holds nothing.
constexpr int transformerMaxMilliC{50'000};
constexpr int transformerResumeMilliC{45'000};
}   // namespace safetyConfig
