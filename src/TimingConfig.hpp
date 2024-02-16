//
// Created by patrick on 1/28/22.
//

#pragma once

#include <chrono>

namespace timingConfig{
    constexpr auto menuTimeOut{std::chrono::seconds{5}};
    constexpr auto failTimeOut{std::chrono::hours{1}};
    constexpr auto standbyTimeOut{std::chrono::minutes{5}};
    constexpr auto sleepTimeOut{std::chrono::seconds{0}};
    constexpr auto maxRuntime{std::chrono::hours{1}};
}
