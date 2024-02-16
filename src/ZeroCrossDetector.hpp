//
// Created by patrick on 1/5/22.
//
#pragma once

#include "HWConfig.hpp"
#include <chrono>
#include <atomic>
#include "kvasir/Io/Io.hpp"

#include "HWConfig.hpp"

template<typename Clock, typename ZCpin>
struct ZeroCrossDectecor {
    static constexpr std::size_t averages = 64;

    static inline std::atomic<typename Clock::time_point> nextCrossing{};
    static inline std::atomic<typename Clock::time_point> nextPeak{};

    static inline bool                       edge{false};
    static inline typename Clock::duration   avgE1{std::chrono::milliseconds{10}};
    static inline typename Clock::duration   avgE2{std::chrono::milliseconds{10}};
    static inline std::size_t                currentAverages{};
    static inline typename Clock::time_point lastCrossing{};

    static void edgeCallback() {
        auto const now = Clock::now();
        edge           = !edge;
        if(edge) {
            if(currentAverages != averages) {
                ++currentAverages;
            }
        }

        auto& currentAvg = edge ? avgE1 : avgE2;
        auto& nextAvg    = edge ? avgE2 : avgE1;

        currentAvg
                = (currentAvg * currentAverages + (now - lastCrossing) * 8) / (currentAverages + 8);

        nextCrossing.store(now + nextAvg, std::memory_order_relaxed);
        nextPeak.store(now + nextAvg / 2, std::memory_order_relaxed);
        lastCrossing = now;
    }

    struct EicConfig {
        static constexpr auto pin         = ZCpin{};
        static constexpr auto pull        = Kvasir::Io::PullConfiguration::PullNone;
        static constexpr auto type        = Kvasir::EIC::InterruptType::EdgeBoth;
        static constexpr auto filter      = false;
        static constexpr auto callback    = edgeCallback;
        static constexpr auto enableEvent = true;
    };
};
