//
// Created by patrick on 1/5/22.
//
#pragma once

#include "HWConfig.hpp"
#include "kvasir/Io/Io.hpp"

#include <atomic>
#include <chip/atsam_common/EIC.hpp>
#include <chrono>

/// The mains zero crossings on an EXTINT line (both edges, no pull, no filter, with its event
/// output for the heater timing). Listing it in Startup is all it takes; EicPriority is the
/// NVIC priority of the EIC interrupt (all lines on it must agree).
template<typename Clock, typename ZCpin, int EicPriority>
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
            if(currentAverages != averages) { ++currentAverages; }
        }

        auto&       currentAvg = edge ? avgE1 : avgE2;
        auto const& nextAvg    = edge ? avgE2 : avgE1;

        currentAvg
          = (currentAvg * currentAverages + (now - lastCrossing) * 8) / (currentAverages + 8);

        nextCrossing.store(now + nextAvg, std::memory_order_relaxed);
        nextPeak.store(now + nextAvg / 2, std::memory_order_relaxed);
        lastCrossing = now;
    }

    using EicLine = Kvasir::EIC::ExtInt<ZCpin,
                                        Kvasir::EIC::InterruptType::edgeBoth,
                                        &edgeCallback,
                                        EicPriority,
                                        Kvasir::Io::PullConfiguration::pullNone,
                                        false,
                                        true>;

    using Provides                                = typename EicLine::Provides;
    using Claims                                  = typename EicLine::Claims;
    using SubIsrs                                 = typename EicLine::SubIsrs;
    static constexpr auto initStepPinConfig       = EicLine::initStepPinConfig;
    static constexpr auto initStepPeripheryConfig = EicLine::initStepPeripheryConfig;
};
