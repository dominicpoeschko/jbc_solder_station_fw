#pragma once
// The image's config header: what the libraries look up by uc_log::Tag::User - the log sink and the log clock, both
// complete. uc_log/uc_log.hpp includes it first (UC_LOG_CONFIG_HEADER, set by target_configure_kvasir for
// src/kvasir_config.hpp), so every TU that logs sees the same specialisations before any log line. Never include
// uc_log/uc_log.hpp here, directly or through another header (that is a named #error).

#include "chip/atsam_common/DebuggerPresent.hpp"
#include "uc_log/LogClock.hpp"
#include "uc_log/RttLog.hpp"

namespace uc_log {
// A record from an ISR is written with interrupts masked (IsrPolicy::MaskedRecord), so ISRs at
// different priority levels share the ISR buffer; Startup refuses that mix of levels otherwise.
template<>
struct ComBackend<uc_log::Tag::User>
  : public RttLog<
      Kvasir::Chip::DebuggerPresent,
      Rings<512, 4096>,
      // "control": remote/Protocol.hpp, see remote/Remote.hpp. uc_log_printer puts it on a
      // socket, duplex.0.sock, for tui/ (jbc_tui).
      DuplexChannels<DuplexChannelConfig<"control", 256, 128>>,
      IsrPolicy::MaskedRecord> {};
}   // namespace uc_log

#include "core/Systick.hpp"

namespace HW {
static constexpr auto clockSpeedValue = 48'000'000;

struct SystickClockConfig {
    static constexpr auto clockBase = Kvasir::Systick::useProcessorClock;

    static constexpr auto clockSpeed     = clockSpeedValue;
    static constexpr auto minOverrunTime = std::chrono::hours(24 * 365) * 20;
};

using SystickClock = Kvasir::Systick::SystickClockBase<SystickClockConfig>;
}   // namespace HW

namespace uc_log {
template<>
struct LogClock<uc_log::Tag::User> : public HW::SystickClock {};
}   // namespace uc_log
