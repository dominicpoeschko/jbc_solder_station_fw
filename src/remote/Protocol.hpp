#pragma once
// What goes between the station and a host (tui/, the jbc_tui remote): aglio-serialised variants
// in aglio::Packager frames (start word, length, header CRC, body CRC-16), on two transports at
// once - uc_log's "control" duplex channel over the probe (uc_log_printer serves it as
// <build>/rtt_log/<target>/duplex.0.sock) and SERCOM1's USART on the debug pins PA18/PA19,
// 115200 8N1. Shared by the firmware, the host and the tests: nothing but the standard library,
// aglio and Tunables.hpp in here. Every value is a plain integer in the unit its name says.
#include "Tunables.hpp"

#include <aglio/packager.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <variant>

namespace Protocol {

// host -> station. Every command carries a sequence number the Ack names.

/// Tunables, a Status and a Diagnostics at once (all three also come unasked).
struct Get {
    std::uint16_t seq;
};

/// One tunable (Remote::Info[index]) to `value`, clamped to its range; the solder temperature is
/// taken like a knob press (shown, stored in the EEPROM), the others are RAM only.
struct SetTunable {
    std::uint16_t seq;
    std::uint8_t  index;
    std::int32_t  value;
};

/// Every RAM-only tunable back to its default (the solder temperature stays); nothing stored changes.
struct ResetTunables {
    std::uint16_t seq;
};

/// Every tunable as it is now into the EEPROM emulation: what the station starts with after a reset.
struct StoreTunables {
    std::uint16_t seq;
};

/// The link counters and the loop's longest turn back to zero.
struct ClearCounters {
    std::uint16_t seq;
};

using Command = std::variant<Get, SetTunable, ResetTunables, ClearCounters, StoreTunables>;

// station -> host

/// The station's state machine (Application::stateMachine, same order).
enum class State : std::uint8_t { init, sleep, solder, remove, standby, shutdown };

/// Temp::State, the ADC's measurement cycle (same order).
enum class AdcState : std::uint8_t {
    init,
    waitTemp,
    waitCurrentStart,
    waitCurrent,
    waitHeaterTurnOff
};

/// The heater loop since the last Telemetry, every Telemetry period: one ADC conversion of the tip
/// a mains half-cycle.
struct Telemetry {
    std::int16_t  tipDeciC;   ///< mean of the half-cycles' tip temperatures
    std::int16_t  tipMinDeciC;
    std::int16_t  tipMaxDeciC;
    std::int16_t  setpointDeciC;   ///< what the loop regulates to now (0 in standby/remove)
    std::uint16_t rawTemp;         ///< the last tip ADC result
    std::uint16_t rawCurrent;      ///< the last heater current ADC result
    std::uint16_t pidCenti;        ///< the PID's output, 0..1000 = 0..10 half-cycles
    std::uint8_t  halfCycles;      ///< tip conversions in this period
    std::uint8_t  heatedCycles;    ///< of those, with the heater on
};

/// Status bits.
enum Flag : std::uint8_t {
    heaterOn      = 1U << 0,
    tipInSleep    = 1U << 1,   ///< tip 1 in its holder
    tipRemoved    = 1U << 2,   ///< tip 1 out of the handle
    latched       = 1U << 3,   ///< the 1 h safety latch: off until a power cycle
    presetDirty   = 1U << 4,   ///< the knob shows a value not committed yet (display blinks)
    tipNotHeating = 1U
      << 5,   ///< the tip did not warm at full power: heater off until a power cycle
    transformerHot = 1U
      << 6,   ///< the transformer is over its limit: heater held off until it cooled
};

struct Status {
    State         state;
    AdcState      adc;
    std::uint8_t  flags;   ///< Flag bits
    std::uint8_t  ledR;    ///< the status LED (KTD2026) as last set
    std::uint8_t  ledG;
    std::uint8_t  ledB;
    std::uint16_t presetC;    ///< the knob's value
    std::uint32_t uptimeMs;   ///< since boot: smaller than the last one = the station was reset
    std::uint32_t latchInS;   ///< seconds until the 1 h latch, 0 once latched
};

/// PM.RCAUSE of this boot (the station's chip: power-on, brown-out 1.2/3.3 V, reset pin, watchdog,
/// software).
enum class ResetCause : std::uint8_t { unknown, por, bod12, bod33, ext, wdt, syst };

/// One transport's counters since boot or ClearCounters.
struct LinkCounters {
    std::uint32_t framesIn;
    std::uint32_t framesOut;
    std::uint16_t badBytes;   ///< skipped while looking for a frame (garbage, CRC errors)
    std::uint16_t dropped;    ///< reports given up on: nobody reading
};

struct Diagnostics {
    static constexpr std::int16_t Unknown = -32768;

    std::array<std::uint16_t, 2>
                  mainsHalfPeriodUs;   ///< the zero-cross detector's averages, both edges
    std::int16_t  transformerDeciC;    ///< MAX31820 on the transformer, Unknown without a reading
    ResetCause    resetCause;
    bool          fusesOk;          ///< the fuse bits were right (or written) at boot
    bool          eepromPending;    ///< a store of the solder temperature still to go
    std::uint16_t eepromC;          ///< the solder temperature the EEPROM holds
    std::uint32_t maxTurnUs;        ///< the main loop's longest turn
    std::uint32_t adcConversions;   ///< ADC interrupts since boot
    LinkCounters  probe;            ///< the RTT duplex channel
    LinkCounters  uart;             ///< SERCOM1
};

/// The tunables in use, and what the EEPROM emulation holds - what a reset brings back (Remote::Info
/// has names, units, ranges and defaults).
struct Tunables {
    Remote::Values values;
    Remote::Values stored;
};

/// The answer to a command.
struct Ack {
    /// storePending: the NVM controller refused the write; it is retried from the main loop
    /// (Diagnostics.eepromPending says when it is in).
    enum class Code : std::uint8_t { ok, clamped, refused, badIndex, storePending };
    std::uint16_t seq;
    Code          code;
};

using Report = std::variant<Telemetry, Status, Diagnostics, Tunables, Ack>;

// framing

/// CRC-16/CCITT-FALSE, as water_mix and psu302: in software on both sides so they cannot disagree.
struct Crc {
    using type = std::uint16_t;

    static constexpr type calc(std::span<std::byte const> data) {
        std::uint32_t crc = 0xFFFF;
        for(auto const byte : data) {
            crc ^= static_cast<std::uint32_t>(byte) << 8U;
            for(int bit = 0; bit < 8; ++bit) {
                crc = ((crc & 0x8000U) != 0 ? (crc << 1U) ^ 0x1021U : crc << 1U) & 0xFFFFU;
            }
        }
        return static_cast<type>(crc);
    }
};

struct PackagerConfig {
    using Crc                                   = Protocol::Crc;
    using Size_t                                = std::uint8_t;
    static constexpr std::uint16_t PackageStart = 0x1BC0;
    static constexpr Size_t        MaxSize      = 96;
};

using Packager = aglio::Packager<PackagerConfig>;

/// The firmware's frame buffers.
inline constexpr std::size_t MaxFrame = PackagerConfig::MaxSize;

}   // namespace Protocol
