#pragma once
// A command against the tunables, without hardware: what the firmware's Remote, the host's fake
// station and the tests all run. The caller does the effects the Outcome names.
#include "Protocol.hpp"
#include "Tunables.hpp"

#include <cstddef>
#include <variant>

namespace Remote {

struct Outcome {
    Protocol::Ack ack;
    bool          changed{};         ///< a tunable changed: apply them, send Tunables
    bool          solderChanged{};   ///< the solder temperature changed: show it, store it
    bool          clearCounters{};
    bool          sendAll{};   ///< Get: Tunables, Status and Diagnostics
    bool store{};   ///< StoreTunables: write them to the EEPROM emulation, then Ack and Tunables
};

/// `latched`: the station sits in its safety latch; it answers, but changes nothing.
constexpr Outcome handle(Protocol::Command const& command,
                         Values&                  values,
                         bool                     latched) {
    using Code       = Protocol::Ack::Code;
    auto const seqOf = [](auto const& c) { return c.seq; };
    Outcome    out{
      .ack = {.seq = std::visit(seqOf, command), .code = Code::ok}
    };

    if(auto const* get = std::get_if<Protocol::Get>(&command)) {
        static_cast<void>(get);
        out.sendAll = true;
        return out;
    }
    if(latched) {
        out.ack.code = Code::refused;
        return out;
    }
    if(auto const* set = std::get_if<Protocol::SetTunable>(&command)) {
        if(set->index >= Count) {
            out.ack.code = Code::badIndex;
            return out;
        }
        auto const value = clampTo(set->index, set->value);
        if(value != set->value) { out.ack.code = Code::clamped; }
        if(values[set->index] != value) {
            values[set->index] = value;
            out.changed        = true;
            out.solderChanged  = set->index == solderC;
        }
        return out;
    }
    if(std::holds_alternative<Protocol::ResetTunables>(command)) {
        auto const fresh = defaults();
        for(std::size_t i = 0; i < Count; ++i) {
            if(i == solderC || values[i] == fresh[i]) { continue; }
            values[i]   = fresh[i];
            out.changed = true;
        }
        return out;
    }
    if(std::holds_alternative<Protocol::StoreTunables>(command)) {
        out.store = true;
        return out;
    }
    out.clearCounters = true;   // ClearCounters
    return out;
}

}   // namespace Remote
