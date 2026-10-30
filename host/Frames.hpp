#pragma once
// The host's end of remote/Protocol.hpp's framing, without a transport: commands to frames, a byte
// stream back to reports (garbage between frames skipped, a split frame kept until it is whole).
// Shared by jbc_tui and its fake station (psu302's host/Frames.hpp); host-tested
// (tests/tui_model_test.cpp).
#include "remote/Protocol.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace Host {

/// The frame for one message; empty if it did not pack (it always does: MaxSize is checked by the
/// protocol test).
template<typename Message>
std::vector<std::byte> frame(Message const& message) {
    std::vector<std::byte> out;
    if(!Protocol::Packager::pack(out, message)) { out.clear(); }
    return out;
}

/// Bytes in, whole messages out. `Message` is Protocol::Report on the host, Protocol::Command in
/// the fake station.
template<typename Message>
class FrameReader {
public:
    void append(std::span<std::byte const> bytes) {
        buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
    }

    /// The next whole message, if there is one.
    std::optional<Message> next() {
        while(!buffer_.empty()) {
            Message    message{};
            auto const result = Protocol::Packager::unpack(buffer_, message);
            auto const used   = result ? result->consumed : result.error().consumed;
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(used));
            if(result) { return message; }
            if(used == 0) { break; }
        }
        return std::nullopt;
    }

    void clear() { buffer_.clear(); }

private:
    std::vector<std::byte> buffer_;
};

}   // namespace Host
