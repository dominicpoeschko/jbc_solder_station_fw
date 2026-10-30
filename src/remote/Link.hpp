#pragma once
// One end of the remote protocol (Protocol.hpp) over a byte transport: frames in, commands out;
// reports in, frames out (psu302's Link, with counters for Diagnostics). The transports, each
// with read(span) -> what arrived, write(span) -> what did not fit and idle() -> the last write
// has let go of its bytes:
//   - ProbeTransport<Channel>: uc_log's duplex channel "control" (HW::ComBackend::DuplexChannel<0>),
//     which copies into its ring at once
//   - UartTransport<Uart>: SERCOM1 (RemoteUart), which sends by DMA from the caller's bytes - they
//     must stay as they are until the transfer is done
#include "Protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <kvasir/Util/StaticVector.hpp>
#include <optional>
#include <span>

namespace Remote {

template<typename Channel>
struct ProbeTransport {
    static std::span<std::byte> read(std::span<std::byte> into) { return Channel::read(into); }

    static std::span<std::byte const> write(std::span<std::byte const> bytes) {
        return Channel::write(bytes);
    }

    static bool idle() { return true; }
};

template<typename Uart>
struct UartTransport {
    static std::span<std::byte> read(std::span<std::byte> into) {
        std::size_t              n = 0;
        std::optional<std::byte> b;
        while(n < into.size() && Uart::receive(b)) {
            if(b) { into[n++] = *b; }   // an empty one is a frame error: the CRC will tell
        }
        return into.first(n);
    }

    /// All of `bytes` or nothing: Uart::send takes a whole transfer once the last one is done.
    static std::span<std::byte const> write(std::span<std::byte const> bytes) {
        return bytes.subspan(Uart::send(bytes));
    }

    static bool idle() { return Uart::txIdle(); }
};

template<typename Transport>
class Link {
public:
    /// The next complete command, if one has arrived. Garbage between frames is skipped and counted.
    std::optional<Protocol::Command> receive() {
        std::array<std::byte, 32> chunk{};
        for(auto const byte : Transport::read(chunk)) {
            if(in_.size() == in_.max_size()) {
                count(counters_.badBytes, in_.size());
                in_.clear();
            }
            in_.push_back(byte);
        }
        while(!in_.empty()) {
            Protocol::Command command{};
            auto const        result = Protocol::Packager::unpack(in_, command);
            auto const        used   = result ? result->consumed : result.error().consumed;
            in_.erase(in_.begin(), std::next(in_.begin(), static_cast<std::ptrdiff_t>(used)));
            if(result) {
                ++counters_.framesIn;
                return command;
            }
            if(used == 0) { break; }
            count(counters_.badBytes, used);
        }
        return std::nullopt;
    }

    /// True when the last report has left and the transport let go of it: a new one may be packed.
    bool ready() {
        flush();
        return sent_ == out_.size() && Transport::idle();
    }

    /// Pack a report and start it; call only when ready().
    void send(Protocol::Report const& report) {
        out_.clear();
        sent_ = 0;
        if(!Protocol::Packager::pack(out_, report)) {
            out_.clear();
            return;
        }
        ++counters_.framesOut;
        flush();
    }

    [[nodiscard]] Protocol::LinkCounters const& counters() const { return counters_; }

    void clearCounters() { counters_ = {}; }

private:
    static void count(std::uint16_t& c,
                      std::size_t    n) {
        c = static_cast<std::uint16_t>(c + n < 0xFFFFU ? c + n : 0xFFFFU);
    }

    void flush() {
        auto const pending = std::span<std::byte const>{out_}.subspan(sent_);
        if(pending.empty()) { return; }
        auto const rest = Transport::write(pending);
        sent_ += pending.size() - rest.size();
        if(rest.size() != pending.size()) {
            stalled_ = 0;
            return;
        }
        // nobody reading (no printer on the probe): give up on this one, the next report comes anyway
        if(++stalled_ > StallTurns) {
            out_.clear();
            sent_    = 0;
            stalled_ = 0;
            count(counters_.dropped, 1);
        }
    }

    static constexpr std::uint32_t StallTurns = 1'000;

    Kvasir::StaticVector<std::byte, Protocol::MaxFrame> in_;
    Kvasir::StaticVector<std::byte, Protocol::MaxFrame> out_;
    std::size_t                                         sent_{};
    std::uint32_t                                       stalled_{};
    Protocol::LinkCounters                              counters_{};
};

}   // namespace Remote
