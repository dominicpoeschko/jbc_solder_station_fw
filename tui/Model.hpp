#pragma once
// What jbc_tui knows about the station, and the text it turns that into - no screen, no socket and
// no clock of its own (every call that needs the time is handed `now`), so the host tests
// (tests/tui_model_test.cpp) drive it: the reports as they come, the state of the link, the history
// the graphs draw, the tunables editor, and the lines `--dump` prints.
//
// The station is the truth: everything shown is what it reported last. A command shows only as
// "sent" until its Ack names it.
#include "remote/Protocol.hpp"
#include "remote/Tunables.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace Tui {

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

/// The station sends a Status at least every second; this long without any report and it is gone.
inline constexpr auto Silence = std::chrono::seconds{3};

/// The graphs' memory: two minutes of Telemetry at 10 Hz.
inline constexpr std::size_t HistoryLength = 1'200;

/// How far the link got. `missing`: the socket (or device) is not there - no printer runs for this
/// tree. `refused`: it is there, nobody accepts. `waiting`: open, no report yet. `silent`: open, but
/// nothing for Silence (halted, resetting, another firmware). `live`: reports arrive.
enum class LinkState { missing, refused, waiting, silent, live };

constexpr std::string_view name(LinkState s) {
    switch(s) {
    case LinkState::missing: return "missing";
    case LinkState::refused: return "refused";
    case LinkState::waiting: return "waiting";
    case LinkState::silent:  return "silent";
    case LinkState::live:    return "live";
    }
    return "?";
}

constexpr std::string_view name(Protocol::State s) {
    switch(s) {
    case Protocol::State::init:     return "INIT";
    case Protocol::State::sleep:    return "SLEEP";
    case Protocol::State::solder:   return "SOLDER";
    case Protocol::State::remove:   return "REMOVED";
    case Protocol::State::standby:  return "STANDBY";
    case Protocol::State::shutdown: return "SHUTDOWN";
    }
    return "?";
}

constexpr std::string_view name(Protocol::AdcState s) {
    switch(s) {
    case Protocol::AdcState::init:              return "init";
    case Protocol::AdcState::waitTemp:          return "wait temp";
    case Protocol::AdcState::waitCurrentStart:  return "wait peak";
    case Protocol::AdcState::waitCurrent:       return "measure current";
    case Protocol::AdcState::waitHeaterTurnOff: return "wait crossing";
    }
    return "?";
}

constexpr std::string_view name(Protocol::ResetCause c) {
    switch(c) {
    case Protocol::ResetCause::unknown: return "unknown";
    case Protocol::ResetCause::por:     return "power-on";
    case Protocol::ResetCause::bod12:   return "brown-out 1.2 V";
    case Protocol::ResetCause::bod33:   return "brown-out 3.3 V";
    case Protocol::ResetCause::ext:     return "reset pin";
    case Protocol::ResetCause::wdt:     return "watchdog";
    case Protocol::ResetCause::syst:    return "software";
    }
    return "?";
}

constexpr std::string_view name(Protocol::Ack::Code c) {
    switch(c) {
    case Protocol::Ack::Code::ok:           return "ok";
    case Protocol::Ack::Code::clamped:      return "clamped to range";
    case Protocol::Ack::Code::refused:      return "refused (latched)";
    case Protocol::Ack::Code::badIndex:     return "unknown tunable";
    case Protocol::Ack::Code::storePending: return "flash busy, retried by the station";
    }
    return "?";
}

/// `value` in units of 10^-decimals as a decimal: formatFixed(300, 3) is "0.300", (-5, 1) "-0.5".
inline std::string formatFixed(std::int64_t value,
                               int          decimals) {
    std::int64_t scale = 1;
    for(int i = 0; i < decimals; ++i) { scale *= 10; }
    bool const   negative  = value < 0;
    std::int64_t magnitude = negative ? -value : value;
    std::string  text      = (negative ? "-" : "") + std::to_string(magnitude / scale);
    if(decimals > 0) {
        auto fraction = std::to_string(magnitude % scale);
        text += "." + std::string(static_cast<std::size_t>(decimals) - fraction.size(), '0')
              + fraction;
    }
    return text;
}

/// "0.25" with 3 decimals -> 250: a decimal with at most `decimals` places, an optional leading
/// minus, exactly - no exponent, no unit, nothing past ±2^31.
inline std::optional<std::int32_t> parseFixed(std::string_view text,
                                              int              decimals) {
    while(!text.empty() && text.front() == ' ') { text.remove_prefix(1); }
    while(!text.empty() && text.back() == ' ') { text.remove_suffix(1); }
    bool const negative = !text.empty() && text.front() == '-';
    if(negative) { text.remove_prefix(1); }
    auto const dot   = text.find('.');
    auto const whole = text.substr(0, dot);
    auto const frac  = dot == std::string_view::npos ? std::string_view{} : text.substr(dot + 1);
    if((whole.empty() && frac.empty()) || frac.size() > static_cast<std::size_t>(decimals)
       || whole.size() > 9)
    {
        return std::nullopt;
    }
    std::int64_t value = 0;
    for(auto const c : whole) {
        if(c < '0' || c > '9') { return std::nullopt; }
        value = value * 10 + (c - '0');
    }
    for(int i = 0; i < decimals; ++i) {
        char const c
          = static_cast<std::size_t>(i) < frac.size() ? frac[static_cast<std::size_t>(i)] : '0';
        if(c < '0' || c > '9') { return std::nullopt; }
        value = value * 10 + (c - '0');
    }
    if(negative) { value = -value; }
    if(value > INT32_MAX || value < INT32_MIN) { return std::nullopt; }
    return static_cast<std::int32_t>(value);
}

/// A tunable's value with its unit: "0.300 1/°C", "320 °C".
inline std::string formatTunable(std::size_t  index,
                                 std::int32_t value) {
    auto const& info = Remote::Infos[index];
    return formatFixed(value, info.decimals) + " " + std::string{info.unit};
}

/// Tenths of a degree as "312.4".
inline std::string deciC(std::int16_t v) { return formatFixed(v, 1); }

/// One Telemetry as the graphs keep it.
struct Sample {
    std::int16_t  tip;
    std::int16_t  tipMin;
    std::int16_t  tipMax;
    std::int16_t  setpoint;
    std::uint16_t pidCenti;
    std::uint8_t  halfCycles;
    std::uint8_t  heatedCycles;
};

class Model {
public:
    // -- the link ----------------------------------------------------------------------------

    void linkMissing(std::string path) {
        connected_ = false;
        refused_   = false;
        path_      = std::move(path);
    }

    void linkRefused(std::string path) {
        connected_ = false;
        refused_   = true;
        path_      = std::move(path);
    }

    void linkUp(std::string path,
                TimePoint   now) {
        connected_  = true;
        refused_    = false;
        path_       = std::move(path);
        upAt_       = now;
        lastReport_ = std::nullopt;
    }

    [[nodiscard]] LinkState link(TimePoint now) const {
        if(!connected_) { return refused_ ? LinkState::refused : LinkState::missing; }
        if(!lastReport_) { return now - upAt_ > Silence ? LinkState::silent : LinkState::waiting; }
        return now - *lastReport_ > Silence ? LinkState::silent : LinkState::live;
    }

    [[nodiscard]] std::string const& path() const { return path_; }

    // -- reports -----------------------------------------------------------------------------

    void apply(Protocol::Report const& report,
               TimePoint               now) {
        lastReport_ = now;
        ++reports_;
        std::visit([&](auto const& r) { take(r, now); }, report);
    }

    [[nodiscard]] std::optional<Protocol::Status> const& status() const { return status_; }

    [[nodiscard]] std::optional<Protocol::Diagnostics> const& diagnostics() const {
        return diagnostics_;
    }

    [[nodiscard]] std::optional<Remote::Values> const& tunables() const { return tunables_; }

    /// What the station's EEPROM emulation holds: what a reset brings back.
    [[nodiscard]] std::optional<Remote::Values> const& stored() const { return stored_; }

    /// Tunable `i` is in use but not stored: a reset would lose it.
    [[nodiscard]] bool unsaved(std::size_t i) const {
        return tunables_ && stored_ && (*tunables_)[i] != (*stored_)[i];
    }

    [[nodiscard]] std::size_t unsavedCount() const {
        std::size_t n = 0;
        for(std::size_t i = 0; i < Remote::Count; ++i) { n += unsaved(i) ? 1U : 0U; }
        return n;
    }

    [[nodiscard]] std::optional<Protocol::Telemetry> const& telemetry() const { return telemetry_; }

    [[nodiscard]] std::deque<Sample> const& history() const { return history_; }

    /// Station resets seen (uptime went back) since the remote started, and when the last was.
    [[nodiscard]] unsigned resets() const { return resets_; }

    [[nodiscard]] std::optional<TimePoint> const& lastReset() const { return lastReset_; }

    [[nodiscard]] std::uint64_t reports() const { return reports_; }

    [[nodiscard]] bool latched() const {
        return status_ && (status_->flags & Protocol::Flag::latched) != 0;
    }

    // -- commands ----------------------------------------------------------------------------

    /// The next command's sequence number, noted as sent with `what` until its Ack comes.
    std::uint16_t nextSeq(std::string what) {
        ++seq_;
        if(seq_ == 0) { seq_ = 1; }   // 0 is the link's own Get
        pendingSeq_  = seq_;
        pendingWhat_ = std::move(what);
        answer_      = "sent: " + pendingWhat_;
        return seq_;
    }

    /// What became of the last command: "sent: ...", "kp = 0.400: ok", ...
    [[nodiscard]] std::string const& answer() const { return answer_; }

    [[nodiscard]] bool answerBad() const { return answerBad_; }

    // -- the tunables editor -----------------------------------------------------------------

    [[nodiscard]] std::size_t selected() const { return selected_; }

    void select(int delta) {
        auto const n = static_cast<int>(Remote::Count);
        selected_ = static_cast<std::size_t>(((static_cast<int>(selected_) + delta) % n + n) % n);
    }

    void selectIndex(std::size_t i) {
        if(i < Remote::Count) { selected_ = i; }
    }

    /// The selected tunable moved `steps` arrow presses from what the station uses; nullopt before
    /// the station said what it uses.
    [[nodiscard]] std::optional<Protocol::SetTunable> step(int steps) {
        if(!tunables_) { return std::nullopt; }
        auto const& info  = Remote::Infos[selected_];
        auto const  value = Remote::clampTo(selected_, (*tunables_)[selected_] + steps * info.step);
        return set(selected_, value);
    }

    /// Every tunable but the solder temperature back to its default, in RAM (a reset brings back
    /// what is stored).
    [[nodiscard]] Protocol::ResetTunables defaults() {
        return {nextSeq("every tunable but the solder temp to its default (RAM)")};
    }

    /// The defaults into the flash too: undoes a stored bad set. Two commands, sent in this order; the
    /// answer shown is the store's.
    [[nodiscard]] std::array<Protocol::Command,
                             2>
    defaultsToFlash() {
        Protocol::Command const reset{Protocol::ResetTunables{nextSeq("defaults")}};
        Protocol::Command const store{
          Protocol::StoreTunables{nextSeq("defaults stored (all but the solder temp)")}};
        return {reset, store};
    }

    /// Every tunable in use into the station's flash.
    [[nodiscard]] Protocol::StoreTunables store() { return {nextSeq("store every tunable")}; }

    [[nodiscard]] std::optional<Protocol::SetTunable> toDefault() {
        return set(selected_, Remote::Infos[selected_].fallback);
    }

    /// A typed value for the selected tunable, in its shown unit ("0.4" for kp); nullopt with
    /// entryError() saying why.
    [[nodiscard]] std::optional<Protocol::SetTunable> typed(std::string_view text) {
        auto const& info  = Remote::Infos[selected_];
        auto const  value = parseFixed(text, info.decimals);
        if(!value) {
            entryError_ = "not a number with up to " + std::to_string(info.decimals) + " decimals";
            return std::nullopt;
        }
        if(!Remote::inRange(selected_, *value)) {
            entryError_ = "outside " + formatFixed(info.min, info.decimals) + " .. "
                        + formatFixed(info.max, info.decimals);
            return std::nullopt;
        }
        entryError_.clear();
        return set(selected_, *value);
    }

    [[nodiscard]] std::string const& entryError() const { return entryError_; }

    // -- --dump ------------------------------------------------------------------------------

    /// One line per report, for scripts and the end-to-end test.
    static std::string dumpLine(Protocol::Report const& report) {
        char line[256];
        if(auto const* t = std::get_if<Protocol::Telemetry>(&report)) {
            std::snprintf(line,
                          sizeof(line),
                          "telemetry tip=%s min=%s max=%s set=%s pid=%s heated=%u/%u raw=%u cur=%u",
                          deciC(t->tipDeciC).c_str(),
                          deciC(t->tipMinDeciC).c_str(),
                          deciC(t->tipMaxDeciC).c_str(),
                          deciC(t->setpointDeciC).c_str(),
                          formatFixed(t->pidCenti, 2).c_str(),
                          unsigned{t->heatedCycles},
                          unsigned{t->halfCycles},
                          unsigned{t->rawTemp},
                          unsigned{t->rawCurrent});
        } else if(auto const* s = std::get_if<Protocol::Status>(&report)) {
            std::snprintf(
              line,
              sizeof(line),
              "status state=%s adc=%s flags=0x%02x preset=%u led=%u,%u,%u uptime=%u latchIn=%u",
              std::string{name(s->state)}.c_str(),
              std::string{name(s->adc)}.c_str(),
              unsigned{s->flags},
              unsigned{s->presetC},
              unsigned{s->ledR},
              unsigned{s->ledG},
              unsigned{s->ledB},
              s->uptimeMs,
              s->latchInS);
        } else if(auto const* d = std::get_if<Protocol::Diagnostics>(&report)) {
            std::snprintf(
              line,
              sizeof(line),
              "diagnostics mains=%u/%u trafo=%s reset=%s fuses=%d eeprom=%u%s maxTurn=%u adc=%u "
              "probe=%u/%u/%u/%u uart=%u/%u/%u/%u",
              unsigned{d->mainsHalfPeriodUs[0]},
              unsigned{d->mainsHalfPeriodUs[1]},
              d->transformerDeciC == Protocol::Diagnostics::Unknown
                ? "-"
                : deciC(d->transformerDeciC).c_str(),
              std::string{name(d->resetCause)}.c_str(),
              d->fusesOk ? 1 : 0,
              unsigned{d->eepromC},
              d->eepromPending ? "(pending)" : "",
              d->maxTurnUs,
              d->adcConversions,
              d->probe.framesIn,
              d->probe.framesOut,
              unsigned{d->probe.badBytes},
              unsigned{d->probe.dropped},
              d->uart.framesIn,
              d->uart.framesOut,
              unsigned{d->uart.badBytes},
              unsigned{d->uart.dropped});
        } else if(auto const* v = std::get_if<Protocol::Tunables>(&report)) {
            std::string text = "tunables";
            for(std::size_t i = 0; i < Remote::Count; ++i) {
                text += ' ';
                text += Remote::Infos[i].name;
                text += '=';
                text += formatFixed(v->values[i], Remote::Infos[i].decimals);
            }
            text += " unsaved=";
            bool any = false;
            for(std::size_t i = 0; i < Remote::Count; ++i) {
                if(v->values[i] == v->stored[i]) { continue; }
                if(any) { text += ','; }
                text += Remote::Infos[i].name;
                any = true;
            }
            if(!any) { text += '-'; }
            return text;
        } else if(auto const* a = std::get_if<Protocol::Ack>(&report)) {
            std::snprintf(line,
                          sizeof(line),
                          "ack seq=%u %s",
                          unsigned{a->seq},
                          std::string{name(a->code)}.c_str());
        }
        return line;
    }

private:
    std::optional<Protocol::SetTunable> set(std::size_t  index,
                                            std::int32_t value) {
        auto const& info = Remote::Infos[index];
        auto const  seq
          = nextSeq(std::string{info.name} + " = " + formatFixed(value, info.decimals));
        return Protocol::SetTunable{.seq   = seq,
                                    .index = static_cast<std::uint8_t>(index),
                                    .value = value};
    }

    void take(Protocol::Telemetry const& t,
              TimePoint) {
        telemetry_ = t;
        history_.push_back(Sample{.tip          = t.tipDeciC,
                                  .tipMin       = t.tipMinDeciC,
                                  .tipMax       = t.tipMaxDeciC,
                                  .setpoint     = t.setpointDeciC,
                                  .pidCenti     = t.pidCenti,
                                  .halfCycles   = t.halfCycles,
                                  .heatedCycles = t.heatedCycles});
        while(history_.size() > HistoryLength) { history_.pop_front(); }
    }

    void take(Protocol::Status const& s,
              TimePoint               now) {
        if(status_ && s.uptimeMs < status_->uptimeMs) {
            ++resets_;
            lastReset_ = now;
            history_.clear();
        }
        status_ = s;
    }

    void take(Protocol::Diagnostics const& d,
              TimePoint) {
        diagnostics_ = d;
    }

    void take(Protocol::Tunables const& t,
              TimePoint) {
        tunables_ = t.values;
        stored_   = t.stored;
    }

    void take(Protocol::Ack const& a,
              TimePoint) {
        if(a.seq == 0 || a.seq != pendingSeq_) { return; }   // the link's own Get, or an old one
        answer_     = pendingWhat_ + ": " + std::string{name(a.code)};
        answerBad_  = a.code == Protocol::Ack::Code::refused
                   || a.code == Protocol::Ack::Code::badIndex
                   || a.code == Protocol::Ack::Code::storePending;
        pendingSeq_ = 0;
    }

    bool                     connected_{};
    bool                     refused_{};
    std::string              path_;
    TimePoint                upAt_{};
    std::optional<TimePoint> lastReport_;
    std::uint64_t            reports_{};

    std::optional<Protocol::Status>      status_;
    std::optional<Protocol::Diagnostics> diagnostics_;
    std::optional<Remote::Values>        tunables_;
    std::optional<Remote::Values>        stored_;
    std::optional<Protocol::Telemetry>   telemetry_;
    std::deque<Sample>                   history_;
    unsigned                             resets_{};
    std::optional<TimePoint>             lastReset_;

    std::uint16_t seq_{};
    std::uint16_t pendingSeq_{};
    std::string   pendingWhat_;
    std::string   answer_;
    bool          answerBad_{};
    std::size_t   selected_{};
    std::string   entryError_;
};

}   // namespace Tui
