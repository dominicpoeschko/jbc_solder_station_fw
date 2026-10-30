// jbc_tui's model (tui/Model.hpp), its graphs (tui/Plot.hpp) and the host's frame reader
// (host/Frames.hpp): no RTXUI, no sockets, and the time only what the test hands in.
#include "Check.hpp"
#include "Frames.hpp"
#include "Model.hpp"
#include "Plot.hpp"

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <string>
#include <system_error>

namespace {
using namespace std::chrono_literals;

Tui::TimePoint at(std::chrono::milliseconds ms) { return Tui::TimePoint{} + ms; }

Protocol::Status status(std::uint32_t uptimeMs,
                        std::uint8_t  flags = 0) {
    Protocol::Status s{};
    s.state    = Protocol::State::solder;
    s.uptimeMs = uptimeMs;
    s.flags    = flags;
    return s;
}

Protocol::Telemetry telemetry(std::int16_t tipDeciC,
                              std::int16_t setDeciC,
                              std::uint8_t heated) {
    Protocol::Telemetry t{};
    t.tipDeciC      = tipDeciC;
    t.tipMinDeciC   = tipDeciC;
    t.tipMaxDeciC   = tipDeciC;
    t.setpointDeciC = setDeciC;
    t.halfCycles    = 10;
    t.heatedCycles  = heated;
    return t;
}

bool near(double a,
          double b) {
    return std::abs(a - b) < 1e-9;
}

std::size_t count(std::string const& text,
                  std::string const& what) {
    std::size_t n = 0;
    for(auto p = text.find(what); p != std::string::npos; p = text.find(what, p + what.size())) {
        ++n;
    }
    return n;
}
}   // namespace

int main() {
    Test::testCase("fixed-point text");
    Test::check(Tui::formatFixed(300, 3) == "0.300", "0.300");
    Test::check(Tui::formatFixed(-5, 1) == "-0.5", "-0.5");
    Test::check(Tui::formatFixed(3204, 1) == "320.4", "320.4");
    Test::check(Tui::formatFixed(700, 0) == "700", "700");
    Test::check(Tui::parseFixed("0.4", 3) == 400, "0.4 -> 400");
    Test::check(Tui::parseFixed(" 12 ", 0) == 12, "spaces");
    Test::check(Tui::parseFixed("-1.25", 2) == -125, "negative");
    Test::check(!Tui::parseFixed("0.0001", 3), "too many places");
    Test::check(!Tui::parseFixed("1e3", 0), "no exponent");
    Test::check(!Tui::parseFixed("", 0), "empty");
    Test::check(!Tui::parseFixed("99999999999", 0), "too long");

    Test::testCase("link states");
    {
        Tui::Model m;
        Test::check(m.link(at(0ms)) == Tui::LinkState::missing, "missing at first");
        m.linkRefused("x.sock");
        Test::check(m.link(at(0ms)) == Tui::LinkState::refused, "refused");
        m.linkUp("x.sock", at(1'000ms));
        Test::check(m.link(at(1'500ms)) == Tui::LinkState::waiting, "waiting");
        Test::check(m.link(at(4'500ms)) == Tui::LinkState::silent, "silent without a first report");
        m.apply(Protocol::Report{status(10)}, at(4'600ms));
        Test::check(m.link(at(5'000ms)) == Tui::LinkState::live, "live");
        Test::check(m.link(at(7'700ms)) == Tui::LinkState::silent, "silent after Silence");
        Test::check(m.path() == "x.sock", "path kept");
    }

    Test::testCase("a station reset: uptime goes back");
    {
        Tui::Model m;
        m.apply(Protocol::Report{telemetry(3000, 3200, 5)}, at(0ms));
        m.apply(Protocol::Report{status(50'000)}, at(0ms));
        m.apply(Protocol::Report{status(51'000)}, at(1'000ms));
        Test::check(m.resets() == 0, "no reset while it grows");
        m.apply(Protocol::Report{status(200)}, at(2'000ms));
        Test::check(m.resets() == 1 && m.lastReset() == at(2'000ms), "reset seen");
        Test::check(m.history().empty(), "history starts again");
    }

    Test::testCase("latched");
    {
        Tui::Model m;
        Test::check(!m.latched(), "not before a status");
        m.apply(Protocol::Report{status(1, Protocol::Flag::latched)}, at(0ms));
        Test::check(m.latched(), "latched flag");
    }

    Test::testCase("history is bounded");
    {
        Tui::Model m;
        for(std::size_t i = 0; i < Tui::HistoryLength + 50; ++i) {
            m.apply(Protocol::Report{telemetry(static_cast<std::int16_t>(i), 0, 0)}, at(0ms));
        }
        Test::check(m.history().size() == Tui::HistoryLength, "capped");
        Test::check(m.history().front().tip == 50, "oldest dropped first");
    }

    Test::testCase("the editor");
    {
        Tui::Model m;
        Test::check(!m.step(1), "nothing to step before the station's tunables");
        m.apply(
          Protocol::Report{
            Protocol::Tunables{Remote::defaults(), Remote::defaults()}
        },
          at(0ms));
        Test::check(m.unsavedCount() == 0, "all stored");
        auto c = m.step(1);
        Test::check(c && c->index == Remote::solderC && c->value == 325, "solder +5");
        c = m.step(-1000);
        Test::check(c && c->value == Remote::Infos[Remote::solderC].min, "clamped low");
        m.select(-1);
        Test::check(m.selected() == Remote::Count - 1, "wraps upward");
        m.select(1);
        m.select(2);
        Test::check(m.selected() == Remote::kpMilli, "kp");
        c = m.typed("0.45");
        Test::check(c && c->value == 450 && m.entryError().empty(), "typed 0.45");
        Test::check(m.answer() == "sent: kp = 0.450", "sent shown");
        Test::check(!m.typed("9"), "out of range refused");
        Test::check(!m.entryError().empty(), "says why");
        Test::check(!m.typed("abc"), "not a number");

        // the Ack names the command; an old or the link's own (seq 0) is ignored
        m.apply(
          Protocol::Report{
            Protocol::Ack{0, Protocol::Ack::Code::refused}
        },
          at(0ms));
        Test::check(m.answer() == "sent: kp = 0.450", "seq 0 ignored");
        m.apply(
          Protocol::Report{
            Protocol::Ack{static_cast<std::uint16_t>(c ? c->seq - 1 : 0), Protocol::Ack::Code::ok}
        },
          at(0ms));
        Test::check(m.answer() == "sent: kp = 0.450", "an older seq ignored");
        auto const d = m.toDefault();
        Test::check(d.has_value(), "a default to send");
        m.apply(
          Protocol::Report{
            Protocol::Ack{d.value_or(Protocol::SetTunable{}).seq, Protocol::Ack::Code::refused}
        },
          at(0ms));
        Test::check(m.answer() == "kp = 0.300: refused (latched)" && m.answerBad(),
                    "refused shown");
    }

    Test::testCase("unsaved tunables");
    {
        Tui::Model m;
        Test::check(!m.unsaved(0) && m.unsavedCount() == 0, "nothing known yet");
        auto inUse             = Remote::defaults();
        inUse[Remote::kdMilli] = 900;
        m.apply(
          Protocol::Report{
            Protocol::Tunables{inUse, Remote::defaults()}
        },
          at(0ms));
        Test::check(m.unsaved(Remote::kdMilli) && !m.unsaved(Remote::kpMilli)
                      && m.unsavedCount() == 1,
                    "kd unsaved");
        auto const s = m.store();
        Test::check(m.answer() == "sent: store every tunable", "store sent");
        m.apply(
          Protocol::Report{
            Protocol::Ack{s.seq, Protocol::Ack::Code::storePending}
        },
          at(0ms));
        Test::check(m.answerBad() && m.answer().contains("retried"), "pending shown as a problem");
        m.apply(
          Protocol::Report{
            Protocol::Tunables{inUse, inUse}
        },
          at(0ms));
        Test::check(m.unsavedCount() == 0, "stored now");
    }

    Test::testCase("defaults into the flash: reset, then store, the store's answer shown");
    {
        Tui::Model m;
        auto const r = m.defaults();
        Test::check(m.answer().starts_with("sent: every tunable"), "defaults sent");
        auto const  both  = m.defaultsToFlash();
        auto const* reset = std::get_if<Protocol::ResetTunables>(both.data());
        auto const* store = std::get_if<Protocol::StoreTunables>(std::next(both.data()));
        Test::check(reset != nullptr && store != nullptr, "reset first, then store");
        if(reset != nullptr && store != nullptr) {
            Test::check(reset->seq != r.seq && store->seq != reset->seq, "new seqs");
            m.apply(
              Protocol::Report{
                Protocol::Ack{reset->seq, Protocol::Ack::Code::ok}
            },
              at(0ms));
            Test::check(m.answer() == "sent: defaults stored (all but the solder temp)",
                        "the reset's Ack is not the answer");
            m.apply(
              Protocol::Report{
                Protocol::Ack{store->seq, Protocol::Ack::Code::ok}
            },
              at(0ms));
            Test::check(m.answer() == "defaults stored (all but the solder temp): ok",
                        "the store's is");
        }
    }

    Test::testCase("dump lines");
    {
        auto const t = Tui::Model::dumpLine(Protocol::Report{telemetry(3204, 3200, 7)});
        Test::check(t.starts_with("telemetry tip=320.4 ") && t.contains("set=320.0")
                      && t.contains("heated=7/10"),
                    t.c_str());
        auto const v = Tui::Model::dumpLine(Protocol::Report{
          Protocol::Tunables{Remote::defaults(), Remote::defaults()}
        });
        Test::check(v.contains("kp=0.300") && v.contains("solder temp=320")
                      && v.contains("unsaved=-"),
                    v.c_str());
        auto inUse              = Remote::defaults();
        inUse[Remote::kpMilli]  = 400;
        inUse[Remote::standbyS] = 60;
        auto const u            = Tui::Model::dumpLine(Protocol::Report{
          Protocol::Tunables{inUse, Remote::defaults()}
        });
        Test::check(u.contains("unsaved=kp,standby after"), u.c_str());
        Protocol::Diagnostics d{};
        d.transformerDeciC = Protocol::Diagnostics::Unknown;
        auto const dl      = Tui::Model::dumpLine(Protocol::Report{d});
        Test::check(dl.contains("trafo=-"), dl.c_str());
    }

    Test::testCase("the chart");
    {
        std::deque<Tui::Sample> h;
        for(int i = 0; i < 100; ++i) {
            h.push_back(Tui::Sample{.tip          = static_cast<std::int16_t>(2000 + i * 10),
                                    .tipMin       = 0,
                                    .tipMax       = 0,
                                    .setpoint     = 3000,
                                    .pidCenti     = 0,
                                    .halfCycles   = 10,
                                    .heatedCycles = 10});
        }
        auto const c = Tui::temperatureChart(h, 100, 46, 8);
        Test::check(c.rows.size() == 8, "height");
        Test::check(near(c.low, 200) && near(c.high, 300), "range from the data, on round ticks");
        bool        widthsOk = true;
        std::size_t tipCells = 0;
        for(auto const& row : c.rows) {
            std::size_t cells = 0;
            for(auto const& run : row.runs) {
                // a braille glyph is 3 bytes of UTF-8, a blank 1
                for(std::size_t i = 0; i < run.text.size();) {
                    i += static_cast<unsigned char>(run.text[i]) >= 0x80 ? 3U : 1U;
                    ++cells;
                }
                if(run.style == "tip") { tipCells += run.text.size() / 3; }
            }
            widthsOk = widthsOk && cells == 40;
        }
        Test::check(widthsOk, "every row 40 cells");
        Test::check(tipCells >= 40, "a rising line touches every column");
        Test::check(c.rows.front().axis.contains("300"), "top label");
        Test::check(c.rows.back().axis.contains("200"), "bottom label");

        auto const shortHistory
          = Tui::temperatureChart(std::deque<Tui::Sample>(h.end() - 10, h.end()), 100, 46, 8);
        Test::check(shortHistory.rows.front().runs.front().style.empty(),
                    "a short history draws at the right only");

        auto const empty = Tui::temperatureChart({}, 100, 46, 8);
        Test::check(empty.rows.size() == 8 && near(empty.low, 0) && near(empty.high, 400),
                    "empty: 0..400");
    }

    Test::testCase("nice ticks");
    Test::check(near(Tui::niceStep(220, 8), 50), "220 over 8: 50");
    Test::check(near(Tui::niceStep(20, 2), 10), "20 over 2: 10");
    Test::check(near(Tui::niceStep(7, 4), 2), "7 over 4: 2");

    Test::testCase("tick labels on round values, a setpoint step joined");
    {
        std::deque<Tui::Sample> h;
        for(int i = 0; i < 100; ++i) {
            h.push_back(Tui::Sample{.tip          = 2500,
                                    .tipMin       = 0,
                                    .tipMax       = 0,
                                    .setpoint     = static_cast<std::int16_t>(i < 50 ? 2000 : 3500),
                                    .pidCenti     = 0,
                                    .halfCycles   = 10,
                                    .heatedCycles = 0});
        }
        auto const  c      = Tui::temperatureChart(h, 100, 46, 12);
        bool        round  = true;
        std::size_t labels = 0;
        for(auto const& row : c.rows) {
            if(!row.axis.contains("┤")) { continue; }
            ++labels;
            auto const first = row.axis.find_first_not_of(' ');
            int        v{};
            auto const end = row.axis.data() + row.axis.size();
            auto const r   = std::from_chars(row.axis.data() + first, end, v);
            round          = round && r.ec == std::errc{} && v % 10 == 0;
        }
        Test::check(labels >= 3 && round, "at least three labels, all round");
        // the step: the set series in the middle column region touches several rows
        std::size_t setRows = 0;
        for(auto const& row : c.rows) {
            for(auto const& run : row.runs) {
                if(run.style == "set" && run.text.size() <= 6) { ++setRows; }
            }
        }
        Test::check(setRows >= 3, "the step is drawn across the rows between");
    }

    Test::testCase("the duty chart and the big digits");
    {
        std::deque<Tui::Sample> h(20, Tui::Sample{0, 0, 0, 0, 0, 10, 5});
        auto const              d = Tui::dutyChart(h, 20, 16, 3);
        Test::check(d.rows.size() == 3, "three rows");
        Test::check(count(d.rows[2].runs.back().text, "█") == 10
                      && count(d.rows[1].runs.back().text, "▄") == 10
                      && count(d.rows[0].runs.back().text, " ") == 10,
                    "half duty: the bottom full, the middle half, the top empty");
        Test::check(d.rows[0].axis.contains("100%") && d.rows[2].axis.contains("0%"), "the axis");
        std::deque<Tui::Sample> little(20, Tui::Sample{0, 0, 0, 0, 0, 100, 1});
        auto const              l = Tui::dutyChart(little, 20, 16, 3);
        Test::check(count(l.rows[2].runs.back().text, "▁") == 10, "1 % still shows");
        auto const half
          = Tui::dutyChart(std::deque<Tui::Sample>(h.begin(), h.begin() + 10), 20, 16, 3);
        Test::check(half.rows[2].runs.front().style.empty()
                      && half.rows[2].runs.back().style == "duty",
                    "a short history at the right");
        auto const big = Tui::bigText("1.5");
        Test::check(count(big, "\n") == 2, "three rows");
    }

    Test::testCase("frames from the host's reader");
    {
        Host::FrameReader<Protocol::Report> reader;
        auto const                          a = Host::frame(Protocol::Report{status(1)});
        auto const                          b = Host::frame(Protocol::Report{telemetry(1, 2, 3)});
        std::vector<std::byte>              stream{std::byte{0x42}};
        stream.insert(stream.end(), a.begin(), a.end());
        stream.insert(stream.end(), b.begin(), b.end());
        reader.append(std::span<std::byte const>{stream}.first(stream.size() - 3));
        auto const first = reader.next();
        Test::check(first && std::holds_alternative<Protocol::Status>(*first),
                    "first after garbage");
        Test::check(!reader.next(), "second not whole yet");
        reader.append(std::span<std::byte const>{stream}.last(3));
        auto const second = reader.next();
        Test::check(second && std::holds_alternative<Protocol::Telemetry>(*second),
                    "second once whole");
    }
    return Test::finish();
}
