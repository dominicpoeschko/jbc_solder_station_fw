#pragma once
// The remote's graphs as text: a braille chart (2x4 dots a cell) of the tip temperature over its
// setpoint, and a block sparkline of the heater's duty - plain functions over Model's history, so
// the tests can look at them without a screen. A chart row comes as runs of one style each ("tip",
// "set" or ""), which the screen colours.
#include "Model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace Tui {

struct Run {
    std::string text;
    std::string style;   ///< "tip", "set" or "" (empty space)

    bool operator==(Run const&) const = default;
};

struct ChartRow {
    std::string      axis;   ///< the left label, padded to AxisWidth
    std::vector<Run> runs;
};

struct Chart {
    std::vector<ChartRow> rows;
    double                low{};
    double                high{};
};

inline constexpr std::size_t AxisWidth = 6;

namespace Detail {
    /// One cell's braille dot bit for dot column `dx` (0, 1) and dot row `dy` (0 top .. 3 bottom).
    constexpr std::uint8_t brailleBit(std::size_t dx,
                                      std::size_t dy) {
        constexpr std::array<std::array<std::uint8_t, 4>, 2> Bits{
          {{0x01, 0x02, 0x04, 0x40}, {0x08, 0x10, 0x20, 0x80}}
        };
        return Bits[dx][dy];
    }

    /// U+2800 + bits as UTF-8.
    inline std::string braille(std::uint8_t bits) {
        auto const cp = 0x2800U + bits;
        return {static_cast<char>(0xE0U | (cp >> 12U)),
                static_cast<char>(0x80U | ((cp >> 6U) & 0x3FU)),
                static_cast<char>(0x80U | (cp & 0x3FU))};
    }

    /// The samples of `history`'s last `window` reduced to `columns` values (each the mean of its
    /// bucket), oldest first; columns without data are NaN (the history is shorter than the window).
    template<typename Get>
    std::vector<double> buckets(std::deque<Sample> const& history,
                                std::size_t               window,
                                std::size_t               columns,
                                Get                       get) {
        std::vector<double> out(columns, std::nan(""));
        if(columns == 0 || window == 0) { return out; }
        auto const start = history.size() > window ? history.size() - window : 0;
        // the newest sample sits at the right edge; the window is laid out over all columns
        auto const offset = window - (history.size() - start);
        for(std::size_t c = 0; c < columns; ++c) {
            auto const  from = c * window / columns;
            auto const  to   = std::max(from + 1, (c + 1) * window / columns);
            double      sum  = 0;
            std::size_t n    = 0;
            for(auto i = from; i < to; ++i) {
                if(i < offset) { continue; }
                sum += get(history[start + i - offset]);
                ++n;
            }
            if(n != 0) { out[c] = sum / static_cast<double>(n); }
        }
        return out;
    }
}   // namespace Detail

/// A tick step for `span` over about `ticks` labels: 1, 2, 2.5 or 5 times a power of ten.
inline double niceStep(double      span,
                       std::size_t ticks) {
    auto const rough = span / static_cast<double>(std::max<std::size_t>(ticks, 1));
    auto const power = std::pow(10.0, std::floor(std::log10(std::max(rough, 1e-9))));
    for(auto const m : {1.0, 2.0, 2.5, 5.0, 10.0}) {
        if(m * power >= rough) { return m * power; }
    }
    return 10.0 * power;
}

/// The chart of the last `window` samples, `width` cells wide (axis included) and `height` rows.
/// The range follows the data, snapped to round ticks (so it moves in steps, not with every
/// sample), at least 20 °C; the ticks are labelled on the rows they fall on.
inline Chart temperatureChart(std::deque<Sample> const& history,
                              std::size_t               window,
                              std::size_t               width,
                              std::size_t               height) {
    Chart chart;
    if(width <= AxisWidth + 1 || height == 0) { return chart; }
    auto const cells   = width - AxisWidth;
    auto const columns = cells * 2;
    auto const tip
      = Detail::buckets(history, window, columns, [](Sample const& s) { return s.tip / 10.0; });
    auto const set = Detail::buckets(history, window, columns, [](Sample const& s) {
        return s.setpoint / 10.0;
    });

    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for(std::size_t c = 0; c < columns; ++c) {
        for(auto const v : {tip[c], set[c]}) {
            if(!std::isnan(v)) {
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
        }
    }
    if(lo > hi) {
        lo = 0;
        hi = 400;
    }
    if(hi - lo < 20) {
        auto const mid = (lo + hi) / 2;
        lo             = mid - 10;
        hi             = mid + 10;
    }
    auto const step = niceStep(hi - lo, std::max<std::size_t>(height / 3, 2));
    lo              = std::floor(lo / step) * step;
    hi              = std::ceil(hi / step) * step;
    if(hi <= lo) { hi = lo + step; }
    chart.low  = lo;
    chart.high = hi;

    auto const dots = height * 4;
    // a value's dot row, 0 at the top
    auto const rowOf = [&](double v) {
        auto const r
          = static_cast<long>(std::lround((hi - v) / (hi - lo) * static_cast<double>(dots - 1)));
        return static_cast<std::size_t>(std::clamp<long>(r, 0, static_cast<long>(dots) - 1));
    };

    std::vector<std::uint8_t> tipBits(cells * height, 0);
    std::vector<std::uint8_t> setBits(cells * height, 0);
    auto const plot = [&](std::vector<std::uint8_t>& bits, std::size_t column, std::size_t row) {
        bits[(row / 4) * cells + column / 2] |= Detail::brailleBit(column % 2, row % 4);
    };
    std::optional<std::size_t> previous;
    std::optional<std::size_t> previousSet;
    for(std::size_t c = 0; c < columns; ++c) {
        if(std::isnan(set[c])) {
            previousSet.reset();
        } else {
            // a setpoint step drawn as a step
            auto const r    = rowOf(set[c]);
            auto const from = previousSet ? std::min(*previousSet, r) : r;
            auto const to   = previousSet ? std::max(*previousSet, r) : r;
            for(auto y = from; y <= to; ++y) { plot(setBits, c, y); }
            previousSet = r;
        }
        if(std::isnan(tip[c])) {
            previous.reset();
            continue;
        }
        auto const r = rowOf(tip[c]);
        // a steep step joined up, so the line stays a line
        auto const from = previous ? std::min(*previous, r) : r;
        auto const to   = previous ? std::max(*previous, r) : r;
        for(auto y = from; y <= to; ++y) { plot(tipBits, c, y); }
        previous = r;
    }

    // each tick on the row its dot falls in; one label a row
    std::vector<std::optional<double>> labels(height);
    auto const ticks = static_cast<std::size_t>(std::lround((hi - lo) / step));
    for(std::size_t k = 0; k <= ticks; ++k) {
        auto const t    = hi - static_cast<double>(k) * step;
        auto&      slot = labels[rowOf(t) / 4];
        if(!slot) { slot = t; }
    }
    for(std::size_t row = 0; row < height; ++row) {
        ChartRow line;
        if(auto const& tick = labels[row]; tick.has_value()) {
            char label[24];
            std::snprintf(label, sizeof(label), "%4.0f ┤", tick.value_or(0.0));
            line.axis = label;
        } else {
            line.axis = "     │";
        }
        for(std::size_t cell = 0; cell < cells; ++cell) {
            auto const t     = tipBits[row * cells + cell];
            auto const s     = setBits[row * cells + cell];
            auto const style = t != 0 ? "tip" : s != 0 ? "set" : "";
            auto const glyph
              = (t | s) == 0 ? std::string{" "} : Detail::braille(static_cast<std::uint8_t>(t | s));
            if(line.runs.empty() || line.runs.back().style != style) {
                line.runs.push_back(Run{glyph, style});
            } else {
                line.runs.back().text += glyph;
            }
        }
        chart.rows.push_back(std::move(line));
    }
    return chart;
}

/// The heater's duty (heated half-cycles of all) over the same window as a bar chart `height` rows
/// tall, one bar a chart cell (so it lines up with temperatureChart of the same width), eighth
/// blocks: 8 * height levels. Its runs carry the style "duty" ("" where there is no data yet).
inline Chart dutyChart(std::deque<Sample> const& history,
                       std::size_t               window,
                       std::size_t               width,
                       std::size_t               height) {
    static constexpr std::array<char const*, 9> Blocks{" ", "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"};
    Chart                                       chart{.rows = {}, .low = 0, .high = 100};
    if(width <= AxisWidth + 1 || height == 0) { return chart; }
    auto const cells  = width - AxisWidth;
    auto const duty   = Detail::buckets(history, window, cells, [](Sample const& s) {
        return s.halfCycles == 0 ? 0.0 : static_cast<double>(s.heatedCycles) / s.halfCycles;
    });
    auto const levels = static_cast<double>(height * 8);
    for(std::size_t row = 0; row < height; ++row) {
        ChartRow line;
        line.axis       = row == 0 ? "100% ┤" : row == height - 1 ? "  0% ┤" : "     │";
        auto const base = (height - 1 - row) * 8;   // the levels below this row
        for(auto const d : duty) {
            auto const level
              = std::isnan(d)
                ? 0U
                : static_cast<std::size_t>(std::lround(std::clamp(d, 0.0, 1.0) * levels));
            auto fill = level > base ? std::min<std::size_t>(level - base, 8) : 0;
            // any duty above 0 shows as at least a sliver on the bottom row
            if(row == height - 1 && fill == 0 && !std::isnan(d) && d > 0) { fill = 1; }
            std::string const style = std::isnan(d) ? "" : "duty";
            if(line.runs.empty() || line.runs.back().style != style) {
                line.runs.push_back(Run{Blocks[fill], style});
            } else {
                line.runs.back().text += Blocks[fill];
            }
        }
        chart.rows.push_back(std::move(line));
    }
    return chart;
}

/// `text` (digits, '.', '-', ' ') in a three-row block font: three lines joined by '\n'.
inline std::string bigText(std::string_view text) {
    struct Glyph {
        char                       c;
        std::array<char const*, 3> rows;
    };

    static constexpr std::array<Glyph, 13> Font{
      {
       {'0', {"█▀█", "█ █", "▀▀▀"}},
       {'1', {" ▀█", "  █", "  ▀"}},
       {'2', {"▀▀█", "█▀▀", "▀▀▀"}},
       {'3', {"▀▀█", " ▀█", "▀▀▀"}},
       {'4', {"█ █", "▀▀█", "  ▀"}},
       {'5', {"█▀▀", "▀▀█", "▀▀▀"}},
       {'6', {"█▀▀", "█▀█", "▀▀▀"}},
       {'7', {"▀▀█", "  █", "  ▀"}},
       {'8', {"█▀█", "█▀█", "▀▀▀"}},
       {'9', {"█▀█", "▀▀█", "▀▀▀"}},
       {'.', {" ", " ", "▀"}},
       {'-', {"   ", "▀▀▀", "   "}},
       {' ', {"   ", "   ", "   "}},
       }
    };
    std::array<std::string, 3> rows;
    for(auto const c : text) {
        auto const* glyph = &Font.back();
        for(auto const& g : Font) {
            if(g.c == c) { glyph = &g; }
        }
        for(std::size_t r = 0; r < 3; ++r) {
            if(!rows[r].empty()) { rows[r] += ' '; }
            rows[r] += glyph->rows[r];
        }
    }
    return rows[0] + "\n" + rows[1] + "\n" + rows[2];
}

}   // namespace Tui
