// jbc_tui: the soldering station's remote, on RTXUI (https://github.com/ArthurSonzogni/RTXUI). It
// speaks remote/Protocol.hpp to the station over the probe (the socket uc_log_printer opens for
// the firmware's "control" duplex channel: `just log` in another terminal) or over SERCOM1's UART on
// the debug pins (a USB serial adapter on PA18/PA19), shows what the station reports - the tip
// against its setpoint, the heater's duty, the state machine, the safety latch, diagnostics - and
// sets the tunables (remote/Tunables.hpp).
//
//   jbc_tui                                   the running printer's duplex.0.sock (this tree)
//   jbc_tui <duplex.0.sock | /dev/ttyUSB0> [baud=115200]
//   jbc_tui --dump [path] [until=REGEX ...] [defaults] [set=INDEX:VALUE ...] [store] [for=SECONDS]
//   jbc_tui --render [path] [WxH] [until=REGEX] [for=SECONDS]   the screen once, as text
//
// --dump: no screen. Each report is printed as one line (Model::dumpLine); `set=` sends a SetTunable
// once the station's Tunables are there (seq 100, 101, ...), `defaults` a ResetTunables before
// them, `store` a StoreTunables after them (`defaults store`: the defaults into the flash, as F F);
// `until=` waits for a line matching it (in order, each after the one before); `for=` stops after that long (without until= and for=: two seconds). Exit 0:
// every until= matched (or the time ran out without one asked for); 1: no link; 2: a bad argument;
// 3: an until= did not match in time.
//
// Keys: up/down pick a tunable, left/right step it, [ ] by ten, Enter types a value, d its default,
// R R every tunable but the solder temperature back to its default (RAM), s stores them all in the
// station's flash (a change is RAM only until then: ● marks it), F F both - the defaults into the
// flash, for a stored bad set (the tuning card has the three as buttons too), c clears the counters, g asks for everything, 1-4 the graph's window,
// q quits.
#include "Link.hpp"
#include "Model.hpp"
#include "Plot.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <rtxui/rtxui.hpp>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <variant>
#include <vector>

namespace {

using namespace std::chrono_literals;

/// Where this tree's printers put their sockets (tui/CMakeLists.txt; $JBC_RTT_LOG_DIR overrides it).
std::filesystem::path rttLogDir() {
    if(char const* dir = std::getenv("JBC_RTT_LOG_DIR")) { return dir; }
    return TUI_RTT_LOG_DIR;
}

/// The link's path as the title bar shows it: its tail, "…" in front past 32 bytes. The bar has no room to spare at
/// 140 columns - a longer path squeezes every span in it ("JBC station remot", "cup 00:00:0").
std::string barPath(std::string const& path) {
    constexpr std::size_t MaxBytes = 32;
    if(path.size() <= MaxBytes) { return path; }
    auto cut = path.size() - MaxBytes;
    while(cut < path.size() && (static_cast<unsigned char>(path[cut]) & 0xC0U) == 0x80U) { ++cut; }
    return "…" + path.substr(cut);
}

/// The graph's windows: 1..4.
constexpr std::array<std::pair<std::size_t, char const*>, 4> Windows{
  {{150, "15 s"}, {300, "30 s"}, {600, "1 min"}, {1'200, "2 min"}}
};

struct RunView {
    std::string text;
    std::string style;

    bool operator==(RunView const&) const = default;
};

void Fields(RunView const&    r,
            rtxui::FieldSink& out) {
    out("text", r.text);
    out("style", r.style);
}

struct KeyValue {
    std::string key;
    std::string value;
    std::string style;

    bool operator==(KeyValue const&) const = default;
};

void Fields(KeyValue const&   kv,
            rtxui::FieldSink& out) {
    out("key", kv.key);
    out("value", kv.value);
    out("style", kv.style);
}

struct TuneRow {
    std::string name;
    std::string value;
    std::string range;
    std::string style;

    bool operator==(TuneRow const&) const = default;
};

void Fields(TuneRow const&    t,
            rtxui::FieldSink& out) {
    out("name", t.name);
    out("value", t.value);
    out("range", t.range);
    out("style", t.style);
}

std::string css(std::uint8_t r,
                std::uint8_t g,
                std::uint8_t b) {
    // the KTD2026's currents are small numbers (1..10 here): scaled up to something visible
    auto const up
      = [](std::uint8_t v) { return std::to_string(v == 0 ? 40 : std::min(255, 90 + v * 16)); };
    if((r | g | b) == 0) { return "rgb(40, 40, 40)"; }
    return "rgb(" + up(r) + ", " + up(g) + ", " + up(b) + ")";
}

std::string clock(std::uint64_t seconds) {
    char text[32];
    std::snprintf(text,
                  sizeof(text),
                  "%02llu:%02llu:%02llu",
                  static_cast<unsigned long long>(seconds / 3600),
                  static_cast<unsigned long long>(seconds / 60 % 60),
                  static_cast<unsigned long long>(seconds % 60));
    return text;
}

class StationApp : public rtxui::Component<StationApp> {
public:
    // header
    std::string linkText   = "starting";
    std::string linkClass  = "bad";
    std::string stateText  = "-";
    std::string stateClass = "state-off";
    std::string ledSwatch  = "rgb(40, 40, 40)";
    std::string uptimeText = "-";
    std::string latchText  = "-";
    std::string latchClass = "muted";
    int         latchUsed  = 0;   // of 3600 s
    std::string resetText;

    // the iron
    std::string          tipText      = "---.-";
    std::string          tipBig       = Tui::bigText("---.-");
    std::string          tipClass     = "tip-cold";
    std::string          setText      = "-";
    std::string          presetText   = "-";
    std::string          presetClass  = "";
    std::string          spreadText   = "";
    std::string          heaterText   = "off";
    std::string          heaterClass  = "lamp-off";
    int                  dutyPermille = 0;
    std::string          dutyText     = "-";
    std::string          pidText      = "-";
    std::string          tipPosText   = "-";
    std::string          tipPosClass  = "muted";
    std::vector<RunView> chart;
    std::vector<RunView> duty;
    std::string          windowText = "30 s";
    std::string          rangeText;

    // diagnostics and tunables
    std::vector<KeyValue> diagnostics;
    std::vector<TuneRow>  tunables;
    std::string           help;
    std::string           entryText;
    std::string           entryClass = "hidden";
    std::string           answerText;
    std::string           answerClass  = "muted";
    std::string           factoryLabel = "Defaults → flash (F F)";
    std::string           storeText;
    std::string           storeClass = "muted";

    std::string_view view = R"html(
      <div class="app">
        <div class="bar">
          <span class="brand">JBC</span>
          <span class="muted">station remote</span>
          <span class="{linkClass}">● {linkText}</span>
          <span class="spacer"></span>
          <span class="pill {stateClass}"> {stateText} </span>
          <span class="swatch" style="background-color: {ledSwatch};">  </span>
          <span class="muted">up</span><span>{uptimeText}</span>
          <span class="muted">latch in</span><span class="{latchClass}">{latchText}</span>
          <progress value="{latchUsed}" max="3600" width="14" />
          <span class="warn">{resetText}</span>
        </div>

        <div class="body">
          <div class="left">
            <div class="card iron">
              <div class="readout">
                <div class="big-col">
                  <span class="label">TIP °C</span>
                  <span class="big {tipClass}">{tipBig}</span>
                  <span class="muted">{spreadText}</span>
                </div>
                <div class="side-col">
                  <div class="kv"><span class="ik">setpoint</span><span class="iv">{setText}</span></div>
                  <div class="kv"><span class="ik">knob</span><span class="iv {presetClass}">{presetText}</span></div>
                  <div class="kv"><span class="ik">tip</span><span class="iv {tipPosClass}">{tipPosText}</span></div>
                  <div class="kv"><span class="ik">heater</span><span class="iv {heaterClass}">{heaterText}</span></div>
                  <div class="kv"><span class="ik">PID out</span><span class="iv">{pidText}</span></div>
                  <div class="kv"><span class="ik">duty</span><span class="iv">{dutyText}</span></div>
                  <progress value="{dutyPermille}" max="1000" width="16" />
                </div>
              </div>
            </div>

            <div class="card graph">
              <div class="graph-head">
                <span class="label">TEMPERATURE</span>
                <span class="tip-key">━ tip</span>
                <span class="set-key">━ setpoint</span>
                <span class="spacer"></span>
                <span class="muted">window {windowText} (1-4)</span>
              </div>
              <div class="chart"><for each="{chart}" as="run"><span class="{run.style}">{run.text}</span></for></div>
              <div class="graph-head"><span class="label">HEATER DUTY</span><span class="muted">heated half-cycles of all</span></div>
              <div class="chart"><for each="{duty}" as="run"><span class="{run.style}">{run.text}</span></for></div>
            </div>
          </div>

          <div class="right">
            <div class="card">
              <div class="graph-head"><span class="label">TUNING</span><span class="spacer"></span><span class="{storeClass}">{storeText}</span></div>
              <div class="tune-head"><span class="t-name">name</span><span class="t-value">value</span><span class="t-range">range (default)</span></div>
              <for each="{tunables}" as="row">
                <div class="tune {row.style}">
                  <span class="t-name">{row.name}</span>
                  <span class="t-value">{row.value}</span>
                  <span class="t-range">{row.range}</span>
                </div>
              </for>
              <div class="actions">
                <button class="act" onclick="Store">Store (s)</button>
                <button class="act" onclick="Defaults">Defaults (R R)</button>
                <button class="act danger" onclick="DefaultsToFlash">{factoryLabel}</button>
              </div>
              <span class="help">{help}</span>
              <span class="{entryClass}">{entryText}</span>
              <span class="{answerClass}">{answerText}</span>
            </div>

            <div class="card">
              <span class="label">DIAGNOSTICS</span>
              <for each="{diagnostics}" as="d">
                <div class="kv"><span class="k">{d.key}</span><span class="v {d.style}">{d.value}</span></div>
              </for>
            </div>
          </div>
        </div>

        <div class="footer">
          <span class="key">↑↓</span><span>pick</span>
          <span class="key">←→ [ ]</span><span>step</span>
          <span class="key">Enter</span><span>type</span>
          <span class="key">d</span><span>default</span>
          <span class="key">R R</span><span>defaults</span>
          <span class="key">s</span><span>store to flash</span>
          <span class="key">F F</span><span>defaults to flash</span>
          <span class="key">c</span><span>clear counters</span>
          <span class="key">g</span><span>refresh</span>
          <span class="key">1-4</span><span>window</span>
          <span class="key">q</span><span>quit</span>
        </div>
      </div>

      <style>
        self {
          --bg: rgb(13, 17, 23);
          --surface: rgb(22, 27, 34);
          --border: rgb(48, 54, 61);
          --muted: rgb(139, 148, 158);
          --text: rgb(230, 237, 243);
          --accent: rgb(88, 166, 255);
          --good: rgb(63, 185, 80);
          --warn: rgb(210, 153, 34);
          --bad: rgb(248, 81, 73);
          --hot: rgb(255, 123, 54);
          display: block;
          width: 100%;
          height: 100%;
          background-color: var(--bg);
          color: var(--text);
        }
        .app { display: flex; flex-direction: column; height: 100%; }
        .bar {
          display: flex; gap: 1; padding: 0 1; white-space: nowrap; overflow: hidden;
          background-color: var(--surface);
          border-bottom: solid; border-color: var(--border);
        }
        .brand { color: var(--hot); font-weight: bold; }
        .spacer { flex-grow: 1; }
        .muted { color: var(--muted); }
        .good { color: var(--good); }
        .warn { color: var(--warn); font-weight: bold; }
        .bad { color: var(--bad); font-weight: bold; }
        .hidden { display: none; }
        .swatch { width: 2; }
        .pill { font-weight: bold; color: rgb(13, 17, 23); }
        .state-solder { background-color: var(--hot); }
        .state-sleep { background-color: var(--good); }
        .state-standby { background-color: var(--warn); }
        .state-remove { background-color: rgb(163, 113, 247); }
        .state-shutdown { background-color: var(--bad); color: white; }
        .state-off { background-color: var(--border); color: var(--text); }

        .body { display: flex; flex-grow: 1; gap: 1; padding: 0 1; overflow: hidden; }
        .left { display: flex; flex-direction: column; flex-grow: 1; flex-shrink: 1; gap: 0; overflow: hidden; }
        .right { display: flex; flex-direction: column; width: 60; flex-shrink: 0; overflow: hidden; }
        .card { border: round; border-color: var(--border); padding: 0 1; display: flex; flex-direction: column; }
        .label { color: var(--accent); font-weight: bold; }

        .readout { display: flex; gap: 3; }
        .big-col { display: flex; flex-direction: column; width: 26; flex-shrink: 0; }
        .big { font-weight: bold; white-space: pre; }
        .tip-cold { color: var(--accent); }
        .tip-warm { color: var(--warn); }
        .tip-hot { color: var(--hot); }
        .tip-on { color: var(--good); }
        .side-col { display: flex; flex-direction: column; flex-grow: 1; }
        .kv { display: flex; }
        .k { width: 17; color: var(--muted); }
        .v { flex-grow: 1; white-space: nowrap; overflow: hidden; }
        .ik { width: 10; flex-shrink: 0; color: var(--muted); }
        .iv { flex-grow: 1; white-space: nowrap; overflow: hidden; }
        .lamp-on { color: var(--hot); font-weight: bold; }
        .lamp-off { color: var(--muted); }
        .blink { color: var(--warn); font-weight: bold; }

        .graph { flex-grow: 1; }
        .graph-head { display: flex; gap: 2; }
        .tip-key { color: var(--hot); }
        .set-key { color: var(--accent); }
        .chart { white-space: pre; overflow: hidden; }
        .axis { color: var(--muted); }
        .tip { color: var(--hot); }
        .set { color: var(--accent); }
        .duty { color: var(--good); }

        .tune-head { display: flex; color: var(--muted); }
        .tune { display: flex; }
        .tune.selected { background-color: rgb(30, 46, 84); font-weight: bold; }
        .tune.dim { color: var(--muted); }
        .t-name { width: 17; }
        .t-value { width: 18; }
        .t-range { flex-grow: 1; color: var(--muted); }
        .help { color: var(--muted); margin-top: 1; }
        .actions { display: flex; gap: 1; margin-top: 1; }
        .act { background-color: rgb(33, 38, 45); color: var(--text); padding: 0 1; }
        .act:hover, .act:focus { background-color: rgb(56, 139, 253); color: white; }
        .danger { color: var(--warn); }
        .danger:hover, .danger:focus { background-color: rgb(158, 106, 3); color: white; }
        .entry { color: var(--text); background-color: rgb(33, 38, 45); }
        .entry-bad { color: var(--bad); background-color: rgb(33, 38, 45); }

        .footer { display: flex; gap: 1; padding: 0 1; color: var(--muted); background-color: var(--surface); white-space: nowrap; overflow: hidden; }
        .key { color: var(--accent); font-weight: bold; }
      </style>
    )html";

    StationApp() {
        Bind(linkText);
        Bind(linkClass);
        Bind(stateText);
        Bind(stateClass);
        Bind(ledSwatch);
        Bind(uptimeText);
        Bind(latchText);
        Bind(latchClass);
        Bind(latchUsed);
        Bind(resetText);
        Bind(tipText);
        Bind(tipBig);
        Bind(tipClass);
        Bind(setText);
        Bind(presetText);
        Bind(presetClass);
        Bind(spreadText);
        Bind(heaterText);
        Bind(heaterClass);
        Bind(dutyPermille);
        Bind(dutyText);
        Bind(pidText);
        Bind(tipPosText);
        Bind(tipPosClass);
        Bind(chart);
        Bind(duty);
        Bind(windowText);
        Bind(rangeText);
        Bind(diagnostics);
        Bind(tunables);
        Bind(help);
        Bind(entryText);
        Bind(entryClass);
        Bind(answerText);
        Bind(answerClass);
        Bind(storeText);
        Bind(factoryLabel);
        Bind(Store);
        Bind(Defaults);
        Bind(DefaultsToFlash);
        Bind(storeClass);
        rebuild();
    }

    void connect(std::string    path,
                 unsigned       baud,
                 rtxui::Screen* screen) {
        screen_ = screen;
        link_   = std::make_unique<Host::Link>(
          std::move(path),
          rttLogDir(),
          baud,
          [this](Protocol::Report const& r) {
              rtxui::PostTask([this, r] {
                  model_.apply(r, Tui::Clock::now());
                  rebuild();
              });
          },
          [this](Host::LinkEvent e, std::string const& p) {
              rtxui::PostTask([this, e, p] {
                  switch(e) {
                  case Host::LinkEvent::missing: model_.linkMissing(p); break;
                  case Host::LinkEvent::refused: model_.linkRefused(p); break;
                  case Host::LinkEvent::up:      model_.linkUp(p, Tui::Clock::now()); break;
                  }
                  rebuild();
              });
          });
        tick();
    }

    bool OnEvent(rtxui::Event event) override {
        if(auto const size = event.get_if<rtxui::Event::Resized>()) {
            columns_ = static_cast<std::size_t>(std::max(0, size->width()));
            rows_    = static_cast<std::size_t>(std::max(0, size->height()));
            rebuild();
            return Component<StationApp>::OnEvent(event);
        }
        auto const key = event.get_if<rtxui::Event::Keyboard>();
        if(!key || key->motion() == rtxui::Event::Keyboard::Released) {
            return Component<StationApp>::OnEvent(event);
        }
        using K = rtxui::Event::Keyboard;
        if(entering_) {
            if(key->special() == K::Escape) {
                entering_ = false;
            } else if(key->special() == K::Return) {
                if(auto const c = model_.typed(entry_)) {
                    send(*c);
                    entering_ = false;
                }
            } else if(key->special() == K::Backspace) {
                if(!entry_.empty()) { entry_.pop_back(); }
            } else if(auto const cp = key->codepoint();
                      (cp >= '0' && cp <= '9') || cp == '.' || cp == '-')
            {
                entry_ += static_cast<char>(cp);
            }
            rebuild();
            return true;
        }
        auto const cp = key->codepoint();
        if(cp != 'R' && cp != 'F') { armed_ = Armed::none; }
        switch(key->special()) {
        case K::ArrowUp:    model_.select(-1); break;
        case K::ArrowDown:  model_.select(1); break;
        case K::ArrowLeft:  stepBy(-1); break;
        case K::ArrowRight: stepBy(1); break;
        case K::Return:
            entering_ = true;
            entry_.clear();
            break;
        default:
            switch(cp) {
            case '[': stepBy(-10); break;
            case ']': stepBy(10); break;
            case 'd':
                if(auto const c = model_.toDefault()) { send(*c); }
                break;
            case 'R': Defaults(); return true;
            case 'F': DefaultsToFlash(); return true;
            case 's': Store(); return true;
            case 'c': send(Protocol::ClearCounters{model_.nextSeq("clear counters")}); break;
            case 'g': send(Protocol::Get{model_.nextSeq("get")}); break;
            case '1':
            case '2':
            case '3':
            case '4': window_ = static_cast<std::size_t>(cp - '1'); break;
            case 'q':
                if(screen_ != nullptr) { screen_->Exit(); }
                return true;
            default: return Component<StationApp>::OnEvent(event);
            }
        }
        rebuild();
        return true;
    }

    // the tuning card's buttons; a key does the same

    void Store() {
        armed_ = Armed::none;
        send(model_.store());
        rebuild();
    }

    void Defaults() {
        if(armed_ == Armed::defaults) {
            armed_ = Armed::none;
            send(model_.defaults());
        } else {
            armed_ = Armed::defaults;
        }
        rebuild();
    }

    void DefaultsToFlash() {
        if(armed_ == Armed::factory) {
            armed_ = Armed::none;
            for(auto const& c : model_.defaultsToFlash()) { send(c); }
        } else {
            armed_ = Armed::factory;
        }
        rebuild();
    }

private:
    void stepBy(int steps) {
        if(auto const c = model_.step(steps)) { send(*c); }
    }

    void send(Protocol::Command const& c) {
        if(!link_ || !link_->send(c)) { model_.nextSeq("not sent: no link"); }
    }

    /// Twice a second: the link state ages (silent) without a report to say so.
    void tick() {
        rebuild();
        rtxui::PostDelayedTask([this] { tick(); }, 500ms);
    }

    void rebuild() {
        auto const now  = Tui::Clock::now();
        auto const link = model_.link(now);
        linkText        = std::string{Tui::name(link)}
                        + (model_.path().empty() ? "" : "  " + barPath(model_.path()));
        linkClass       = link == Tui::LinkState::live    ? "good"
                        : link == Tui::LinkState::waiting ? "warn"
                                                          : "bad";
        if(link == Tui::LinkState::missing) { linkText += "  (is `just log` running?)"; }

        rebuildHeader(now);
        rebuildIron();
        rebuildChart();
        rebuildTunables();
        rebuildDiagnostics();
        answerText  = model_.answer();
        answerClass = model_.answerBad() ? "bad" : "muted";
        factoryLabel
          = armed_ == Armed::factory ? "Click again: → flash!" : "Defaults → flash (F F)";
        if(armed_ == Armed::defaults) {
            answerText
              = "R (or the button) again: every tunable but the solder temp to its default, in RAM";
            answerClass = "warn";
        } else if(armed_ == Armed::factory) {
            answerText
              = "F (or the button) again: the defaults into the flash - the stored set is gone";
            answerClass = "warn";
        }
    }

    void rebuildHeader(Tui::TimePoint now) {
        auto const& s = model_.status();
        if(!s) {
            stateText  = "-";
            stateClass = "state-off";
            return;
        }
        stateText  = std::string{Tui::name(s->state)};
        stateClass = s->state == Protocol::State::solder   ? "state-solder"
                   : s->state == Protocol::State::sleep    ? "state-sleep"
                   : s->state == Protocol::State::standby  ? "state-standby"
                   : s->state == Protocol::State::remove   ? "state-remove"
                   : s->state == Protocol::State::shutdown ? "state-shutdown"
                                                           : "state-off";
        ledSwatch  = css(s->ledR, s->ledG, s->ledB);
        uptimeText = clock(s->uptimeMs / 1000);
        if(model_.latched()) {
            latchText  = "LATCHED - power-cycle";
            latchClass = "bad";
            latchUsed  = 3600;
        } else {
            latchText  = clock(s->latchInS);
            latchClass = s->latchInS < 600 ? "warn" : "muted";
            latchUsed  = 3600 - static_cast<int>(std::min<std::uint32_t>(s->latchInS, 3600));
        }
        resetText.clear();
        if(model_.resets() != 0 && model_.lastReset()) {
            auto const ago
              = std::chrono::duration_cast<std::chrono::seconds>(now - *model_.lastReset()).count();
            if(ago < 30) { resetText = "station reset " + std::to_string(ago) + " s ago"; }
        }
    }

    void rebuildIron() {
        auto const& t = model_.telemetry();
        auto const& s = model_.status();
        if(t) {
            tipText         = Tui::deciC(t->tipDeciC);
            tipBig          = Tui::bigText(tipText);
            auto const diff = t->tipDeciC - t->setpointDeciC;
            tipClass        = t->setpointDeciC > 0 && diff > -50 && diff < 50 ? "tip-on"
                            : t->tipDeciC >= 1500                             ? "tip-hot"
                            : t->tipDeciC >= 500                              ? "tip-warm"
                                                                              : "tip-cold";
            spreadText
              = Tui::deciC(t->tipMinDeciC) + " .. " + Tui::deciC(t->tipMaxDeciC) + " in 100 ms";
            setText = Tui::deciC(t->setpointDeciC) + " °C";
            pidText = Tui::formatFixed(t->pidCenti, 2) + " / 10";
            dutyPermille
              = t->halfCycles == 0 ? 0 : static_cast<int>(1000U * t->heatedCycles / t->halfCycles);
            dutyText = std::to_string(t->heatedCycles) + "/" + std::to_string(t->halfCycles)
                     + " half-cycles";
        }
        if(s) {
            presetText       = std::to_string(s->presetC) + " °C";
            bool const dirty = (s->flags & Protocol::Flag::presetDirty) != 0;
            if(dirty) { presetText += "  (press to take)"; }
            presetClass   = dirty ? "blink" : "";
            bool const on = (s->flags & Protocol::Flag::heaterOn) != 0;
            heaterText    = on ? "ON" : "off";
            heaterClass   = on ? "lamp-on" : "lamp-off";
            if((s->flags & Protocol::Flag::tipNotHeating) != 0) {
                heaterText  = "off: tip did not warm (power cycle)";
                heaterClass = "bad";
            } else if((s->flags & Protocol::Flag::transformerHot) != 0) {
                heaterText  = "held: transformer hot";
                heaterClass = "warn";
            }
            if((s->flags & Protocol::Flag::tipRemoved) != 0) {
                tipPosText  = "removed from the handle";
                tipPosClass = "warn";
            } else if((s->flags & Protocol::Flag::tipInSleep) != 0) {
                tipPosText  = "in the holder";
                tipPosClass = "good";
            } else {
                tipPosText  = "in hand";
                tipPosClass = "";
            }
        }
    }

    static constexpr std::size_t DutyRows = 3;

    /// A chart's rows as one flat run list (RTXUI's loop items carry strings only), "\n" ending a row.
    static void flatten(Tui::Chart const&     plot,
                        std::vector<RunView>& out) {
        out.clear();
        for(std::size_t r = 0; r < plot.rows.size(); ++r) {
            out.push_back(RunView{plot.rows[r].axis, "axis"});
            for(auto const& run : plot.rows[r].runs) {
                out.push_back(RunView{run.text, run.style});
            }
            if(r + 1 != plot.rows.size()) { out.back().text += "\n"; }
        }
    }

    void rebuildChart() {
        auto const [samples, label] = Windows[window_];
        windowText                  = label;
        // the graph card's inside: the terminal less the right column (60), the body's padding (2), the
        // gap (1), the card's border and padding (4); the height what the header (2), the iron card (9: borders, six rows, its duty bar),
        // the graph card's border, its two heads, the duty rows and the footer (1) leave
        constexpr std::size_t Fixed = 2 + 9 + 2 + 2 + DutyRows + 1;
        auto const            width = std::max<std::size_t>(30, columns_ > 67 ? columns_ - 67 : 30);
        auto const            height = std::max<std::size_t>(6, rows_ > Fixed ? rows_ - Fixed : 0);
        flatten(Tui::temperatureChart(model_.history(), samples, width, height), chart);
        flatten(Tui::dutyChart(model_.history(), samples, width, DutyRows), duty);
    }

    void rebuildTunables() {
        auto const& v = model_.tunables();
        tunables.clear();
        for(std::size_t i = 0; i < Remote::Count; ++i) {
            auto const& info  = Remote::Infos[i];
            auto const  def   = Tui::formatFixed(info.fallback, info.decimals);
            auto const  value = v ? Tui::formatTunable(i, (*v)[i]) : std::string{"-"};
            tunables.push_back(TuneRow{
              .name  = std::string{i == model_.selected() ? "▶ " : "  "} + std::string{info.name},
              .value = value + (model_.unsaved(i) ? " ●" : ""),
              .range = Tui::formatFixed(info.min, info.decimals) + "-"
                     + Tui::formatFixed(info.max, info.decimals) + " (" + def + ")",
              .style = i == model_.selected() ? "selected"
                     : v                      ? ""
                                              : "dim"});
        }
        auto const& info = Remote::Infos[model_.selected()];
        help             = std::string{info.help};
        if(model_.unsaved(model_.selected())) {
            help += " - stored: "
                  + Tui::formatTunable(model_.selected(), (*model_.stored())[model_.selected()]);
        }
        auto const unsaved = model_.unsavedCount();
        storeText          = !model_.stored() ? ""
                           : unsaved == 0 ? "✓ all stored"
                                          : "● " + std::to_string(unsaved) + " not stored - s stores";
        storeClass         = unsaved == 0 ? "good" : "warn";
        if(entering_) {
            entryText
              = "new " + std::string{info.name} + ": " + entry_ + "▏  (Enter sends, Esc cancels)";
            entryClass = model_.entryError().empty() ? "entry" : "entry-bad";
            if(!model_.entryError().empty()) { entryText += "  " + model_.entryError(); }
        } else {
            entryText.clear();
            entryClass = "hidden";
        }
    }

    void rebuildDiagnostics() {
        diagnostics.clear();
        auto const& d   = model_.diagnostics();
        auto const& t   = model_.telemetry();
        auto const& s   = model_.status();
        auto const  add = [&](std::string k, std::string v, std::string style = "") {
            diagnostics.push_back(KeyValue{std::move(k), std::move(v), std::move(style)});
        };
        if(!d) {
            add("-", "no diagnostics yet");
            return;
        }
        auto const halfUs = (d->mainsHalfPeriodUs[0] + d->mainsHalfPeriodUs[1]) / 2.0;
        char       mains[64];
        std::snprintf(mains,
                      sizeof(mains),
                      "%.2f Hz  (%u / %u us)",
                      halfUs > 0 ? 1e6 / (2 * halfUs) : 0.0,
                      unsigned{d->mainsHalfPeriodUs[0]},
                      unsigned{d->mainsHalfPeriodUs[1]});
        add("mains", mains, halfUs < 8'000 || halfUs > 11'000 ? "warn" : "");
        if(d->transformerDeciC == Protocol::Diagnostics::Unknown) {
            add("transformer", "no reading", "warn");
        } else {
            add("transformer",
                Tui::deciC(d->transformerDeciC) + " °C",
                d->transformerDeciC >= 500 ? "bad" : "");
        }
        if(t) {
            add("ADC tip/current",
                std::to_string(t->rawTemp) + " / " + std::to_string(t->rawCurrent));
        }
        if(s) { add("ADC cycle", std::string{Tui::name(s->adc)}); }
        add("ADC conversions", std::to_string(d->adcConversions));
        add("reset cause",
            std::string{Tui::name(d->resetCause)},
            d->resetCause == Protocol::ResetCause::wdt
                || d->resetCause == Protocol::ResetCause::bod33
              ? "warn"
              : "");
        add("fuse bits",
            d->fusesOk ? "ok" : "NOT WRITTEN (retried at boot)",
            d->fusesOk ? "good" : "bad");
        add("EEPROM temp",
            std::to_string(d->eepromC) + " °C" + (d->eepromPending ? "  store pending" : ""),
            d->eepromPending ? "warn" : "");
        add("longest turn",
            std::to_string(d->maxTurnUs) + " us",
            d->maxTurnUs > 500'000 ? "warn" : "");
        auto const counters = [](Protocol::LinkCounters const& c) {
            return std::to_string(c.framesIn) + " in  " + std::to_string(c.framesOut) + " out  "
                 + std::to_string(c.badBytes) + " bad  " + std::to_string(c.dropped) + " dropped";
        };
        add("probe link", counters(d->probe), d->probe.badBytes != 0 ? "warn" : "");
        add("UART link", counters(d->uart), d->uart.badBytes != 0 ? "warn" : "");
        add("reports here",
            std::to_string(model_.reports()) + "  resets seen " + std::to_string(model_.resets()));
    }

public:
    /// The terminal's size, for the graph (Event::Resized).
    std::size_t columns_{140};
    std::size_t rows_{40};

    /// Reports and link events straight into the model, without a link (--render).
    void feed(Protocol::Report const& r,
              Tui::TimePoint          now) {
        model_.apply(r, now);
    }

    void fedLink(std::string const& path,
                 Tui::TimePoint     now) {
        model_.linkUp(path, now);
    }

    void redraw() { rebuild(); }

private:
    Tui::Model                  model_;
    std::unique_ptr<Host::Link> link_;
    rtxui::Screen*              screen_{};
    bool                        entering_{};
    std::string                 entry_;
    /// A two-step action waiting for its second press (key or button); any other key disarms it.
    enum class Armed { none, defaults, factory };
    Armed       armed_{Armed::none};
    std::size_t window_{1};
};

/// `--dump [path] [baud=N] [until=REGEX ...] [defaults] [set=INDEX:VALUE ...] [store] [for=SECONDS]`
int dump(int    argc,
         char** argv) {
    std::string                                        path;
    unsigned                                           baud = 115'200;
    std::vector<std::regex>                            untils;
    std::vector<std::pair<std::uint8_t, std::int32_t>> sets;
    std::optional<std::chrono::seconds>                limit;
    bool                                               store    = false;
    bool                                               defaults = false;
    for(int i = 2; i < argc; ++i) {
        std::string_view const arg{argv[i]};
        if(arg == "store") {
            store = true;
        } else if(arg == "defaults") {
            defaults = true;
        } else if(arg.starts_with("until=")) {
            untils.emplace_back(std::string{arg.substr(6)});
        } else if(arg.starts_with("set=")) {
            auto const colon = arg.find(':');
            if(colon == std::string_view::npos) {
                std::fprintf(stderr, "set=INDEX:VALUE, not '%s'\n", argv[i]);
                return 2;
            }
            sets.emplace_back(
              static_cast<std::uint8_t>(std::atoi(std::string{arg.substr(4, colon - 4)}.c_str())),
              static_cast<std::int32_t>(std::atol(std::string{arg.substr(colon + 1)}.c_str())));
        } else if(arg.starts_with("for=")) {
            limit = std::chrono::seconds{std::atoi(std::string{arg.substr(4)}.c_str())};
        } else if(arg.starts_with("baud=")) {
            baud = static_cast<unsigned>(std::atoi(std::string{arg.substr(5)}.c_str()));
        } else if(path.empty()) {
            path = arg;
        } else {
            std::fprintf(stderr, "unknown: %s\n", argv[i]);
            return 2;
        }
    }

    std::mutex                   mutex;
    std::condition_variable      changed;
    std::size_t                  matched      = 0;
    bool                         haveTunables = false;
    bool                         up           = false;
    std::optional<std::uint32_t> lastUptime;
    Host::Link                   link{path,
                                      rttLogDir(),
                                      baud,
                                      [&](Protocol::Report const& r) {
                        std::lock_guard const lock{mutex};
                        auto                  line = Tui::Model::dumpLine(r);
                        // the Model's reset rule: the uptime went back
                        if(auto const* s = std::get_if<Protocol::Status>(&r)) {
                            if(lastUptime && s->uptimeMs < *lastUptime) {
                                line = "station reset\n" + line;
                            }
                            lastUptime = s->uptimeMs;
                        }
                        std::printf("%s\n", line.c_str());
                        std::fflush(stdout);
                        if(std::holds_alternative<Protocol::Tunables>(r)) { haveTunables = true; }
                        if(matched < untils.size() && std::regex_search(line, untils[matched])) {
                            ++matched;
                        }
                        changed.notify_all();
                                      },
                                      [&](Host::LinkEvent e, std::string const& p) {
                        std::lock_guard const lock{mutex};
                        std::printf("link %s %s\n",
                                    e == Host::LinkEvent::up        ? "up"
                                    : e == Host::LinkEvent::missing ? "missing"
                                                                    : "refused",
                                    p.c_str());
                        std::fflush(stdout);
                        up = e == Host::LinkEvent::up;
                        changed.notify_all();
                                      }};

    std::unique_lock lock{mutex};
    auto const deadline = std::chrono::steady_clock::now() + limit.value_or(std::chrono::hours{24});
    // the link must come up (no limit: two seconds for it)
    if(!changed.wait_until(lock,
                           limit ? deadline : std::chrono::steady_clock::now() + 2s,
                           [&] { return up; }))
    {
        std::fprintf(stderr, "no link - is `just log` running, or the path right?\n");
        return 1;
    }
    if(!sets.empty() || store || defaults) {
        changed.wait_until(lock, deadline, [&] { return haveTunables; });
        std::uint16_t seq = 100;
        lock.unlock();
        if(defaults) { link.send(Protocol::ResetTunables{seq++}); }
        for(auto const& [index, value] : sets) {
            link.send(Protocol::SetTunable{.seq = seq++, .index = index, .value = value});
        }
        if(store) { link.send(Protocol::StoreTunables{seq}); }
        lock.lock();
    }
    if(untils.empty()) {
        changed.wait_until(lock, limit ? deadline : std::chrono::steady_clock::now() + 2s, [] {
            return false;
        });
        return 0;
    }
    return changed.wait_until(lock, deadline, [&] { return matched == untils.size(); }) ? 0 : 3;
}

/// `--render [path] [WxH] [until=REGEX] [for=SECONDS]`: the screen once, as text, after the reports
/// up to the first line matching `until` (Model::dumpLine; default: the third Tunables-or-later
/// report after a Diagnostics) or `for` seconds - for the end-to-end test and for a look without a
/// terminal. Exit 1 without a link, 3 when `until` did not match in time.
int render(int    argc,
           char** argv) {
    std::string               path;
    int                       width  = 150;
    int                       height = 44;
    std::optional<std::regex> until;
    std::chrono::seconds      limit{10};
    for(int i = 2; i < argc; ++i) {
        std::string_view const arg{argv[i]};
        if(arg.starts_with("until=")) {
            until.emplace(std::string{arg.substr(6)});
        } else if(arg.starts_with("for=")) {
            limit = std::chrono::seconds{std::atoi(std::string{arg.substr(4)}.c_str())};
        } else if(auto const x = arg.find('x');
                  x != std::string_view::npos && !arg.starts_with("/")
                  && arg.find_first_not_of("0123456789x") == std::string_view::npos)
        {
            width  = std::atoi(std::string{arg.substr(0, x)}.c_str());
            height = std::atoi(std::string{arg.substr(x + 1)}.c_str());
        } else {
            path = arg;
        }
    }
    std::mutex                    mutex;
    std::condition_variable       changed;
    std::vector<Protocol::Report> reports;
    std::string                   linkedPath;
    bool                          done = false;
    Host::Link                    link{path,
                                       rttLogDir(),
                                       115'200,
                                       [&](Protocol::Report const& r) {
                        std::lock_guard const lock{mutex};
                        if(done) { return; }
                        reports.push_back(r);
                        if(until && std::regex_search(Tui::Model::dumpLine(r), *until)) {
                            done = true;
                        }
                        changed.notify_all();
                                       },
                                       [&](Host::LinkEvent e, std::string const& p) {
                        std::lock_guard const lock{mutex};
                        if(e == Host::LinkEvent::up) { linkedPath = p; }
                        changed.notify_all();
                                       }};
    std::unique_lock              lock{mutex};
    auto const                    matched
      = changed.wait_for(lock, limit, [&] { return done || (!until && reports.size() >= 80); });
    if(linkedPath.empty()) {
        std::fprintf(stderr, "no link - is `just log` running, or the path right?\n");
        return 1;
    }
    auto app       = rtxui::Ref<StationApp>::New();
    app->columns_  = static_cast<std::size_t>(width);
    app->rows_     = static_cast<std::size_t>(height);
    auto const now = Tui::Clock::now();
    app->fedLink(linkedPath, now);
    for(auto const& r : reports) { app->feed(r, now); }
    app->redraw();
    std::printf("%s\n", rtxui::RenderToString(app, width, height).c_str());
    return until && !matched ? 3 : 0;
}

}   // namespace

int main(int    argc,
         char** argv) {
    if(argc > 1 && std::string_view{argv[1]} == "--dump") { return dump(argc, argv); }
    if(argc > 1 && std::string_view{argv[1]} == "--render") { return render(argc, argv); }

    std::string path;
    unsigned    baud = 115'200;
    for(int i = 1; i < argc; ++i) {
        std::string_view const arg{argv[i]};
        if(arg.starts_with("baud=")) {
            baud = static_cast<unsigned>(std::atoi(std::string{arg.substr(5)}.c_str()));
        } else {
            path = arg;
        }
    }
    if(!Host::baudConstant(baud)) {
        std::fprintf(stderr, "baud=%u: not a rate this host's termios knows\n", baud);
        return 2;
    }

    auto app = rtxui::Ref<StationApp>::New();
    // the terminal's size now: the graph is cut to it, and a resize event need not come at the start
    if(winsize ws{}; ::ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col != 0) {
        app->columns_ = ws.ws_col;
        app->rows_    = ws.ws_row;
    }
    rtxui::Screen screen(app);
    screen.SetInline(false);
    app->connect(path, baud, &screen);
    screen.Loop();
    return 0;
}
