# Soldering station specifications

## Safety features
- turn off:
  - station is on longer than 1 hour
  - if transformer temperature is > 50°C. Transformer external temp max: 40°C
  - station is not handled for 5 minutes. Handling events:
    - turning knob
    - pressing knob
    - toggling sleep/remove signals
  - watchdog triggered
  - instant when remove signal
  - after 0...1-minute when sleep signal

## Settings
- Soldering temperature
- Sleep time
## Remote (`tui/`)
A terminal remote, `jbc_tui` (RTXUI): the tip against its setpoint as a graph, the heater's duty, the state, the 1 h
latch countdown, diagnostics (mains frequency, transformer temperature, reset cause, EEPROM, link counters) and the
tunables (solder and sleep temperature, PID gains, standby and sleep times). A change is used at once but kept in
RAM; `s` stores them all in flash, which the station starts with after a reset (the solder temperature is stored at
once, as with the knob). `●` marks a value that is not stored yet. A bad set in flash: `F F` (or the
"Defaults → flash" button) puts the defaults back and stores them (the solder temperature stays).
It speaks `src/remote/Protocol.hpp` (aglio frames) over two links at once:
- the J-Link: `just log debug` in one terminal, `just tui` in another (the RTT duplex channel "control");
- SERCOM1's UART on the debug pins, TX PA18 / RX PA19, 115200 8N1: `just tui /dev/ttyUSB0`.

Without a station: `just fake` and `just tui /tmp/jbc_fake.sock`. Tests: `just test`, `just tui-test`.
