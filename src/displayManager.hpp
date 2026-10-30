//
// Created by patrick on 1/25/22.
//
#pragma once

#include <algorithm>
#include <cstdint>
#include <kvasir/Devices/I2C/Adapters/SegmentBackends.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Pca9956b.hpp>

/// The three digits on the PCA9956B: the preset temperature, steady or blinking while it is
/// being changed. The glyphs, the wiring and the wire traffic are Kvasir::I2C::Pca9956bDisplay's.
template<typename I2C, typename Clock>
struct DisplayManager {
    /// 0x3F with a 2.2 k external resistor, as the board is built.
    using Leds   = Kvasir::I2C::Chips::Pca9956b<0x3F, Kvasir::Units::ohm(2200)>;
    using Device = Kvasir::I2C::Device<I2C, Clock, Leds>;

    std::uint32_t displayValue{};

    /// The device lives on the application's Bus with the other part of the board (one engine
    /// for both); this only drives the digits on it.
    Device&                              display;
    Kvasir::I2C::Pca9956bDisplay<Device> digits{display, Kvasir::Units::milliAmp(5)};

    explicit DisplayManager(Device& device) : display{device} {}

    DisplayManager(DisplayManager const&)            = delete;   // digits points at display
    DisplayManager& operator=(DisplayManager const&) = delete;

    void setBlinkMode() { digits.blink(Kvasir::SegmentDisplay::Blink::on); }

    void setStaticMode() { digits.blink(Kvasir::SegmentDisplay::Blink::off); }

    void turn() {
        digits.setNumber(static_cast<std::int32_t>(std::min(displayValue, std::uint32_t{999})));
        digits.update(Clock::now());
    }
};
