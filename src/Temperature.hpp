//
// Created by patrick on 1/5/22.
//

#include "HWConfig.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <kvasir/Atomic/CriticalSection.hpp>
#include <kvasir/Control/Pid.hpp>

#pragma once

template<typename Clock, typename Float_t, auto const& Gains, typename ZCD, typename Evsys>
struct Temp {
    enum class State : uint8_t {
        init,
        wait_temp,
        wait_current_start,
        wait_current,
        wait_heater_turn_off
    };
    static constexpr Float_t adcToTempFactor = Float_t(0.01115);

    std::uint16_t onCyles  = 0;
    std::uint16_t offCyles = 0;

    Float_t lastTempVal{};
    Float_t lastCurrentVal{};

    Float_t lastTempRawVal{};
    // the two-sample mean starts at the first sample: from 0 it showed the PID half the iron's
    // temperature on the first update (up to a full burst of heat on a warm restart)
    bool haveTempRawVal{false};

    State                         s{State::init};
    Kvasir::Control::Pid<Float_t> pid{Gains};
    typename Clock::time_point    lastPidAt{};
    bool                          heaterOn{false};
    bool                          tip1Sleep{false};
    bool                          tip1Remove{false};

    Float_t setPoint{};

    void setTemp(Float_t v) { setPoint = v; }

    /// What the tip conversions since the last take() came to (remote/Remote.hpp's Telemetry).
    struct Window {
        Float_t       sum{};
        Float_t       min{};
        Float_t       max{};
        std::uint8_t  samples{};
        std::uint8_t  heated{};
        std::uint16_t rawTemp{};
        std::uint16_t rawCurrent{};
        Float_t       pidOut{};
        Float_t       setPoint{};
    };

    Window window{};
    /// ADC interrupts since boot.
    std::uint32_t conversions{};

    /// The window so far, and a new one started; the ADC interrupt fills it.
    Window take() {
        return Kvasir::criticalSection([&] {
            auto w     = window;
            w.pidOut   = pid.output();
            w.setPoint = setPoint;
            window     = Window{};
            return w;
        });
    }

    /// New gains (remote/Remote.hpp), taken at the next conversion; the loop starts again from P only,
    /// as after a reset, so no stale integral or derivative of the old gains drives the heater.
    void setGains(Kvasir::Control::PidGains<Float_t> const& g) {
        Kvasir::criticalSection([&] {
            pid.setGains(g);
            pid.reset();
        });
    }

    using heaterPin = HW::Pin::tip1Sw;

    using regs = Kvasir::Peripheral::ADC::Registers<>;

    std::atomic<bool> newTemp{false};

    void adcCallback() {
        std::uint16_t val = apply(read(regs::RESULT::result));
        ++conversions;

        if(s == State::wait_temp) {
            Float_t tempVal = static_cast<Float_t>(val) * adcToTempFactor;
            //lastTempVal     = tempVal;
            Float_t const previous = haveTempRawVal ? lastTempRawVal : tempVal;
            lastTempVal            = (tempVal + previous) / Float_t(2.0);
            haveTempRawVal         = true;
            newTemp.store(true, std::memory_order::relaxed);
            lastTempRawVal = tempVal;
            s              = State::wait_current_start;

            if(onCyles + offCyles == 0) {
                auto const now     = Clock::now();
                auto const heatval = pid.update(setPoint, lastTempVal, now - lastPidAt);
                lastPidAt          = now;

                onCyles = static_cast<std::uint16_t>(std::ceil(heatval - Float_t(0.2)));
                if(onCyles != 0) {
                    offCyles = 10 - onCyles;
                    if(onCyles == 10) { offCyles = 0; }

                    --onCyles;
                    apply(HW::heat<true>(), set(heaterPin{}));
                    heaterOn = true;
                }
            } else {
                if(onCyles == 0) {
                    --offCyles;
                } else {
                    --onCyles;
                    apply(HW::heat<true>(), set(heaterPin{}));
                    heaterOn = true;
                }
            }

            window.rawTemp = val;
            if(window.samples == 0) {
                window.min = lastTempVal;
                window.max = lastTempVal;
            }
            if(window.samples != 0xFF) {   // nobody took it for 2.5 s: the window stops growing
                window.min = std::min(window.min, lastTempVal);
                window.max = std::max(window.max, lastTempVal);
                window.sum += lastTempVal;
                ++window.samples;
                if(heaterOn) { ++window.heated; }
            }
        } else if(s == State::wait_current) {
            lastCurrentVal    = val;
            window.rawCurrent = val;
            s                 = State::wait_heater_turn_off;
            apply(write(regs::INPUTCTRL::MUXPOSValC::pin2));
            measureAndDisableTipPosition();
        }
    }

    void turn() {
        using namespace std::chrono_literals;
        auto const currentTime = Clock::now();

        switch(s) {
        case State::init:
            {
                adcinit();
                s = State::wait_temp;
            }
            break;

        case State::wait_temp:
            {
            }
            break;

        case State::wait_current_start:
            {
                if(currentTime > ZCD::nextPeak.load(std::memory_order_relaxed) - 5us) {
                    s = State::wait_current;
                    apply(write(regs::INPUTCTRL::MUXPOSValC::pin3));
                    apply(regs::SWTRIG::overrideDefaults(set(regs::SWTRIG::start)));
                    enableTipPositionMeasurement();
                }
            }
            break;

        case State::wait_current:
            {
                //TODO timeout
            }
            break;

        case State::wait_heater_turn_off:
            {
                if(currentTime > ZCD::nextCrossing.load(std::memory_order_relaxed) - 600us) {
                    s = State::wait_temp;

                    //KL_D("Sleep {}  Remove {}", tip1Sleep, tip1Remove);
                    //TODO feed the dog
                    apply(HW::heat<false>(), set(heaterPin{}));
                    heaterOn = false;
                }
            }
            break;
        }
    }

    void measureAndDisableTipPosition() {
        auto const signals = apply(read(HW::Pin::tip1Sleep{}, HW::Pin::tip1Remove{}));
        tip1Sleep          = !(get<0>(signals));
        tip1Remove         = !(get<1>(signals));
        apply(clear(HW::Pin::sleepSw{}));
    }

    void enableTipPositionMeasurement() { apply(set(HW::Pin::sleepSw{})); }

    void adcinit() {
        using Kvasir::Register::value;
        apply(Kvasir::PM::Enable<regs::baseAddr>::action{});

        apply(
          regs::REFCTRL::overrideDefaults(write(regs::REFCTRL::REFSELValC::arefb)),
          regs::AVGCTRL::overrideDefaults(write(regs::AVGCTRL::SAMPLENUMValC::_16),
                                          write(regs::AVGCTRL::adjres, value<std::uint8_t, 0>())),
          regs::SAMPCTRL::overrideDefaults(
            write(regs::SAMPCTRL::samplen, value<std::uint8_t, 4>())),
          regs::CTRLB::overrideDefaults(write(regs::CTRLB::PRESCALERValC::div8),
                                        write(regs::CTRLB::RESSELValC::_16bit)),
          regs::INPUTCTRL::overrideDefaults(write(regs::INPUTCTRL::MUXPOSValC::pin2),
                                            write(regs::INPUTCTRL::MUXNEGValC::gnd)),
          regs::EVCTRL::overrideDefaults(set(regs::EVCTRL::startei)),
          set(regs::INTENSET::resrdy));

        // ADC LINEARITY and BIASCAL from the NVM software calibration area (SAM D21 DS40001882L 10.3.2, Table 10-8)
        auto const cal = Kvasir::Nvm::calibration();
        apply(write(regs::CALIB::linearity_cal, cal.adcLinearity),
              write(regs::CALIB::bias_cal, cal.adcBiasCal));
        apply(Evsys::template setupEventChannel<Evsys::Channel::ch0,
                                                Evsys::Generator::eic_extint12,
                                                Evsys::User::adc_start,
                                                Evsys::Path::asynchronous,
                                                Evsys::Edge::no_evt_output>());
        apply(regs::CTRLA::overrideDefaults(set(regs::CTRLA::enable)));
        apply(makeEnable(Kvasir::Interrupt::adc));
    }

    Float_t temp() const { return lastTempVal; }
};
