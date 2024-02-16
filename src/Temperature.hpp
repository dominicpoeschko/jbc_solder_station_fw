//
// Created by patrick on 1/5/22.
//

#include "HWConfig.hpp"
#include "pid.hpp"

#include <atomic>

#pragma once
template<typename Clock, typename Float_t, typename PidConfig, typename ZCD, typename Evsys>
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

    State                 s{State::init};
    Pid<Clock, PidConfig> pid;
    bool                  heaterOn{false};
    bool tip1Sleep{false};
    bool tip1Remove{false};

    Float_t setPoint{};

    void setTemp(Float_t v) { setPoint = v; }

    using heaterPin = HW::Pin::tip1Sw;

    using regs = Kvasir::Peripheral::ADC::Registers<>;

    std::atomic<bool> newTemp{false};
    void              adcCallback() {
        std::uint16_t val = apply(read(regs::RESULT::result));


        if(s == State::wait_temp) {
            Float_t tempVal = val * adcToTempFactor;
            //lastTempVal     = tempVal;
            lastTempVal = (tempVal + lastTempRawVal) / Float_t(2.0);
            newTemp.store(true, std::memory_order::relaxed);
            lastTempRawVal = tempVal;
            s              = State::wait_current_start;

            if(onCyles + offCyles == 0) {
                auto heatval = pid.update(typename PidConfig::type{setPoint}, lastTempVal);

                onCyles = static_cast<std::uint16_t>(std::ceil(heatval - Float_t(0.2)));
                if(onCyles != 0) {
                    offCyles = 10 - onCyles;
                    if(onCyles == 10) {
                        offCyles = 0;
                    }

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


           UC_LOG_T("{},{},{},{}", lastTempVal, setPoint, lastCurrentVal, heaterOn?50000:0);
        } else if(s == State::wait_current) {
            lastCurrentVal = val;
            if(heaterOn) {
                UC_LOG_T("current {}", lastCurrentVal);
            }
            s = State::wait_heater_turn_off;
            apply(write(regs::INPUTCTRL::MUXPOSValC::pin2));
            measureAndDisableTipPosition();
        }
    }

    void handler() {
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

    void measureAndDisableTipPosition(){
        auto const signals = apply(
                read(
                        HW::Pin::tip1Sleep{},
                        HW::Pin::tip1Remove{}
                ));
        tip1Sleep = !(get<0>(signals));
        tip1Remove = !(get<1>(signals));
        apply(clear(HW::Pin::sleepSw{}));
    }

    void enableTipPositionMeasurement(){
        apply(set(HW::Pin::sleepSw{}));
    }

    void adcinit() {
        using Kvasir::Register::value;
        apply(Kvasir::PM::enable<regs::baseAddr>::action{});

        apply(
          regs::REFCTRL::overrideDefaults(write(regs::REFCTRL::REFSELValC::arefb)),
          regs::AVGCTRL::overrideDefaults(
            write(regs::AVGCTRL::SAMPLENUMValC::_16),
            write(regs::AVGCTRL::adjres, value<std::uint8_t,0>())),
          regs::SAMPCTRL::overrideDefaults(
            write(regs::SAMPCTRL::samplen, value<std::uint8_t, 4>())),
          regs::CTRLB::overrideDefaults(
            write(regs::CTRLB::PRESCALERValC::div8),
            write(regs::CTRLB::RESSELValC::_16bit)),
          regs::INPUTCTRL::overrideDefaults(
            write(regs::INPUTCTRL::MUXPOSValC::pin2),
            write(regs::INPUTCTRL::MUXNEGValC::gnd)),
          regs::EVCTRL::overrideDefaults(set(regs::EVCTRL::startei)),
          set(regs::INTENSET::resrdy));

        std::array<std::byte, 8> const NVM_Calibration_Data = []() {
            static constexpr std::uint32_t     NVM_Calibration_Addr = 0x00806020;
            std::array<std::uint32_t, 2> const bits{
              *reinterpret_cast<std::uint32_t const volatile*>(NVM_Calibration_Addr),
              *reinterpret_cast<std::uint32_t const volatile*>(NVM_Calibration_Addr + 4)};
            return *reinterpret_cast<std::array<std::byte, 8> const*>(bits.data());
        }();

        Kvasir::BitField<34, 27> const Adc_LinearityCal(NVM_Calibration_Data);
        Kvasir::BitField<37, 35> const Adc_BiasCal(NVM_Calibration_Data);
        apply(
          write(regs::CALIB::linearity_cal, Adc_LinearityCal.asValue()),
          write(regs::CALIB::bias_cal, Adc_BiasCal.asValue()));
        apply(Evsys::template setupEventChannel<
              Evsys::Channel::ch0,
              Evsys::Generator::eic_extint12,
              Evsys::User::adc_start,
              Evsys::Path::asynchronous,
              Evsys::Edge::no_evt_output>());
        apply(regs::CTRLA::overrideDefaults(set(regs::CTRLA::enable)));
        apply(makeEnable(Kvasir::Interrupt::adc));
    }

    Float_t temp() const { return lastTempVal; }
};
