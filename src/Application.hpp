//
// Created by patrick on 1/28/22.
//

#pragma once

#include "ApplicationConfig.hpp"
//

#include "TimingConfig.hpp"
#include "UnBounce.hpp"
//#include "LedgedHysteresis.hpp"
#include "displayManager.hpp"
#include "kvasir/Devices/Ktd2026.hpp"
#include "kvasir/Devices/Max31820.hpp"

template<typename Clock>
struct Application {
    using time_point_t = typename Clock::time_point;

    Kvasir::Max31820<TempOneWire, Clock> tempSensTransformer;
    DisplayManager<I2C, Clock>           display;
    Kvasir::Ktd2026<I2C, Clock>          ledDriver;
    Temp_t                               heaterManager;

    //TODO change rotary type to right data type
    RotaryEncoder::type solderTemp_;
    RotaryEncoder::type sleepTemp_{210};
    RotaryEncoder::type preSetTemp_;
    RotaryEncoder::type oldPreSetTemp_;

    std::uint32_t actionCounter_{1};
    std::uint32_t menuTimeOutCounter_{1};

    time_point_t next{Clock::now()};
    time_point_t menuTime{Clock::now() + timingConfig::menuTimeOut};
    time_point_t failTime{Clock::now() + timingConfig::failTimeOut};

    struct Init {};

    struct Sleep {
        explicit Sleep(time_point_t t_sleep, time_point_t t_standby)
          : timeOutSleep{t_sleep}
          , timeOutStandby{t_standby} {}
        time_point_t timeOutSleep{};
        time_point_t timeOutStandby{};
    };

    struct Solder {
        explicit Solder(time_point_t t) : timeOutStandby{t} {}
        time_point_t timeOutStandby{};
    };

    struct Remove {};

    struct Standby {};

    struct Shutdown {};

    using stateMachine = std::variant<Init, Sleep, Solder, Remove, Standby, Shutdown>;
    stateMachine st_{Init{}};

    Application() {
        solderTemp_ = RotaryEncoder::type{TempEeeprom::value().temp};
        heaterManager.setTemp(solderTemp_);
        AdcIsr::temp = &heaterManager;
        RotaryEncoder::cnt.store(solderTemp_, std::memory_order_relaxed);
    }
    bool tip1oldSleep{false};
    bool tip1oldRemoved{false};


    template<class... Ts>
    struct overloaded : Ts... {
        using Ts::operator()...;
    };
    template<class... Ts>
    overloaded(Ts...)->overloaded<Ts...>;

    template<typename Variant, typename... Matchers>
    auto match(Variant&& variant, Matchers&&... matchers) {
        return std::visit(overloaded{std::forward<Matchers>(matchers)...}, std::forward<Variant>(variant));
    }


    void handler() {
        auto const currentTime{Clock::now()};
        //TODO
        bool tip1Sleep {heaterManager.tip1Sleep};
        bool tip1Removed{heaterManager.tip1Remove};
        std::uint8_t handlingEvent{0};
        if(tip1Sleep != tip1oldSleep){
            ++handlingEvent;
        }
        if(tip1Removed != tip1oldRemoved){
            ++handlingEvent;
        }
        tip1oldRemoved = tip1Removed;
        tip1oldSleep = tip1Sleep;

        preSetTemp_ = RotaryEncoder::cnt.load(std::memory_order_relaxed);
        //display.setText("1223", std::nullopt);
        Button::handler([&](auto e, [[maybe_unused]] auto const& t) {
            switch(e) {
            case Button::Event::Type::Release_Short:
                {
                    //        modeChange           = true;
                    //        rotaryMode           = false;
                    solderTemp_          = preSetTemp_;
                    TempEeeprom::value() = TempSaveType{solderTemp_};
                    TempEeeprom::writeValue();

                    //display.setStaticMode();
                    ++handlingEvent;
                }
                break;
            //TODO change button type
            case Button::Event::Type::Hit: break;
            case Button::Event::Type::Long: break;
            case Button::Event::Type::Release_Long: break;
            }
        });

        if(currentTime > next) {
            //KL_D("remove: {} sleep: {}", tip1Removed.getValue(), tip1Sleep.getValue());
            //KL_D("{}", st_.index());
            next += 10ms;
        }

        if(currentTime > failTime) {
            st_ = Shutdown{};
        }

        st_ = match(
          st_,
          [&]([[maybe_unused]] Init const& state) -> stateMachine {
              //Do init steps
              return Sleep{
                currentTime + timingConfig::sleepTimeOut,
                currentTime + timingConfig::standbyTimeOut};
          },
          [&](Sleep const& state) -> stateMachine {
              if(currentTime > state.timeOutSleep) {
                  ledDriver.set(0, 0, 3);
                  heaterManager.setTemp(sleepTemp_);
              } else {
                  ledDriver.set(0, 3, 0);
              }

              if(currentTime > state.timeOutStandby) {
                  return Standby{};
              }
              if(tip1Sleep) {
                  return state;
              }
              return Solder{currentTime + timingConfig::standbyTimeOut};
          },
          [&](Solder const& state) -> stateMachine {
              ledDriver.set(0, 0, 0);
              heaterManager.setTemp(solderTemp_);
              if(tip1Sleep) {
                  return Sleep{
                    currentTime + timingConfig::sleepTimeOut,
                    currentTime + timingConfig::standbyTimeOut};
              }
              if(tip1Removed) {
                  return Remove{};
              }
              if(currentTime > state.timeOutStandby) {
                  return Standby{};
              }
              return state;
          },
          [&](Remove const& state) -> stateMachine {
              heaterManager.setTemp(0);
              ledDriver.set(1, 0, 2);
              if(tip1Removed) {
                  return state;
              }
              return Sleep{
                currentTime + timingConfig::sleepTimeOut,
                currentTime + timingConfig::standbyTimeOut};
          },
          [&](Standby const& state) -> stateMachine {
              ledDriver.set(1, 1, 0);
              heaterManager.setTemp(0);
              if(handlingEvent > 0) {
                  handlingEvent = 0;
                  return Sleep{
                    currentTime + timingConfig::sleepTimeOut,
                    currentTime + timingConfig::standbyTimeOut};
              }
              return state;
          },
          [&](Shutdown const& state) -> stateMachine {
              heaterManager.setTemp(0);
              ledDriver.set(10, 0, 0);
              //To leave this state you must reboot the system
              return state;
          });

        if(heaterManager.heaterOn) {
            ledDriver.set(1, 0, 0);
        } else {
        }

        if(preSetTemp_ != solderTemp_) {
            ++actionCounter_;
            display.setBlinkMode();
        } else {
            display.setStaticMode();
        }
        display.displayValue = preSetTemp_;

        tempSensTransformer.handler();
        display.handler();
        ledDriver.handler();
        heaterManager.handler();
        TempOneWire::handler();
    }

    void fault(){
        //Display ERR
        ledDriver.set(10,0,0);
        ledDriver.handler();
        display.handler();
    }
};
