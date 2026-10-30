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
#include "kvasir/Devices/Max31820.hpp"

#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Ktd2026.hpp>

template<typename Clock>
struct Application {
    using time_point_t = typename Clock::time_point;

    Kvasir::Max31820<TempOneWire, Clock> tempSensTransformer;
    using Rgb    = Kvasir::I2C::Chips::Ktd2026<>;
    using RgbDev = Kvasir::I2C::Device<I2C, Clock, Rgb>;
    using Digits = typename DisplayManager<I2C, Clock>::Device;
    /// Both parts of the board on one Bus: one device set, one engine (kvasir_devices SetOps.hpp).
    Kvasir::I2C::Bus<I2C, Clock, Digits, RgbDev> parts{};
    DisplayManager<I2C, Clock>                   display{parts.template get<Digits>()};
    RgbDev&                                      ledDriver = parts.template get<RgbDev>();
    Temp_t                                       heaterManager;

    //TODO change rotary type to right data type
    RotaryEncoder::type solderTemp_;
    RotaryEncoder::type sleepTemp_{210};
    RotaryEncoder::type preSetTemp_;
    RotaryEncoder::type oldPreSetTemp_;

    std::uint32_t actionCounter_{1};
    std::uint32_t menuTimeOutCounter_{1};

    /// Remote-tunable (remote/Tunables.hpp), RAM only: TimingConfig.hpp's values after every boot.
    typename Clock::duration standbyTimeOut_{timingConfig::standbyTimeOut};
    typename Clock::duration sleepTimeOut_{timingConfig::sleepTimeOut};

    /// The status LED as last set (the remote shows it).
    Rgb::Colour::Value led_{};

    time_point_t next{Clock::now()};
    time_point_t menuTime{Clock::now() + timingConfig::menuTimeOut};
    time_point_t failTime{Clock::now() + timingConfig::failTimeOut};

    struct Init {};

    struct Sleep {
        explicit Sleep(time_point_t t_sleep,
                       time_point_t t_standby)
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
        solderTemp_ = RotaryEncoder::type{static_cast<std::uint16_t>(
          Remote::sanitized(TempEeeprom::value().values)[Remote::solderC])};
        heaterManager.setTemp(solderTemp_);
        AdcIsr::temp = &heaterManager;
        RotaryEncoder::cnt.store(solderTemp_, std::memory_order_relaxed);
    }

    void setLed(Rgb::Colour::Value const& c) {
        led_ = c;
        ledDriver.template set<Rgb::Colour>(c);
    }

    /// The solder temperature as a knob press takes it: the knob and the display move to it, the EEPROM stores it.
    void setSolderTemp(std::uint16_t c) {
        solderTemp_ = RotaryEncoder::type{c};
        RotaryEncoder::cnt.store(solderTemp_, std::memory_order_relaxed);
        preSetTemp_                                  = solderTemp_;
        TempEeeprom::value().values[Remote::solderC] = solderTemp_;
        TempEeeprom::writeValue();
    }

    void setSleepTemp(std::uint16_t c) { sleepTemp_ = RotaryEncoder::type{c}; }

    /// New timeouts count from the next state change.
    void setTimeouts(typename Clock::duration standby,
                     typename Clock::duration sleepDelay) {
        standbyTimeOut_ = standby;
        sleepTimeOut_   = sleepDelay;
    }

    /// Application::stateMachine's index: Protocol::State.
    [[nodiscard]] std::size_t state() const { return st_.index(); }

    bool tip1oldSleep{false};
    bool tip1oldRemoved{false};

    template<class... Ts>
    struct overloaded : Ts... {
        using Ts::operator()...;
    };
    template<class... Ts>
    overloaded(Ts...) -> overloaded<Ts...>;

    template<typename Variant,
             typename... Matchers>
    auto match(Variant&& variant,
               Matchers&&... matchers) {
        return std::visit(overloaded{std::forward<Matchers>(matchers)...},
                          std::forward<Variant>(variant));
    }

    void turn() {
        auto const currentTime{Clock::now()};
        //TODO
        bool         tip1Sleep{heaterManager.tip1Sleep};
        bool         tip1Removed{heaterManager.tip1Remove};
        std::uint8_t handlingEvent{0};
        if(tip1Sleep != tip1oldSleep) { ++handlingEvent; }
        if(tip1Removed != tip1oldRemoved) { ++handlingEvent; }
        tip1oldRemoved = tip1Removed;
        tip1oldSleep   = tip1Sleep;

        preSetTemp_ = RotaryEncoder::cnt.load(std::memory_order_relaxed);
        //display.setText("1223", std::nullopt);
        Button::handler([&](auto e, [[maybe_unused]] auto const& t) {
            switch(e) {
            case Button::Event::Type::releaseShort:
                {
                    //        modeChange           = true;
                    //        rotaryMode           = false;
                    solderTemp_                                  = preSetTemp_;
                    TempEeeprom::value().values[Remote::solderC] = solderTemp_;
                    TempEeeprom::writeValue();

                    //display.setStaticMode();
                    ++handlingEvent;
                }
                break;
            //TODO change button type
            case Button::Event::Type::hit:
            case Button::Event::Type::longPress:
            case Button::Event::Type::releaseLong: break;
            }
        });

        if(currentTime > next) {
            //KL_D("remove: {} sleep: {}", tip1Removed.getValue(), tip1Sleep.getValue());
            //KL_D("{}", st_.index());
            next += 10ms;
        }

        if(currentTime > failTime) { st_ = Shutdown{}; }
        // the tip did not warm at full power (Temp::watchFullPower): latched like the 1 h limit, a retry would heat
        // at full power again
        if(heaterManager.runaway.load(std::memory_order_relaxed)
           && !std::holds_alternative<Shutdown>(st_))
        {
            UC_LOG_C("tip did not warm at full power: heater off until a power cycle");
            st_ = Shutdown{};
        }
        watchTransformer();

        st_ = match(
          st_,
          [&]([[maybe_unused]] Init const& state) -> stateMachine {
              //Do init steps
              return Sleep{currentTime + sleepTimeOut_, currentTime + standbyTimeOut_};
          },
          [&](Sleep const& state) -> stateMachine {
              if(currentTime > state.timeOutSleep) {
                  setLed({0, 0, 3});
                  heaterManager.setTemp(sleepTemp_);
              } else {
                  setLed({0, 3, 0});
              }

              if(currentTime > state.timeOutStandby) { return Standby{}; }
              if(tip1Sleep) { return state; }
              return Solder{currentTime + standbyTimeOut_};
          },
          [&](Solder const& state) -> stateMachine {
              setLed({0, 0, 0});
              heaterManager.setTemp(solderTemp_);
              if(tip1Sleep) {
                  return Sleep{currentTime + sleepTimeOut_, currentTime + standbyTimeOut_};
              }
              if(tip1Removed) { return Remove{}; }
              if(currentTime > state.timeOutStandby) { return Standby{}; }
              return state;
          },
          [&](Remove const& state) -> stateMachine {
              heaterManager.setTemp(0);
              setLed({1, 0, 2});
              if(tip1Removed) { return state; }
              return Sleep{currentTime + sleepTimeOut_, currentTime + standbyTimeOut_};
          },
          [&](Standby const& state) -> stateMachine {
              setLed({1, 1, 0});
              heaterManager.setTemp(0);
              if(handlingEvent > 0) {
                  handlingEvent = 0;
                  return Sleep{currentTime + sleepTimeOut_, currentTime + standbyTimeOut_};
              }
              return state;
          },
          [&](Shutdown const& state) -> stateMachine {
              heaterManager.setTemp(0);
              setLed({10, 0, 0});
              //To leave this state you must reboot the system
              return state;
          });

        if(heaterManager.heaterOn) {
            setLed({1, 0, 0});
        } else {
        }

        if(preSetTemp_ != solderTemp_) {
            ++actionCounter_;
            display.setBlinkMode();
        } else {
            display.setStaticMode();
        }
        display.displayValue = preSetTemp_;

        tempSensTransformer.turn();
        display.turn();
        parts.turn();
        heaterManager.turn();
        TempOneWire::turn();
    }

    /// The transformer's limit (safetyConfig): the heater held off while it is hot, back by itself once it cooled.
    void watchTransformer() {
        auto const t = tempSensTransformer.temperature();
        if(!t) { return; }
        auto const milliC = t->numerical_value_in(t->unit);
        bool const held   = heaterManager.inhibited.load(std::memory_order_relaxed);
        if(!held && milliC > safetyConfig::transformerMaxMilliC) {
            UC_LOG_W("transformer {} m°C: heater held off", milliC);
            heaterManager.inhibited.store(true, std::memory_order_relaxed);
        } else if(held && milliC < safetyConfig::transformerResumeMilliC) {
            UC_LOG_I("transformer {} m°C: heater free again", milliC);
            heaterManager.inhibited.store(false, std::memory_order_relaxed);
        }
    }

    void fault() {
        //Display ERR
        setLed({10, 0, 0});
        display.turn();
        parts.turn();
    }
};
