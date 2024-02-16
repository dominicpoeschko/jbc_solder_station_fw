#pragma once

#include "HWConfig.hpp"
////

#include "Temperature.hpp"
#include "ZeroCrossDetector.hpp"
#include "kvasir/Devices/OneWire.hpp"
#include "kvasir/Util/FaultHandler.hpp"
#include "kvasir/Util/SaturatingValue.hpp"
#include "kvasir/Util/StackProtector.hpp"
#include "kvasir/Util/using_literals.hpp"

#include <compare>
struct EvsysConfig {};
using Clock = HW::SystickClock;
using Evsys = Kvasir::EVSYS::EvsysBase<EvsysConfig>;
using I2C   = Kvasir::Sercom::I2C::I2CBehavior<HW::I2CConfig, Clock, 32>;

using StackProtector   = Kvasir::StackProtector<>;
using HardFaultHandler = Kvasir::Fault::Handler<HW::Fault_CleanUpAction>;

using TempOneWire = Kvasir::OneWire<Clock, HW::Pin::trafoTemp, 16>;

struct EicConfig {
    static constexpr auto IsrPriority = 2;
};

struct ButtonConfig {
    static constexpr auto longPressTime   = std::chrono::seconds{1};
    static constexpr auto useLong         = true;
    static constexpr auto useLongRelease  = true;
    static constexpr auto useShortRelease = true;
    static constexpr auto useTime         = true;
    static constexpr auto useHit          = true;
    static constexpr auto invert          = false;
};

struct RotaryConfig {
    static constexpr auto useAcceleration = true;
};

using Button        = Kvasir::SamPushButton<Clock, HW::Pin::rotarySw, 4, ButtonConfig>;
using RotaryEncoder = Kvasir::SamRotaryEncoder<
  Clock,
  HW::Pin::rotaryA,
  HW::Pin::rotaryB,
  Kvasir::SaturatingValue<std::uint16_t, 0, 512>,
  RotaryConfig>;

using ZCD = ZeroCrossDectecor<Clock, HW::Pin::zeroCross>;

using Eic = Kvasir::EIC::
  EicBase<Clock, EicConfig, ZCD::EicConfig, RotaryEncoder::EicConfig, Button::EicConfig>;

struct TempSaveType {
    auto          operator<=>(const TempSaveType&) const = default;
    std::uint16_t temp{};
};

using TempEeeprom = Kvasir::EepromEmulator<Clock, TempSaveType, 2, false>;
using Float_t     = float;

struct PidConfig {
    using type               = Float_t;
    static constexpr type Kp = type(0.3);
    static constexpr type Ki = type(0.000);
    static constexpr type Kd = type(-0.2);

    static constexpr type tau = type(0.7);

    static constexpr type limMin = type(0.0);
    static constexpr type limMax = type(10.0);
};

using Temp_t = Temp<Clock, Float_t, PidConfig, ZCD, Evsys>;

struct AdcIsr {
    using regs = Kvasir::Peripheral::ADC::Registers<>;
    static inline Temp_t* temp{nullptr};
    using InterruptIndex = decltype(Kvasir::Interrupt::adc);

    static constexpr auto initStepInterruptConfig = list(
      action(Kvasir::Nvic::Action::SetPriority<1>{}, InterruptIndex{}),
      action(Kvasir::Nvic::Action::clearPending, InterruptIndex{}));

    // ISR
    static void onIsr() {
        if(temp != nullptr) {
            temp->adcCallback();
        } else {
            apply(read(regs::RESULT::result));
        }
    }

    static constexpr Kvasir::Nvic::Isr<std::addressof(onIsr), std::remove_const_t<InterruptIndex>>
      isr{};
};

using Startup = Kvasir::Startup::Startup<
  HW::ClockSettings,
    HW::ComBackend,
  Clock,
  HardFaultHandler,
  StackProtector,
  I2C,
  Eic,
  Evsys,
  AdcIsr,
  RotaryEncoder,
  Button,
  HW::PinConfig>;

