#pragma once

#include "HWConfig.hpp"
#include "chip/atsam_common/DMAC.hpp"
#include "chip/atsam_common/Sercom_I2CQueued.hpp"
#include "chip/atsam_common/Sercom_Usart.hpp"
////

#include "Temperature.hpp"
#include "ZeroCrossDetector.hpp"
#include "kvasir/Devices/OneWire.hpp"
#include "kvasir/Runtime/Standard.hpp"
#include "kvasir/Util/SaturatingValue.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "remote/Tunables.hpp"

#include <chip/Evsys.hpp>
#include <chip/atsam_common/SamPushButton.hpp>
#include <chip/atsam_common/SamRotaryEncoder.hpp>
#include <compare>
#include <kvasir/StartUp/StartUp.hpp>

using Clock = HW::SystickClock;
using Evsys = Kvasir::EVSYS::EvsysBase;
/// The queued SERCOM master: the same interface the RP2040 and RP2350 buses present, which
/// is what the kvasir_devices chip descriptions are written against.
using I2C = Kvasir::Sercom::I2C::I2CBehaviorQueued<HW::I2CConfig, Clock, 8, 16>;

// the default clean-up: the image's fatal start, HW::PinConfig::safeState (every heater and tip switch off)
// the fault handler, the stack protector, a boot-loop guard and the boot report (kvasir/Runtime/Standard.hpp)
using Runtime = Kvasir::Runtime::Standard<Clock>;

/// A panic (assert, abort, a smashed stack, a sanitizer report, a fault): the safe state ran first (the fatal start
/// below), the record for the next boot, then wait: HW::Watchdog (2 s, not fed any more) resets the board.
template<>
struct Kvasir::Panic::PanicReaction<Kvasir::Tag::User> {
    using type = Kvasir::Panic::OnPanic::Wait;
};

using TempOneWire = Kvasir::OneWire<Clock, HW::Pin::trafoTemp, 16>;

struct DmacConfig {
    static constexpr auto numberOfChannels = 1;
};

using Dma = Kvasir::DMAC::DmaBase<DmacConfig>;

/// The remote's UART (remote/Remote.hpp): receive interrupt into 64 bytes, transmit by DMA channel 0.
using RemoteUart = Kvasir::Sercom::Usart::UartBehavior<HW::DebugUartConfig,
                                                       Clock,
                                                       Dma,
                                                       Kvasir::DMAC::DMAChannel::ch0,
                                                       Kvasir::DMAC::DMAPriority::p0,
                                                       64>;

/// The NVIC priority of the EIC interrupt, which the zero crossings, the encoder and the button
/// share.
inline constexpr int EicPriority = 2;

struct ButtonConfig {
    static constexpr auto longPressTime   = std::chrono::seconds{1};
    static constexpr auto useLong         = true;
    static constexpr auto useLongRelease  = true;
    static constexpr auto useShortRelease = true;
    static constexpr auto useTime         = true;
    static constexpr auto useHit          = true;
    static constexpr auto invert          = false;
    static constexpr auto eicPriority     = EicPriority;
};

struct RotaryConfig {
    static constexpr auto useAcceleration = true;
    static constexpr auto eicPriority     = EicPriority;
};

using Button        = Kvasir::SamPushButton<Clock, HW::Pin::rotarySw, 4, ButtonConfig>;
using RotaryEncoder = Kvasir::SamRotaryEncoder<Clock,
                                               HW::Pin::rotaryA,
                                               HW::Pin::rotaryB,
                                               Kvasir::SaturatingValue<std::uint16_t, 0, 512>,
                                               RotaryConfig>;

using ZCD = ZeroCrossDectecor<Clock, HW::Pin::zeroCross, EicPriority>;

using Eic = Kvasir::EIC::EicBase;

/// What the station keeps across a reset: every remote tunable (remote/Tunables.hpp), the solder
/// temperature among them. A knob press stores the solder temperature, the remote's StoreTunables all of
/// them. Nothing valid in the emulation (a new image, another layout): the defaults.
struct StoredSettings {
    auto           operator<=>(StoredSettings const&) const = default;
    Remote::Values values{Remote::defaults()};
};

using TempEeeprom = Kvasir::EepromEmulator<Clock, StoredSettings, 2, Kvasir::Nvm::Array::rwwee>;
using Float_t     = float;

// kd is positive: it damps (Kvasir::Control::Pid takes the derivative on the measurement).
// Ki is 0: a PD controller.
inline constexpr Kvasir::Control::PidGains<Float_t> HeaterGains{
  .kp     = static_cast<Float_t>(0.3),
  .ki     = static_cast<Float_t>(0.000),
  .kd     = static_cast<Float_t>(0.2),
  .tau    = static_cast<Float_t>(0.7),
  .outMin = static_cast<Float_t>(0.0),
  .outMax = static_cast<Float_t>(10.0),
};

using Temp_t = Temp<Clock, Float_t, HeaterGains, ZCD, Evsys>;

struct AdcIsr {
    using regs = Kvasir::Peripheral::ADC::Registers<>;
    static inline Temp_t* temp{nullptr};
    using InterruptIndex = decltype(Kvasir::Interrupt::adc);

    static constexpr auto initStepInterruptConfig
      = list(action(Kvasir::Nvic::Action::SetPriority<1>{}, InterruptIndex{}),
             action(Kvasir::Nvic::Action::clearPending, InterruptIndex{}));

    // ISR
    static void onIsr() {
        if(temp != nullptr) {
            temp->adcCallback();
        } else {
            apply(read(regs::RESULT::result));
        }
    }

    using Isr = Kvasir::Nvic::Isr<std::addressof(onIsr), std::remove_const_t<InterruptIndex>>;
};

using Startup = Kvasir::Startup::Startup<HW::ClockSettings,
                                         HW::ComBackend,
                                         Runtime,
                                         Clock,
                                         I2C,
                                         // the EIC's clock and enable; ZCD, RotaryEncoder and
                                         // Button bring their own EXTINT lines. The app drives
                                         // Button::handler() itself.
                                         Eic,
                                         ZCD,
                                         RotaryEncoder,
                                         Button,
                                         Evsys,
                                         AdcIsr,
                                         Dma,
                                         RemoteUart,
                                         HW::PinConfig>;

// The image's safe state (every Hook::SafeState entry of the list) is what each fatal path does first.
template<>
struct Kvasir::Panic::FatalStartReaction<Kvasir::Tag::User> {
    using type = Kvasir::Startup::SafeStateOf<::Startup>;
};
