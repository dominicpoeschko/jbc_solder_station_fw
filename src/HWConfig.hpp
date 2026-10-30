#pragma once

// uc_log.hpp reads kvasir_config.hpp (log sink, log clock) before anything else; every other header in
// any order.
#include "chip/chip.hpp"
#include "uc_log/uc_log.hpp"

#include <chip/Fuses.hpp>
#include <kvasir/Hal/Uart.hpp>

namespace HW {
static constexpr auto crystalSpeed   = 8'000'000;
static constexpr auto WDTClockDivide = 32;   // OSCULP32K down to the 1.024 kHz the WDT counts

using ComBackend = uc_log::ComBackend<uc_log::Tag::User>;

namespace Pin {
    using a2           = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin2));
    using tip3TcSw2    = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin3));
    using vRef         = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin4));
    using sleepSw      = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin5));
    using tip3Sw1      = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin6));
    using tip3Sw2      = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin7));
    using tip2Sw       = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin8));
    using tip1Sw       = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin9));
    using tipComSw     = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin10));
    using select_24_48 = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin11));
    using zeroCross    = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin12));
    using rotarySw     = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin13));
    using xIn          = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin14));
    using xOut         = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin15));
    using rotaryB      = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin16));
    using rotaryA      = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin17));
    using debugTx      = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin18));
    using debugRx      = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin19));
    using tip3Sleep    = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin20));
    using piezo        = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin21));
    using Sda          = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin22));
    using Scl          = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin23));
    using a24          = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin24));
    using a25          = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin25));
    using tip3Remove   = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin27));
    using tip2Sleep    = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin28));
    using swClk        = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin30));
    using swDio        = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin31));

    using tip2Remove   = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin0));
    using tip1Sleep    = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin1));
    using tip1Remove   = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin2));
    using tip3TcSw1    = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin3));
    using tip2TcSw     = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin4));
    using tip1TcSw     = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin5));
    using tempSense    = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin8));   // AIN[2]
    using currentSense = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin9));   // AIN[3]
    using b10          = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin10));
    using trafoTemp    = decltype(makePinLocation(Kvasir::Io::portB, Kvasir::Io::pin11));
}   // namespace Pin

struct ClockSettings {
    /// XOSC 8 MHz -> FDPLL96M -> GCLK0 at 48 MHz, from the target (chip/ClockTree.hpp): the solver
    /// takes the lowest DCO (48 MHz: errata DS80000760M 1.3.2 makes the FDPLL96M above 64 MHz
    /// unusable below 0 C on revisions A-D) and a 1 MHz reference (DIV 3) inside the 32 kHz..2 MHz
    /// f_IN of SAM D21 Tables 37-58/59/60; LBYPASS is on (errata 1.3.5, every revision: below 25 C
    /// a spurious unlock halts the output). The wait states come from Table 37-42 at 2.7-3.63 V.
    using Tree = Kvasir::Clocks::Sam::Tree<{.cpu            = clockSpeedValue,
                                            .source         = {.hz = crystalSpeed},
                                            .supply         = Kvasir::Nvm::Supply::from2V7,
                                            .dpllRunStandby = true}>;
    // The tree's clocks, and what peripheryClockInit routes (atsam_common Clocks.hpp): generator 3
    // (the FDPLL at ClockSpeed / 32) to SERCOM3, whose I2C claims its core clock at clockSpeed.
    // Generator 0 (the CPU clock) to SERCOM1, the remote's USART (DebugUartConfig).
    using Provides = brigand::append<
      Tree::Provides,
      brigand::list<Kvasir::Hal::Clk<Kvasir::Clocks::ClkSercomCore<3>, clockSpeedValue / 32>,
                    Kvasir::Hal::Clk<Kvasir::Clocks::ClkSercomCore<1>, clockSpeedValue>>>;

    // the expected solver result: DIV 3, LDR 47, GCLK0 / 1, 1 wait state
    static_assert(Tree::dpll.div == 3 && Tree::dpll.ldr == 47 && Tree::dpll.ldrFrac == 0
                  && Tree::dpll.gclkDiv == 1 && Tree::waitStates == 1);

    static void coreClockInit() {
        Tree::coreClockInit();

        /*
        // WD clock
        apply(write(KGR::GENDIV::genid, value<2>()),
              write(KGR::GENDIV::div, value<0>()),
              Kvasir::Register::sequencePoint,
              write(KGR::GENCTRL::genid, value<2>()),
              write(KGR::GENCTRL::SRCValC::osculp32k),
              write(KGR::GENCTRL::GENENValC::enabled),
              write(KGR::GENCTRL::IDCValC::duty_cycle_not_improved),
              write(KGR::GENCTRL::OOVValC::output_zero),
              write(KGR::GENCTRL::OEValC::output_disabled),
              write(KGR::GENCTRL::DIVSELValC::gendiv),
              write(KGR::GENCTRL::RUNSTDBYValC::enabled),
              Kvasir::Register::sequencePoint,
              write(KGR::CLKCTRL::genid, value<2>()),
              write(KGR::CLKCTRL::IDValC::gclk_wdt),
              write(KGR::CLKCTRL::CLKENValC::enabled),
              write(KGR::CLKCTRL::WRTLOCKValC::not_locked));*/
    }

    static void peripheryClockInit() {
        using namespace Kvasir::GCLK;
        apply(GenericClockGenerator<3, GeneratorSource::fdpll, 32>::enable(),   // 1.5 MHz
              GenericClockGenerator<4, GeneratorSource::osculp32k, WDTClockDivide>::enable(),
              Kvasir::Register::sequencePoint,
              PeripheralChannelController<0, Peripheral::sercom1_core>::enable(),
              PeripheralChannelController<0, Peripheral::eic>::enable(),
              PeripheralChannelController<0, Peripheral::adc>::enable(),
              PeripheralChannelController<0, Peripheral::evsys_0>::enable(),
              PeripheralChannelController<3, Peripheral::sercom3_core>::enable(),
              PeripheralChannelController<4, Peripheral::wdt>::enable());
    }
};

/// The remote's UART (remote/Protocol.hpp) on the debug pins: TX on PA18 = SERCOM1/PAD[2], RX on PA19 =
/// SERCOM1/PAD[3], both peripheral function C (SAM D21 DS40001882L Table 7-1, I/O multiplexing); TXPO and RXPO come
/// from the chip's pad table (Sercom_Usart.hpp).
struct DebugUartConfig {
    static constexpr auto clockSpeed    = clockSpeedValue;   // claims ClkSercomCore<1> at this rate
    static constexpr auto instance      = 1;
    static constexpr auto txPinLocation = Pin::debugTx{};
    static constexpr auto rxPinLocation = Pin::debugRx{};
    static constexpr auto baudRate      = 115'200;
    static constexpr auto dataBits      = Kvasir::Hal::Uart::DataBits::bits8;
    static constexpr auto parity        = Kvasir::Hal::Uart::Parity::none;
    static constexpr auto stopBits      = Kvasir::Hal::Uart::StopBits::stop1;
    static constexpr auto isrPriority
      = 3;   // below the ADC (1) and the EIC (2): the heater timing first
};

struct I2CConfig {
    static constexpr auto clockSpeed = clockSpeedValue / 32;

    static constexpr auto instance       = 3;
    static constexpr auto sdaPinLocation = Pin::Sda{};
    static constexpr auto sclPinLocation = Pin::Scl{};
    static constexpr auto baudRate       = 100'000;
    static constexpr auto isrPriority    = 3;
};

/// The watchdog (chip_atsam_common Wdt.hpp): generator 4 makes 32.768 kHz / 32 = 1.024 kHz of the
/// OSCULP32K, 2 s is CONFIG.PER 0x8 (2048 cycles). The heater runs from the ADC interrupt, so a main loop
/// that stops turning does not stop it: the reset does (the pins are inputs again).
struct WatchdogConfig {
    static constexpr auto clockSpeed  = 32'768 / WDTClockDivide;
    static constexpr auto overrunTime = std::chrono::seconds{2};
};

using Watchdog = Kvasir::Watchdog<WatchdogConfig>;

constexpr Kvasir::Fuses::FuseBits fuseBits() {
    using namespace Kvasir::Fuses;
    FuseBits sollFuseBits{
      Bod33{Bod33::Action::reset,
            Bod33::Enable::enabled,
            Bod33::Hysteresis::enabled,
            Bod33::Level{40}},
      Wdt{Wdt::Enable::disabled,
            Wdt::AlwaysOn::disabled,
            Wdt::WindowEnable::disabled,
            Wdt::Period{11},
            Wdt::Window{11},
            Wdt::Offset{11}},
      Nvm{Nvm::Bootprot{7}, Nvm::Eeprom{7}, Nvm::Lock{0xffff}}
    };
    return sollFuseBits;
}

template<bool on>
constexpr auto heat() {
    if constexpr(on) {
        return list(set(Pin::tipComSw{}), set(Pin::b10{}));
        //return list(clear(Pin::tipComSw{}), clear(Pin::b10{}));
        //return list(clear(Pin::tipComSw{}));
    } else {
        return list(clear(Pin::tipComSw{}), clear(Pin::b10{}));
        //return list(clear(Pin::tipComSw{}));
    }
}

struct PinConfig {
    static constexpr auto initStepPinConfig
      = list(makeInput(HW::Pin::tip1Sleep{}),
             makeInput(HW::Pin::tip1Remove{}),
             makeInput(HW::Pin::tip2Sleep{}),
             makeInput(HW::Pin::tip2Remove{}),
             makeInput(HW::Pin::tip3Sleep{}),
             makeInput(HW::Pin::tip3Remove{}),
             makeOutput(HW::Pin::tip1Sw{}),
             makeOutput(HW::Pin::tip2Sw{}),
             makeOutput(HW::Pin::tip3Sw1{}),
             makeOutput(HW::Pin::tip3Sw2{}),
             makeOutput(HW::Pin::tipComSw{}),
             makeOutput(HW::Pin::select_24_48{}),
             makeOutput(HW::Pin::sleepSw{}),
             makeOutput(HW::Pin::tip1TcSw{}),
             makeOutput(HW::Pin::tip2TcSw{}),
             makeOutput(HW::Pin::tip3TcSw1{}),
             makeOutput(HW::Pin::tip3TcSw2{}),
             makeOutput(HW::Pin::a2{}),
             makeOutput(HW::Pin::b10{}),
             action(Kvasir::Io::Action::PinFunction<1>{}, HW::Pin::vRef{}),
             action(Kvasir::Io::Action::PinFunction<1>{}, HW::Pin::tempSense{}),
             action(Kvasir::Io::Action::PinFunction<1>{}, HW::Pin::currentSense{}),
             //makeOutput(HW::Pin::piezo{})

             set(HW::Pin::tip1TcSw{}),
             clear(HW::Pin::sleepSw{}),
             //set(HW::Pin::sleepSw{}),
             set(HW::Pin::tip1Sw{}));
    // apply(clear(HW::Pin::tip2TcSw{}));
    // apply(set(HW::Pin::select_24_48{}));

    // The safe state (Hook::SafeState): every heater and tip switch off. Every fatal path runs it (the image's
    // fatal start, ApplicationConfig.hpp), and main() does when it gives up. PORT writes only; PORT's bus clock is
    // on from reset (D21 DS 16.8.9 APBBMASK, reset 0x7F).
    static void safeState() {
        apply(clear(HW::Pin::tip1Sw{}),
              clear(HW::Pin::tip2Sw{}),
              clear(HW::Pin::tip3Sw1{}),
              clear(HW::Pin::tip3Sw2{}),
              clear(HW::Pin::tipComSw{}),
              clear(HW::Pin::tip1TcSw{}),
              clear(HW::Pin::tip2TcSw{}),
              clear(HW::Pin::tip3TcSw1{}),
              clear(HW::Pin::tip3TcSw2{}),
              clear(HW::Pin::sleepSw{}),
              clear(HW::Pin::piezo{}),
              clear(HW::Pin::select_24_48{}),
              clear(HW::Pin::trafoTemp{}));
    }

    using Extends = Kvasir::Startup::Extend<Kvasir::Hook::SafeState, &safeState>;
};

}   // namespace HW
