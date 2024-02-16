#pragma once

bool dbgpres();

struct DebuggerPresentFunction {
    bool operator()() { return dbgpres(); }
};

#include "uc_log/DefaultRttComBackend.hpp"

namespace uc_log {
template<>
struct ComBackend<uc_log::Tag::User>
  : public DefaultRttComBackend<DebuggerPresentFunction, 512, 4096, rtt::BufferMode::skip> {};
}   // namespace uc_log
// need to be included first

#include "chip/Interrupt.hpp"
#include "core/core.hpp"
#include "uc_log/uc_log.hpp"

namespace HW {
static constexpr auto ClockSpeed   = 48'000'000;
static constexpr auto CrystalSpeed = 8'000'000;

struct SystickClockConfig {
    static constexpr auto clockBase = Kvasir::Systick::useProcessorClock;

    static constexpr auto clockSpeed     = ClockSpeed;
    static constexpr auto minOverrunTime = std::chrono::hours(24 * 365) * 20;
};
using SystickClock = Kvasir::Systick::SystickClockBase<SystickClockConfig>;
}   // namespace HW

namespace uc_log {
template<>
struct LogClock<uc_log::Tag::User> : public HW::SystickClock {};
}   // namespace uc_log

#include "chip/chip.hpp"

bool inline dbgpres() {
    return apply(read(Kvasir::Peripheral::DSU::Registers<>::STATUSB::dbgpres));
}

namespace HW {
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
    static void coreClockInit() {
        using Kvasir::Register::value;
        // ClockSpeed=(CrystalSpeed*(1/(2*(DPLL_div+1))))*(DPLL_ldr+1+(DPLL_ldrfrac/16))/GENDIV_div
        // if ClockSpeed > 24MHZ flash_waitstates=1 else 0

        using namespace Kvasir::GCLK;
        using KSR                              = Kvasir::Peripheral::SYSCTRL::Registers<>;
        using KNR                              = Kvasir::Peripheral::NVMCTRL::Registers<>;
        static constexpr auto flash_waitstates = KNR::CTRLB::RWSVal::half;
        static constexpr auto DPLL_ldr         = 3095;
        static constexpr auto DPLL_ldrfrac     = 0;
        static constexpr auto DPLL_div         = 128;
        static constexpr auto GENDIV_div       = 2;
        // 48'000'000=(8000000*(1/(2*(128+1))))*(3095+1+(0/16))/2

        static_assert(
          ClockSpeed
            == std::uint64_t(
              (CrystalSpeed * (1.0 / (2.0 * (double(DPLL_div) + 1.0))))
              * (double(DPLL_ldr) + 1.0 + (double(DPLL_ldrfrac) / 16.0)) / double(GENDIV_div)),
          "ClockSpeed wrong");

        static_assert(
          ClockSpeed <= 24'000'000 || flash_waitstates == KNR::CTRLB::RWSVal::half,
          "need flash_waitstates");

        apply(KSR::XOSC::overrideDefaults(
          write(KSR::XOSC::startup, value<std::uint16_t, 15>()),
          set(KSR::XOSC::ampgc),
          write(KSR::XOSC::GAINValC::_2),
          clear(KSR::XOSC::ondemand),
          set(KSR::XOSC::runstdby),
          set(KSR::XOSC::xtalen),
          set(KSR::XOSC::enable)));
        // wait for XOSCRDY ready
        while(!apply(read(KSR::PCLKSR::xoscrdy))) {
        }

        apply(
          write(KSR::DPLLRATIO::ldr, value<DPLL_ldr>()),
          write(KSR::DPLLRATIO::ldrfrac, value<DPLL_ldrfrac>()),
          KSR::DPLLCTRLB::overrideDefaults(
            write(KSR::DPLLCTRLB::div, value<DPLL_div>()),
            write(KSR::DPLLCTRLB::REFCLKValC::ref1)),
          Kvasir::Register::sequencePoint,
          KSR::DPLLCTRLA::overrideDefaults(
            clear(KSR::DPLLCTRLA::ondemand),
            set(KSR::DPLLCTRLA::enable),
            set(KSR::DPLLCTRLA::runstdby)));
        // wait for clkrdy ready
        while(!apply(read(KSR::DPLLSTATUS::clkrdy))) {
        }

        apply(
          KNR::CTRLB::overrideDefaults(
            write(KNR::CTRLB::rws, value<KNR::CTRLB::RWSVal, flash_waitstates>())),
          GenericClockGenerator<0, GeneratorSource::fdpll, GENDIV_div>::enable());

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
        apply(
          GenericClockGenerator<3, GeneratorSource::fdpll, 64>::enable(),
          Kvasir::Register::sequencePoint,
          PeripheralChannelController<0, Peripheral::sercom1_core>::enable(),
          PeripheralChannelController<0, Peripheral::eic>::enable(),
          PeripheralChannelController<0, Peripheral::adc>::enable(),
          PeripheralChannelController<0, Peripheral::evsys_0>::enable(),
          PeripheralChannelController<3, Peripheral::sercom3_core>::enable());
    }
};
/*
struct DebugUartConfig {
    static constexpr auto clockSpeed = ClockSpeed;

    static constexpr auto usartInstance    = 1;
    static constexpr auto rxPinLocation    = Kvasir::Sercom::Usart::NotUsed{};
    static constexpr auto txPinLocation    = Pin::debugTx{};
    static constexpr auto baudRate         = 115'200;
    static constexpr auto maxBaudRateError = std::ratio<1, 10000>{};
    static constexpr auto dataBits         = Kvasir::Sercom::Usart::DataBits::_8;
    static constexpr auto parity           = Kvasir::Sercom::Usart::Parity::none;
    static constexpr auto stopBits         = Kvasir::Sercom::Usart::StopBits::_1;
    static constexpr auto IsrPriority      = 3;

    static constexpr auto userConfigOverride = brigand::list<>{};
};
*/
struct I2CConfig {
    static constexpr auto clockSpeed = ClockSpeed / 32;

    static constexpr auto instance       = 3;
    static constexpr auto sdaPinLocation = Pin::Sda{};
    static constexpr auto sclPinLocation = Pin::Scl{};
    static constexpr auto baudRate       = 100'000;
    static constexpr auto isrPriority    = 3;
};

struct Fault_CleanUpAction {
    void operator()() {
        apply(
          clear(HW::Pin::tip1Sw{}),
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
};

constexpr Kvasir::Fuses::FuseBits fuseBits() {
    using namespace Kvasir::Fuses;
    FuseBits sollFuseBits{
      Bod33{
            Bod33::Action::Reset,
            Bod33::Enable::enabled,
            Bod33::Hysteresis::enabled,
            Bod33::Level{40}},
      Wdt{
            Wdt::Enable::disabled,
            Wdt::AlwaysOn::disabled,
            Wdt::WindowEnable::disabled,
            Wdt::Period{11},
            Wdt::Window{11},
            Wdt::Offset{11}},
      Nvm{     Nvm::Bootprot{7}, Nvm::Eeprom{7}, Nvm::Lock{0xffff} }
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
    static constexpr auto initStepPinConfig = list(
      makeInput(HW::Pin::tip1Sleep{}),
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
      set(HW::Pin::tip1Sw{})

    );
    // apply(clear(HW::Pin::tip2TcSw{}));
    // apply(set(HW::Pin::select_24_48{}));
};

}   // namespace HW
