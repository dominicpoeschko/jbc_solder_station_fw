//
// Created by patrick on 1/25/22.
//
#pragma once

#include <optional>
#include <charconv>
#include <utility>
#include <algorithm>
#include <array>
#include "kvasir/Devices/pca9956b.hpp"
#include "kvasir/Util/using_literals.hpp"

template <typename I2C, typename Clock>
struct DisplayManager{
    using tp = typename Clock::time_point;

    std::uint32_t displayValue;
    Kvasir::Pca9956b<I2C, Clock>         display;

    static constexpr auto offTime{std::chrono::milliseconds{200}};
    static constexpr auto onTime{std::chrono::milliseconds{400}};
    static constexpr auto refreshRate{std::chrono::milliseconds{50}};
    static_assert(offTime > refreshRate);
    static_assert(onTime > refreshRate);

    tp nextUpdate;
    tp blinkUpdate;

    bool displayOn{true};

    enum class State {
        static_mode,
        blink_mode
    };

    State st{State::static_mode};

    explicit DisplayManager():
    display(0x3F, 2200)
    {
        display.setCurrentAll(5);
        display.setPwmAll(255);
    }

    void setBlinkMode()
    {
        st = State::blink_mode;
    }

    void setStaticMode()
    {
        st = State::static_mode;
    }

    void handler(){
        auto now{Clock::now()};
        if(now > nextUpdate) {
            setNum(
                    displayValue,
                    std::nullopt
            );
            switch (st) {
                case State::static_mode: {
                    displayOn = true;
                    display.setPwmAll(255);
                }
                    break;

                case State::blink_mode: {
                    if(now > blinkUpdate){
                        if(displayOn)
                        {
                            displayOn = false;
                            display.setPwmAll(0);
                            blinkUpdate = now + offTime;
                        }
                        else{
                            displayOn = true;
                            display.setPwmAll(255);
                            blinkUpdate = now + onTime;
                        }
                    }
                }
                    break;
            }
            nextUpdate = now + refreshRate;
        }
        display.handler();
    }


    void setNum(std::uint32_t v, std::optional<std::uint8_t> dotpos) {
        static constexpr std::array numbers{
                0xFC_b, 0xFC_b,   // 0
                0xC0_b, 0x0C_b,   // 1
                0xF3_b, 0xF0_b,   // 2
                0xF3_b, 0x3C_b,   // 3
                0xCF_b, 0x0C_b,   // 4
                0x3F_b, 0x3C_b,   // 5
                0x3F_b, 0xFC_b,   // 6
                0xF0_b, 0x0C_b,   // 7
                0xFF_b, 0xFC_b,   // 8
                0xFF_b, 0x3C_b,   // 9
                0x00_b, 0x00_b,   // space
        };
        v = std::clamp(v, 0_u32, 999_u32);
        std::array<char, 3> num{};
        auto const r =
            std::next(num.begin(), std::distance(num.data(), std::to_chars(num.data(), num.data() + num.size(), std::uint16_t(v)).ptr));
        std::transform(begin(num), r, begin(num), [](auto c) { return (c - '0') * 2_u; });
        std::fill(r, end(num), 20);
        std::array<char, 3> num2{};
        std::rotate_copy(begin(num), r, end(num), rbegin(num2));

        std::array<std::byte, 3 * 2> data{};
        std::size_t const            dotIndex = dotpos.value_or(num2.size());
        for(std::size_t o{}, n{}; o < data.size(); o += 2, ++n) {
            data[o]     = numbers[num2[n]];
            data[o + 1] = std::byte(std::size_t(numbers[num2[n] + 1]) + (dotIndex == n ? 3 : 0));
        }
        display.setLedOut(data);
    }



    void clear() {
        std::array<std::byte, 3 * 2> data{0x00_b, 0x00_b,0x00_b, 0x00_b,0x00_b, 0x00_b};
        display.setLedOut(data);
    }

};
