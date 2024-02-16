//
// Created by patrick on 1/30/22.
//

#pragma once

template <typename Clock>
struct UnBounce{
    using time_point_t = typename Clock::time_point;
    std::chrono::duration<double> bounceTime_{};
    bool bounceVal_{false};
    bool bounceValOld_{false};
    bool unBouncedVal_{false};

    explicit UnBounce(std::chrono::duration<double> const bounceTime): bounceTime_{bounceTime} {}

    struct Idle{};
    struct WaitSecondValue{
        explicit WaitSecondValue(time_point_t t): bounceTimeOut{t} {}
        time_point_t bounceTimeOut{};
    };
    struct StillEqual{};

    using stateMachine = std::variant<Idle, WaitSecondValue, StillEqual>;
    stateMachine st_{Idle{}};

    void setValue(bool const bounceVal){bounceVal_ = bounceVal;}
    bool getValue() const {return unBouncedVal_;}

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

    void handler(){
        auto const currentTime = Clock::now();

        st_ = match(
                st_,
                [&](Idle const& state) -> stateMachine {
                    if(bounceVal_ != bounceValOld_) //-> State has changed
                    {
                        bounceValOld_ = bounceVal_;
                        return WaitSecondValue{currentTime + bounceTime_};
                    }
                    else
                    {
                        return state;
                    }
                },
                [&](WaitSecondValue const& state) -> stateMachine {
                    if(currentTime > state.bounceTimeOut)
                    {
                        return StillEqual{};
                    }
                    else
                    {
                        return state;
                    }

                },
                [&](StillEqual const& ) -> stateMachine {
                    if(bounceVal_ == bounceValOld_){
                        bounceValOld_ = bounceVal_;
                        unBouncedVal_ = bounceVal_;
                        return Idle{};
                    }
                    else{
                        return Idle{};
                    }
                }

        );
    }

};
