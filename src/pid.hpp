#pragma once
#include <algorithm>
#include <chrono>

template<typename Clock, typename Config>
struct Pid {
    using T  = typename Config::type;
    using tp = typename Clock::time_point;

    T integrator{};
    T prevError{};
    T differentiator{};
    T prevMeasurment{};

    tp prevTime{};

    T update(T setpoint, T measurment) {
        auto const currentTime = Clock::now();
        T const    error       = setpoint - measurment;

        T const proportional = Config::Kp * error;

        T out = proportional;
        if(prevTime != tp{}) {
            T const deltaT = static_cast<T>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              currentTime - prevTime)
                                              .count())
                           / T{1000000000.0};
            integrator += T{0.5} * Config::Ki * deltaT * (error + prevError);

            T const integratorMax
              = Config::limMax > proportional ? Config::limMax - proportional : T{};

            T const integratorMin
              = Config::limMin < proportional ? Config::limMin - proportional : T{};

            integrator = std::clamp(integrator, integratorMin, integratorMax);

            differentiator = (T{2} * Config::Kd * (measurment - prevMeasurment)
                              + (T{2} * Config::tau - deltaT) * differentiator)
                           / (T{2} * Config::tau + deltaT);

            out += integrator + differentiator;
       /*     KL_T(
              "{:10.1f} {:10.1f} {:10.6f} {:10.6f} {:10.6f} {:10.6f} {}",
              setpoint,
              measurment,
              proportional,
              integrator,
              differentiator,
              out,
              deltaT);*/
        }

        prevTime       = currentTime;
        prevMeasurment = measurment;
        prevError      = error;

       // Kvasir::breakPoint();
        auto ret = std::clamp(out, Config::limMin, Config::limMax);
      // Kvasir::breakPoint();
        return ret;
        //auto var = std::clamp(out,Config::limMin, Config::limMax);
        /*  if(out > Config::limMax){
            return Config::limMax;
        }
        else if(out < Config::limMin){
            return Config::limMin;
        }
        else{
            return out;
        }*/
    }
};
