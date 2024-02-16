#include "Application.hpp"
// need to be included first

#include "cmake_git_version/version.hpp"

int main() {
    UC_LOG_I("{}", CMakeGitVersion::FullVersion);
    //TODO unleash the dog
    Kvasir::Fuses::setFuseBits<Clock>(HW::fuseBits(), false);
    Application<Clock> app;

    auto next = Clock::time_point{};

    while(Clock::now() < Clock::time_point{} + timingConfig::maxRuntime) {
        app.handler();
        StackProtector::handler();
        if(Clock::now() > next) {
            next += 1s;
            UC_LOG_D("alive...");
        }
    }
    HW::Fault_CleanUpAction();
    UC_LOG_C("Failed...");
    while(true) {
        app.fault();
        //TODO feed the dog
    }
}

KVASIR_START(Startup)
