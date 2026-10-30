#pragma once
// The smallest harness (as fs's): named cases, checks that print where they failed, a count.
#include <cstdio>
#include <source_location>

namespace Test {

inline int         failures{};
inline char const* currentCase{"?"};

inline void testCase(char const* name) {
    currentCase = name;
    std::printf("%s\n", name);
}

inline bool check(bool                        ok,
                  char const*                 what,
                  std::source_location const& at = std::source_location::current()) {
    if(!ok) {
        ++failures;
        std::printf("  FAIL [%s] %s:%u: %s\n", currentCase, at.file_name(), at.line(), what);
    }
    return ok;
}

inline int finish() {
    if(failures == 0) {
        std::printf("all checks passed\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}

}   // namespace Test
