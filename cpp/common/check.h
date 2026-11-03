#pragma once

// 极简测试辅助：CHECK 失败时打印位置并计数，report() 汇总并返回进程退出码

#include <cstdio>

namespace ga::test {
inline int failures = 0;
}

#define CHECK(expr)                                                                       \
    do {                                                                                  \
        if (!(expr)) {                                                                    \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #expr); \
            ++ga::test::failures;                                                         \
        }                                                                                 \
    } while (0)

inline int report(const char* name) {
    if (ga::test::failures == 0) {
        std::printf("%s: all checks passed\n", name);
        return 0;
    }
    std::printf("%s: %d check(s) failed\n", name, ga::test::failures);
    return 1;
}
