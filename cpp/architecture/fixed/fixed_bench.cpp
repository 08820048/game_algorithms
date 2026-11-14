// 定点数的精度、速度，以及浮点在不同编译方式下的结果
//   c++ -std=c++20 -O2 -Icpp/common cpp/architecture/fixed/fixed_bench.cpp -o /tmp/fxb && /tmp/fxb [section]
//   section: precision | orbit | order | speed | platform（默认全部）
//   platform 打印几个哈希，用不同的编译选项（-ffp-contract=off、-ffast-math、em++ 编译后用 node 运行）各跑一次比较；
//   platform <file> 还会把 100 万个 sinf 的结果写进文件，用来逐个比较两个平台的数学库

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../command/command.h"
#include "fixed.h"

using namespace ga;
using namespace ga::fixed;

template <class F>
static double ms(F&& f) {
    auto t0 = std::chrono::steady_clock::now();
    f();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
template <class F>
static double best(int rounds, F&& f) {
    double b = 1e30;
    for (int i = 0; i < rounds; ++i) b = std::min(b, ms(f));
    return b;
}
constexpr double kTau = 6.283185307179586;

template <int F>
static void precisionRow() {
    Pcg32 r{7};
    double mul = 0, sq = 0, sn = 0;
    for (int i = 0; i < 100000; ++i) {
        double a = (r.nextFloat() - 0.5) * 8, b = (r.nextFloat() - 0.5) * 8;
        Fixed<F> fa = Fixed<F>::of(a), fb = Fixed<F>::of(b);
        mul = std::max(mul, std::fabs((fa * fb).toDouble() - fa.toDouble() * fb.toDouble()));
        Fixed<F> x = Fixed<F>::of(std::fabs(a) * 4);
        sq = std::max(sq, std::fabs(sqrt(x).toDouble() - std::sqrt(x.toDouble())));
        Fixed<F> t = Fixed<F>::of(r.nextFloat());
        sn = std::max(sn, std::fabs(sinTurns(t).toDouble() - std::sin(t.toDouble() * kTau)));
    }
    std::printf("Q%d.%-4d %12.3g %14.0f %12.3g %12.3g %12.3g\n", 31 - F, F, std::ldexp(1.0, -F), std::ldexp(1.0, 31 - F), mul, sq, sn);
}
static void precision() {
    std::printf("== formats: resolution, largest value, worst error of one operation vs double ==\n");
    std::printf("%-8s %12s %14s %12s %12s %12s\n", "format", "resolution", "max value", "mul err", "sqrt err", "sin err");
    precisionRow<8>(), precisionRow<12>(), precisionRow<16>(), precisionRow<20>(), precisionRow<24>();
    std::printf("float spacing: at 1 %.3g, at 100 %.3g, at 10000 %.3g, at 1e6 %.3g\n", double(std::nextafter(1.0f, 2.0f) - 1.0f),
                double(std::nextafter(100.0f, 200.0f) - 100.0f), double(std::nextafter(1e4f, 2e4f) - 1e4f),
                double(std::nextafter(1e6f, 2e6f) - 1e6f));
}

// 同一个积分器，只换数值类型：和 double 版本的距离
template <class T>
static void orbitRow(const char* name) {
    std::printf("%-10s", name);
    for (double r0 : {30.0, 60.0, 100.0}) {
        auto o = Orbit<T>::circular(r0);
        auto d = Orbit<double>::circular(r0);
        double worst = 0;
        int steps = int(10 * kTau * r0 / std::sqrt(Orbit<double>::kGM / r0));  // 约 10 圈
        for (int i = 0; i < steps; ++i) {
            o.step(), d.step();
            worst = std::max(worst, std::hypot(toD(o.x) - d.x, toD(o.y) - d.y));
        }
        std::printf(" %14.3g", worst);
    }
    std::printf("\n");
}
static void orbit() {
    std::printf("\n== orbits (GM = 100, dt = 1), worst distance from the double run over 10 revolutions ==\n");
    std::printf("%-10s %14s %14s %14s\n", "type", "r = 30", "r = 60", "r = 100");
    orbitRow<Fixed<8>>("Q23.8"), orbitRow<Fixed<12>>("Q19.12"), orbitRow<Fixed<16>>("Q15.16"), orbitRow<Fixed<20>>("Q11.20");
    orbitRow<Fixed<24>>("Q7.24"), orbitRow<float>("float");
}

// 换一种加法顺序 / 换成 fma，多少帧后有粒子差出 1 px
static void order() {
    std::printf("\n== swarm of 64 particles: frames until two runs differ by > 1 px ==\n");
    auto until = [](auto a, auto b, bool revB, bool fusedB, int limit) {
        int first = -1, frames = 0;
        for (int f = 0; f < limit; ++f, ++frames) {
            a.step(false, false), b.step(revB, fusedB);
            if (first < 0 && a.hash() != b.hash()) first = f;
            if (a.maxDistance(b) > 1) return std::pair{first, f};
        }
        return std::pair{first, -1};
    };
    for (uint64_t seed : {1, 2, 3}) {
        auto f = Swarm<float>::make(seed, 64);
        auto [h1, d1] = until(f, f, true, false, 100000);
        auto [h2, d2] = until(f, f, false, true, 100000);
        auto x = Swarm<Fx>::make(seed, 64);
        auto [h3, d3] = until(x, x, true, false, 100000);
        std::printf("seed %llu  float reversed: first bit at %d, > 1 px at %d | float fma: %d, %d | fixed reversed: %d, %d (100000 frames)\n",
                    (unsigned long long)seed, h1, d1, h2, d2, h3, d3);
    }
}

static void speed() {
    std::printf("\n== speed (best of several rounds; the machine is shared, see load) ==\n");
    for (int n : {64, 256}) {
        auto f = Swarm<float>::make(1, n);
        auto x = Swarm<Fx>::make(1, n);
        int steps = n == 64 ? 2000 : 200;
        double tf = best(5, [&] { for (int i = 0; i < steps; ++i) f.step(); }) / steps;
        double tx = best(5, [&] { for (int i = 0; i < steps; ++i) x.step(); }) / steps;
        std::printf("swarm %3d particles: float %.3f ms/step, fixed %.3f ms/step (%.2fx)\n", n, tf, tx, tx / tf);
    }
    // 依赖链：每次运算都要等上一次的结果（延迟）
    const int N = 10000000;
    {
        volatile float seed = 1.0000001f;
        float a = seed;
        double t = best(5, [&] { for (int i = 0; i < N; ++i) a = a * seed; });
        Fx b = Fx::of(double(seed) * 0 + 1.0001), m = b;  // 由 volatile 导出，编译器不能在编译期算完
        double t2 = best(5, [&] { for (int i = 0; i < N; ++i) b = b * m; });
        std::printf("mul chain:  float %.2f ns, fixed %.2f ns  (%g %d)\n", t * 1e6 / N, t2 * 1e6 / N, double(a), b.raw);
    }
    {
        volatile float seed = 0.999f;
        float a = seed;
        double t = best(5, [&] { for (int i = 0; i < N; ++i) a = 1.0f / (a + seed); });
        Fx b = Fx::of(double(seed)), one = Fx::fromInt(1), s = b;
        double t2 = best(5, [&] { for (int i = 0; i < N; ++i) b = one / (b + s); });
        std::printf("div chain:  float %.2f ns, fixed %.2f ns  (%g %d)\n", t * 1e6 / N, t2 * 1e6 / N, double(a), b.raw);
    }
    {
        volatile float seed = 2.0f;
        float a = seed;
        double t = best(5, [&] { for (int i = 0; i < N; ++i) a = std::sqrt(a + seed); });
        Fx b = Fx::of(double(seed)), s = b;
        double t2 = best(5, [&] { for (int i = 0; i < N; ++i) b = sqrt(b + s); });
        uint64_t c = uint64_t(seed) + 1;
        double t3 = best(5, [&] { for (int i = 0; i < N; ++i) c = isqrt64((c << 16) + 12345); });
        std::printf("sqrt chain: float %.2f ns, fixed %.2f ns, bit-by-bit isqrt64 %.2f ns  (%g %d %llu)\n", t * 1e6 / N, t2 * 1e6 / N,
                    t3 * 1e6 / N, double(a), b.raw, (unsigned long long)c);
    }
    {
        volatile float seed = 0.3f;
        float a = seed;
        double t = best(5, [&] { for (int i = 0; i < N; ++i) a = std::sin(a + seed); });
        Fx b = Fx::of(double(seed)), s = b;
        double t2 = best(5, [&] { for (int i = 0; i < N; ++i) b = sinTurns(b + s); });
        std::printf("sin chain:  float %.2f ns, fixed %.2f ns  (%g %d)\n", t * 1e6 / N, t2 * 1e6 / N, double(a), b.raw);
    }
}

static void platform(const char* dump) {
    std::printf("\n== platform fingerprints ==\n");
    auto f = Swarm<float>::make(1, 64);
    auto x = Swarm<Fx>::make(1, 64);
    for (int i = 0; i < 3000; ++i) f.step(), x.step();
    std::printf("swarm float, 3000 frames: %016llx\n", (unsigned long long)f.hash());
    std::printf("swarm fixed, 3000 frames: %016llx\n", (unsigned long long)x.hash());
    // 对照：6.3 的模拟，写法没有刻意避开 a*b + c
    {
        command::Recording rec{1, 40, {}};
        Pcg32 r{101};
        command::Input cur;
        for (int i = 0; i < 5000; ++i) {
            if (r.next() % 20 == 0) cur = {int8_t(int(r.next() % 3) - 1), int8_t(int(r.next() % 3) - 1), r.next() % 4 == 0};
            rec.inputs.push_back(cur);
        }
        std::printf("6.3 sim, 5000 frames:     %016llx\n", (unsigned long long)rec.replay(5000).hash());
    }
    uint64_t h = 1469598103934665603ull, hs = h;
    int notNearest = 0;
    std::vector<float> vals;
    for (int i = 0; i < 1000000; ++i) {
        float t = float(i) * 0.001f;  // 0 … 1000
        float s = std::sin(t);
        vals.push_back(s);
        if (s != float(std::sin(double(t)))) ++notNearest;
        uint32_t u;
        std::memcpy(&u, &s, 4);
        h = (h ^ u) * 1099511628211ull;
        hs = (hs ^ uint32_t(sinTurns(Fx::fromRaw(i * 7)).raw)) * 1099511628211ull;
    }
    std::printf("sinf on 1e6 inputs:       %016llx  (%d differ from (float)sin(double))\n", (unsigned long long)h, notNearest);
    std::printf("fixed sin on 1e6 inputs:  %016llx\n", (unsigned long long)hs);
    if (dump) {
        FILE* fp = std::fopen(dump, "wb");
        std::fwrite(vals.data(), 4, vals.size(), fp);
        std::fclose(fp);
    }
}

int main(int argc, char** argv) {
    auto want = [&](const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; };
    if (want("precision")) precision();
    if (want("orbit")) orbit();
    if (want("order")) order();
    if (want("speed")) speed();
    if (want("platform")) platform(argc > 2 ? argv[2] : nullptr);
}
