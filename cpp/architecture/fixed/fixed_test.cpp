#include "fixed.h"

#include <cmath>
#include <cstdio>

#include "check.h"

using namespace ga;
using namespace ga::fixed;
using i128 = __int128;

// 乘法：结果与精确乘积的差不超过半个单位（用 128 位整数做参照）
template <int F>
static void testMul() {
    Pcg32 r{uint64_t(F)};
    for (int t = 0; t < 200000; ++t) {
        // 两个因子的大小随机，乘积不超出范围
        int ea = int(r.next() % 31), eb = int(r.next() % 31);
        if (ea + eb - F > 29) continue;
        int32_t a = int32_t(r.next() & ((1u << ea) - 1)), b = int32_t(r.next() & ((1u << eb) - 1));
        if (r.next() & 1) a = -a;
        if (r.next() & 1) b = -b;
        Fixed<F> p = Fixed<F>::fromRaw(a) * Fixed<F>::fromRaw(b);
        i128 exact = i128(a) * b, err = i128(p.raw) * (i128(1) << F) - exact;
        CHECK(err > -(i128(1) << (F - 1)) && err <= (i128(1) << (F - 1)));
    }
    CHECK((Fixed<F>::fromInt(3) * Fixed<F>::fromInt(-2)) == Fixed<F>::fromInt(-6));
    CHECK((Fixed<F>::of(0.5) * Fixed<F>::of(0.5)) == Fixed<F>::of(0.25));
}

// 除法：|q·b − a·2^F| ≤ |b| / 2，即向最近的值舍入
template <int F>
static void testDiv() {
    Pcg32 r{uint64_t(F) + 50};
    for (int t = 0; t < 200000; ++t) {
        int32_t a = int32_t(r.next() >> (r.next() % 32)), b = int32_t(r.next() >> (r.next() % 32));
        if (r.next() & 1) a = -a;
        if (r.next() & 1) b = -b;
        if (b == 0) continue;
        i128 n = i128(a) * (i128(1) << F);
        i128 exactQ = n / b;
        if (exactQ > INT32_MAX - 1 || exactQ < INT32_MIN + 1) continue;  // 商超出范围
        Fixed<F> q = Fixed<F>::fromRaw(a) / Fixed<F>::fromRaw(b);
        i128 err = i128(q.raw) * b - n, bound = b < 0 ? -i128(b) : i128(b);
        CHECK(2 * (err < 0 ? -err : err) <= bound);
    }
    CHECK((Fixed<F>::fromInt(7) / Fixed<F>::fromInt(-2)) == Fixed<F>::of(-3.5));
    CHECK((Fixed<F>::fromInt(1) / Fixed<F>{}).raw == INT32_MAX);
    CHECK((Fixed<F>::fromInt(-1) / Fixed<F>{}).raw == INT32_MIN);
}

// 平方根：r² ≤ x·2^F < (r+1)²
template <int F>
static void testSqrt() {
    Pcg32 r{uint64_t(F) + 99};
    auto ok = [](int32_t raw) {
        uint64_t n = uint64_t(raw) << F, s = uint64_t(sqrt(Fixed<F>::fromRaw(raw)).raw);
        return s * s <= n && (s + 1) * (s + 1) > n;
    };
    for (int32_t raw : {1, 2, 3, 4, 1 << F, INT32_MAX, INT32_MAX - 1}) CHECK(ok(raw));
    for (int t = 0; t < 200000; ++t) CHECK(ok(int32_t(r.next() >> (1 + r.next() % 31)) | 1));
    CHECK(sqrt(Fixed<F>::fromInt(-4)).raw == 0);
    CHECK(sqrt(Fixed<F>::fromInt(9)) == Fixed<F>::fromInt(3));
    // isqrt64 在整个 64 位范围内正确
    for (int t = 0; t < 100000; ++t) {
        uint64_t n = (uint64_t(r.next()) << 32 | r.next()) >> (r.next() % 64);
        uint64_t s = isqrt64(n);
        CHECK(i128(s) * s <= n && i128(s + 1) * (s + 1) > n);
        CHECK(isqrt64Fast(n) == s);
    }
    // 边界：完全平方数和它的前一个数、64 位的最大值
    for (uint64_t k : {1ull, 2ull, 3037000499ull, 4294967295ull}) {
        CHECK(isqrt64Fast(k * k) == k && isqrt64(k * k) == k);
        CHECK(isqrt64Fast(k * k - 1) == k - 1 && isqrt64(k * k - 1) == k - 1);
    }
    CHECK(isqrt64Fast(~0ull) == 4294967295u && isqrt64(~0ull) == 4294967295u);
    // 完全平方数附近最容易出错（double 放不下 k² 时会被舍入）：猜测不能偏小，也不能偏大
    for (int t = 0; t < 200000; ++t) {
        uint64_t k = r.next() | (uint64_t(1) << 26);  // 2^26 … 2^32：k² 超出 double 的 53 位
        CHECK(isqrt64Fast(k * k) == k);
        CHECK(isqrt64Fast(k * k - 1) == k - 1);
        if (k < 0xffffffffu) CHECK(isqrt64Fast(k * k + 2 * k) == k);  // (k+1)² − 1
    }
}

// sin/cos：Q16 的每一个角度（65536 个）都和 double 的 sin 比较
static void testSin() {
    double worst = 0, worstPyth = 0;
    for (int32_t raw = -65536; raw < 65536 * 2; ++raw) {
        Fx t = Fx::fromRaw(raw);
        Fx s = sinTurns(t), c = cosTurns(t);
        double ref = std::sin(t.toDouble() * 2 * 3.14159265358979323846);
        worst = std::max(worst, std::fabs(s.toDouble() - ref));
        worstPyth = std::max(worstPyth, std::fabs(s.toDouble() * s.toDouble() + c.toDouble() * c.toDouble() - 1));
        CHECK(sinTurns(-t) == -s);                         // 奇函数，逐位成立
        CHECK(sinTurns(t + Fx::fromInt(1)) == s);          // 周期为一圈
    }
    CHECK(worst <= 1.0 / 65536);  // 不超过 1 个单位（舍入本身就有半个单位）
    CHECK(worstPyth <= 3.0 / 65536);
    CHECK(sinTurns(Fx{}) == Fx{});
    CHECK(sinTurns(Fx::of(0.25)) == Fx::fromInt(1));
    CHECK(sinTurns(Fx::of(0.5)) == Fx{});
    CHECK(sinTurns(Fx::of(0.75)) == Fx::fromInt(-1));
    CHECK(cosTurns(Fx{}) == Fx::fromInt(1));
    // 其他精度也能用（F = 24 时多项式误差约 6e-8，接近 1 个单位）
    for (int i = 0; i < 4096; ++i) {
        Fixed<24> t = Fixed<24>::fromRaw(i << 12);
        CHECK(std::fabs(sinTurns(t).toDouble() - std::sin(t.toDouble() * 2 * 3.14159265358979323846)) < 2e-7);
    }
}

// 加减按 2^32 回绕，不是未定义行为；常量转换
static void testBasics() {
    Fx mx = Fx::fromRaw(INT32_MAX), ulp = Fx::fromRaw(1);
    CHECK((mx + ulp).raw == INT32_MIN);
    CHECK((Fx::fromRaw(INT32_MIN) - ulp).raw == INT32_MAX);
    CHECK(Fx::fromInt(5).raw == 5 << 16);
    CHECK(Fx::fromInt(-5).raw == -(5 << 16));
    CHECK(Fx::of(1.5).toDouble() == 1.5);
    CHECK(Fx::of(-0.25).raw == -(1 << 14));
    CHECK(Fx::of(1.0 / 3).raw == 21845);
    CHECK(Fx::of(1e-6).raw == 0);  // 小于半个单位
    // float 和 Fixed 的初始状态完全相同
    for (int n : {0, 1, -1, 255, 256, 143359, -511}) CHECK(toD(ratio256<float>(n)) == toD(ratio256<Fx>(n)));
}

// 粒子群：定点数换累加顺序结果逐位相同；float 换顺序或换成 fma 都会不同
static void testSwarm() {
    for (uint64_t seed : {1, 2, 3}) {
        auto a = Swarm<Fx>::make(seed, 64), b = a;
        for (int f = 0; f < 2000; ++f) a.step(false), b.step(true);
        CHECK(a.hash() == b.hash());
        auto c = Swarm<float>::make(seed, 64), d = c, e = c;
        for (int f = 0; f < 2000; ++f) c.step(false, false), d.step(true, false), e.step(false, true);
        CHECK(c.hash() != d.hash());
        CHECK(c.hash() != e.hash());
        CHECK(c.maxDistance(d) > 10);  // 不只是最后一位不同：混沌把差别放大到肉眼可见
        // 两种类型从同一个状态出发
        auto f0 = Swarm<float>::make(seed, 64);
        CHECK(Swarm<Fx>::make(seed, 64).maxDistance(f0) == 0);
    }
    // 黄金值：在任何平台、任何编译选项下都应该得到这个数（浏览器里的演示也显示它）
    auto g = Swarm<Fx>::make(1, 64);
    for (int f = 0; f < 3000; ++f) g.step();
    CHECK(g.hash() == 0xefaf785213611901ull);
    CHECK(g.frame == 3000);
    // 粒子不会飞出盒子
    for (auto& p : g.ps) CHECK(p.x >= Fx{} && p.x <= Fx::fromInt(560) && p.y >= Fx{} && p.y <= Fx::fromInt(360));
}

// 轨道：Q16.16 在半径 100 内误差很小；Q8.24 的 r² 溢出，轨道完全错误
static void testOrbit() {
    auto o = Orbit<Fx>::circular(60);
    auto d = Orbit<double>::circular(60);
    for (int i = 0; i < 20000; ++i) o.step(), d.step();
    CHECK(std::fabs(o.radius() - d.radius()) < 1.0);
    // 半径不是好的判据：乱跳的轨道某一刻的半径也可能碰巧接近 100。比较和 double 参照的位置
    auto big = Orbit<Fixed<24>>::circular(100);
    auto ref = Orbit<double>::circular(100);
    for (int i = 0; i < 200; ++i) big.step(), ref.step();
    CHECK(std::hypot(big.x.toDouble() - ref.x, big.y.toDouble() - ref.y) > 30);
    // 只差在 r 能表示、r² 不能：F = 20 时 r = 30 正常，r = 100 溢出（100² > 2048）
    auto small20 = Orbit<Fixed<20>>::circular(30);
    auto ref30 = Orbit<Fixed<16>>::circular(30);
    auto big20 = Orbit<Fixed<20>>::circular(100);
    for (int i = 0; i < 500; ++i) small20.step(), ref30.step(), big20.step();
    CHECK(std::fabs(small20.radius() - ref30.radius()) < 0.5);
    CHECK(std::fabs(big20.radius() - 100) > 10);
}

int main() {
    testBasics();
    testMul<8>(), testMul<16>(), testMul<24>();
    testDiv<8>(), testDiv<16>(), testDiv<24>();
    testSqrt<8>(), testSqrt<16>(), testSqrt<24>();
    testSin();
    testSwarm();
    testOrbit();
    return report("fixed_test");
}
