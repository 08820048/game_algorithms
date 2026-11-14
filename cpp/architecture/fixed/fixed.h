#pragma once

// 定点数与确定性计算
//   - 浮点运算本身是确定的（IEEE 754 规定了 + − × ÷ √ 的舍入），不确定的是"同一行源代码会编译成哪些运算"：
//     编译器可能把 a*b + c 合并成一条 FMA（只舍入一次）、-ffast-math 会重排加法、sin/cos 由各平台的数学库实现，
//     多线程归约的加法顺序也不固定。帧同步、录像回放要求不同机器上逐位相同，这些差别经过混沌放大后就是不同步
//   - 定点数：用整数表示小数，Fixed<F> 的值 = raw / 2^F。整数运算在所有平台上结果相同，加法满足结合律，
//     没有编译器能"优化"掉的舍入差别。代价是范围和精度要自己规划：F 越大越精细，但能表示的范围越小
//   - 乘法要用 64 位中间结果，再右移 F 位；除法先左移 F 位再除。√ 用逐位求整数平方根，sin/cos 用整数多项式，
//     不依赖任何数学库

#include <cmath>
#include <compare>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

#include "../../foundations/random/random.h"

namespace ga::fixed {

// #region fixed
// Q(31−F).F：32 位有符号整数，低 F 位是小数。分辨率 2^−F，范围 ±2^(31−F)
template <int F>
struct Fixed {
    static_assert(F > 0 && F < 31);
    static constexpr int kFrac = F;
    int32_t raw = 0;

    static constexpr Fixed fromRaw(int32_t r) { return Fixed{r}; }
    static constexpr Fixed fromInt(int i) { return Fixed{int32_t(uint32_t(i) << F)}; }
    // 只用于常量和初始化：double 乘 2 的幂是精确的，llround 在所有平台上一样
    static Fixed of(double d) { return Fixed{int32_t(std::llround(d * double(int64_t(1) << F)))}; }
    double toDouble() const { return double(raw) / double(int64_t(1) << F); }

    // 加减：按无符号整数做，溢出时回绕（有符号溢出在 C++ 里是未定义行为）
    friend constexpr Fixed operator+(Fixed a, Fixed b) { return Fixed{int32_t(uint32_t(a.raw) + uint32_t(b.raw))}; }
    friend constexpr Fixed operator-(Fixed a, Fixed b) { return Fixed{int32_t(uint32_t(a.raw) - uint32_t(b.raw))}; }
    friend constexpr Fixed operator-(Fixed a) { return Fixed{int32_t(0u - uint32_t(a.raw))}; }
    // 乘：64 位乘积有 2F 位小数，加上半个单位再右移 F 位 = 四舍五入
    friend constexpr Fixed operator*(Fixed a, Fixed b) {
        int64_t p = int64_t(a.raw) * b.raw;
        return Fixed{int32_t((p + (int64_t(1) << (F - 1))) >> F)};
    }
    // 除：被除数左移 F 位再做整数除法，向最近的值舍入。除以 0 返回最大或最小值
    friend constexpr Fixed operator/(Fixed a, Fixed b) {
        if (b.raw == 0) return Fixed{a.raw >= 0 ? INT32_MAX : INT32_MIN};
        int64_t n = int64_t(a.raw) * (int64_t(1) << F), d = b.raw;
        int64_t half = (d < 0 ? -d : d) / 2;
        int64_t q = (n >= 0 ? n + half : n - half) / d;  // 整数除法向 0 截断，先把 |n| 加大半个除数
        return Fixed{int32_t(q)};
    }
    Fixed& operator+=(Fixed b) { return *this = *this + b; }
    Fixed& operator-=(Fixed b) { return *this = *this - b; }
    Fixed& operator*=(Fixed b) { return *this = *this * b; }
    friend constexpr auto operator<=>(Fixed, Fixed) = default;
};
// #endregion

using Fx = Fixed<16>;

// #region sqrt
// 64 位整数平方根（向下取整），逐位确定结果：每次试一位，平方不超过 n 就保留
inline uint32_t isqrt64(uint64_t n) {
    uint64_t r = 0, bit = uint64_t(1) << 62;
    while (bit > n) bit >>= 2;
    while (bit) {
        if (n >= r + bit) n -= r + bit, r = (r >> 1) + bit;
        else r >>= 1;
        bit >>= 2;
    }
    return uint32_t(r);
}
// 更快的做法：IEEE 754 规定 √ 必须正确舍入，所以硬件的 double √ 在所有平台上结果相同，可以放心用来猜。
// 只会猜大、不会猜小：double(n) 的相对误差 ≤ 2^−53，开方后 ≤ 2^−54，比 r 附近 double 的间隔还小，
// 所以 n ≥ k² 时舍入后一定 ≥ k；但 n = k² − 1 时可能舍入成 k。向下修正一次就够了
inline uint32_t isqrt64Fast(uint64_t n) {
    uint64_t r = uint64_t(std::sqrt(double(n)));
    if (r > 0xffffffffu) r = 0xffffffffu;
    while (r * r > n) --r;
    return uint32_t(r);
}
// √x：raw · 2^F 的整数平方根正好是结果的 raw（向下取整）。负数返回 0
template <int F>
Fixed<F> sqrt(Fixed<F> x) {
    if (x.raw <= 0) return {};
    return Fixed<F>::fromRaw(int32_t(isqrt64Fast(uint64_t(x.raw) << F)));
}
// #endregion

// #region sin
// sin/cos，角度以"圈"为单位（1.0 = 360°）。转成 32 位相位后，整圈的回绕由无符号溢出自动完成。
// 前两位是象限，余下的是象限内的位置 z ∈ [0, 1)，sin(πz/2) 用到 z^11 的泰勒多项式，全部是 Q30 整数运算
inline int64_t sinQuarterQ30(int64_t z) {  // z: Q30，0 … 2^30；返回 sin(πz/2)，Q30
    constexpr double h = 1.5707963267948966;  // π/2
    constexpr int64_t one = int64_t(1) << 30;
    // 系数在编译期由 double 常量折叠成整数，所有平台相同
    constexpr int64_t c[6] = {int64_t(h * one + 0.5),
                              -int64_t(h * h * h / 6 * one + 0.5),
                              int64_t(h * h * h * h * h / 120 * one + 0.5),
                              -int64_t(h * h * h * h * h * h * h / 5040 * one + 0.5),
                              int64_t(h * h * h * h * h * h * h * h * h / 362880 * one + 0.5),
                              -int64_t(h * h * h * h * h * h * h * h * h * h * h / 39916800 * one + 0.5)};
    int64_t z2 = (z * z) >> 30, acc = c[5];
    for (int k = 4; k >= 0; --k) acc = c[k] + ((acc * z2) >> 30);
    return (acc * z) >> 30;
}
template <int F>
Fixed<F> sinTurns(Fixed<F> turns) {
    uint32_t phase = uint32_t(turns.raw) << (32 - F);  // 1 圈 = 2^32
    uint32_t quadrant = phase >> 30;
    int64_t z = phase & ((1u << 30) - 1);
    if (quadrant & 1) z = (int64_t(1) << 30) - z;  // 第二、四象限：sin(π/2 − x)
    int64_t s = sinQuarterQ30(z);
    int64_t r = (s + (int64_t(1) << (29 - F))) >> (30 - F);  // Q30 → QF，四舍五入（对绝对值舍入，正负对称）
    return Fixed<F>::fromRaw(int32_t(quadrant & 2 ? -r : r));
}
template <int F>
Fixed<F> cosTurns(Fixed<F> turns) {
    return sinTurns(turns + Fixed<F>::fromRaw(int32_t(1) << (F - 2)));  // 加 1/4 圈
}
// #endregion

// 通用的数值辅助：同一份模拟代码既能用 float/double 也能用 Fixed
template <class T>
inline constexpr bool isFloat = std::is_floating_point_v<T>;

template <class T>
T num(double d) {
    if constexpr (isFloat<T>) return T(d);
    else return T::of(d);
}
// n / 256：初始状态从整数生成，float 和 Fixed（F ≥ 8）都能精确表示，两种模拟从完全相同的值出发
template <class T>
T ratio256(int n) {
    if constexpr (isFloat<T>) return T(n) / T(256);
    else {
        static_assert(T::kFrac >= 8);
        return T::fromRaw(int32_t(uint32_t(n) << (T::kFrac - 8)));
    }
}
template <class T>
uint64_t bitsOf(T v) {
    if constexpr (std::is_same_v<T, float>) {
        uint32_t u;
        std::memcpy(&u, &v, 4);
        return u;
    } else if constexpr (std::is_same_v<T, double>) {
        uint64_t u;
        std::memcpy(&u, &v, 8);
        return u;
    } else {
        return uint32_t(v.raw);
    }
}
template <class T>
T sqrtOf(T v) {
    if constexpr (isFloat<T>) return std::sqrt(v);
    else return sqrt(v);
}
template <class T>
double toD(T v) {
    if constexpr (isFloat<T>) return double(v);
    else return v.toDouble();
}


// #region swarm
// 互相排斥的粒子群（混沌）：每个粒子对半径 R 内的邻居求排斥力之和。
//   reverse：按相反的顺序累加（相当于换一种多线程归约方式）
//   fused：  每一项用 fma(s, u, acc) 累加（只舍入一次）；否则先乘、单独一条语句再加（舍入两次）
// 同一份代码分别实例化成 float 和 Fixed
template <class T>
struct Swarm {
    static constexpr int kW = 560, kH = 360;
    struct P {
        T x, y, vx, vy;
    };
    std::vector<P> ps;
    uint32_t frame = 0;

    static Swarm make(uint64_t seed, int count) {
        Swarm s;
        Pcg32 r{seed};
        for (int i = 0; i < count; ++i) {
            int x = int(r.next() % (kW * 256)), y = int(r.next() % (kH * 128));  // 上半部分
            int vx = int(r.next() % 1024) - 512, vy = int(r.next() % 1024) - 512;  // ±2 px/帧
            s.ps.push_back({ratio256<T>(x), ratio256<T>(y), ratio256<T>(vx), ratio256<T>(vy)});
        }
        return s;
    }

    static T mad(T a, T b, T c, bool fused) {
        if constexpr (isFloat<T>) {
            if (fused) return std::fma(a, b, c);
        }
        T p = a * b;  // 单独一条语句：编译器的默认设置（-ffp-contract=on）不会跨语句合并
        return p + c;
    }

    void step(bool reverse = false, bool fused = false) {
        const T invR = num<T>(1.0 / 28), k = num<T>(0.25), g = num<T>(0.05), one = num<T>(1), zero{};
        const T w = num<T>(kW), h = num<T>(kH);
        const int n = int(ps.size());
        std::vector<T> ax(ps.size()), ay(ps.size());
        for (int i = 0; i < n; ++i) {
            T sx{}, sy{};
            for (int m = 0; m < n; ++m) {
                int j = reverse ? n - 1 - m : m;
                if (j == i) continue;
                T ux = (ps[size_t(i)].x - ps[size_t(j)].x) * invR, uy = (ps[size_t(i)].y - ps[size_t(j)].y) * invR;
                if (ux >= one || ux <= -one || uy >= one || uy <= -one) continue;
                T d2 = ux * ux;
                d2 = mad(uy, uy, d2, false);
                T q = one - d2;
                if (q <= zero) continue;
                T s = k * q;
                sx = mad(s, ux, sx, fused), sy = mad(s, uy, sy, fused);
            }
            ax[size_t(i)] = sx, ay[size_t(i)] = sy;
        }
        for (int i = 0; i < n; ++i) {
            P& p = ps[size_t(i)];
            p.vx = p.vx + ax[size_t(i)];
            p.vy = p.vy + ay[size_t(i)] + g;
            p.x = p.x + p.vx, p.y = p.y + p.vy;
            // 撞墙：位置镜像，速度反向（都是精确运算）
            if (p.x < zero) p.x = -p.x, p.vx = -p.vx;
            if (p.x > w) p.x = w + w - p.x, p.vx = -p.vx;
            if (p.y < zero) p.y = -p.y, p.vy = -p.vy;
            if (p.y > h) p.y = h + h - p.y, p.vy = -p.vy;
        }
        ++frame;
    }

    uint64_t hash() const {
        uint64_t x = 1469598103934665603ull;
        auto mix = [&](uint64_t v) { x = (x ^ v) * 1099511628211ull; };
        mix(frame);
        for (const P& p : ps) mix(bitsOf(p.x)), mix(bitsOf(p.y)), mix(bitsOf(p.vx)), mix(bitsOf(p.vy));
        return x;
    }
    // 两局里相差最远的一个粒子
    template <class U>
    double maxDistance(const Swarm<U>& o) const {
        double m = 0;
        for (size_t i = 0; i < ps.size(); ++i) {
            double dx = toD(ps[i].x) - toD(o.ps[i].x), dy = toD(ps[i].y) - toD(o.ps[i].y);
            m = std::max(m, std::sqrt(dx * dx + dy * dy));
        }
        return m;
    }
};
// #endregion

// #region orbit
// 行星绕恒星（原点）运动，引力 a = −GM · p / r³，半隐式欧拉，每步 dt = 1。
// 写成 a = −(GM / r²) · (p / r)：中间值都是 1 左右，定点数也能算（r³ 在半径 100 时是 10^6，Q16.16 放不下）
template <class T>
struct Orbit {
    static constexpr double kGM = 100;
    T x, y, vx, vy;

    static Orbit circular(double radius) {
        return {num<T>(radius), num<T>(0), num<T>(0), num<T>(std::sqrt(kGM / radius))};
    }
    void step() {
        T r2 = x * x + y * y;
        T r = sqrtOf(r2);
        T s = num<T>(kGM) / r2;
        vx = vx - s * (x / r);
        vy = vy - s * (y / r);
        x = x + vx, y = y + vy;
    }
    double radius() const { return std::sqrt(toD(x) * toD(x) + toD(y) * toD(y)); }
};
// #endregion

}  // namespace ga::fixed
