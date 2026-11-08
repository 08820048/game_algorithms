#pragma once

// 格点噪声：Value Noise 与 Perlin（梯度）噪声
//   - 噪声是坐标的函数：同一个种子、同一个坐标，永远得到同一个值；不需要存储，可以随机访问任意位置
//   - 两种噪声都在整数格点上放随机量，再用插值把格点之间填满：
//       Value Noise：格点上放随机的"值"，插值这些值
//       Perlin Noise：格点上放随机的"梯度"（方向），每个格点贡献 g·(p - 格点)，再插值这些贡献
//   - 插值权重用缓和函数 fade(t)：linear（折线）、smoothstep（3t² - 2t³）、quintic（6t⁵ - 15t⁴ + 10t³）
//   - 格点上的随机量来自"格点哈希"：HashLattice（整数哈希，不重复）或 PermLattice（Perlin 原版的 256 项置换表，周期 256）

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "../../foundations/random/random.h"

namespace ga::noise {

// #region fade
enum class FadeCurve { Linear, Smooth, Quintic };

// 缓和函数及其一阶、二阶导数。三者都满足 f(0) = 0、f(1) = 1；
// smoothstep 额外满足 f'(0) = f'(1) = 0，quintic 再满足 f''(0) = f''(1) = 0
template <class T>
T fade(FadeCurve f, T t) {
    switch (f) {
        case FadeCurve::Linear: return t;
        case FadeCurve::Smooth: return t * t * (3 - 2 * t);
        default: return t * t * t * (t * (t * 6 - 15) + 10);
    }
}
template <class T>
T fadeD(FadeCurve f, T t) {
    switch (f) {
        case FadeCurve::Linear: return 1;
        case FadeCurve::Smooth: return 6 * t * (1 - t);
        default: return 30 * t * t * (t - 1) * (t - 1);
    }
}
template <class T>
T fadeDD(FadeCurve f, T t) {
    switch (f) {
        case FadeCurve::Linear: return 0;
        case FadeCurve::Smooth: return 6 - 12 * t;
        default: return 60 * t * (t - 1) * (2 * t - 1);
    }
}
// #endregion fade

// #region lattice
// 32 位整数混合（MurmurHash3 的 fmix32）：输入改动一位，输出每一位翻转的概率都接近 1/2
inline uint32_t mix32(uint32_t h) {
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

inline int wrap(int x, int period) {
    int r = x % period;
    return r < 0 ? r + period : r;
}

// 格点哈希：先混合 x，再加上 y 混合一次——与置换表 p[p[x] + y] 的嵌套结构相同，但没有 256 的周期。
// period > 0 时坐标先取模，噪声以 period 为周期无缝平铺（适合做贴图）
struct HashLattice {
    uint32_t seed = 0;
    int period = 0;

    uint32_t hash(int x, int y) const {
        if (period > 0) x = wrap(x, period), y = wrap(y, period);
        return mix32(uint32_t(y) + mix32(uint32_t(x) ^ mix32(seed)));
    }
    // 格点上的随机值，均匀分布在 [-1, 1]
    template <class T = float>
    T value(int x, int y) const {
        return T(hash(x, y) >> 8) * T(2.0 / 16777215.0) - 1;
    }
    // 格点上的梯度编号 0..7，用低 3 位
    int gradient(int x, int y) const { return int(hash(x, y) & 7); }
};

// Perlin 原版的做法：0..255 的随机排列，查两次表。坐标只取低 8 位，所以噪声每 256 个单位重复一次
struct PermLattice {
    std::array<uint8_t, 512> p{};

    explicit PermLattice(uint64_t seed = 0) {
        Pcg32 rng(seed);
        for (int i = 0; i < 256; ++i) p[i] = uint8_t(i);
        for (int i = 255; i > 0; --i) std::swap(p[i], p[rng.below(uint32_t(i + 1))]);
        for (int i = 0; i < 256; ++i) p[256 + i] = p[i];
    }
    uint32_t hash(int x, int y) const { return p[p[x & 255] + (y & 255)]; }
    template <class T = float>
    T value(int x, int y) const {
        return T(hash(x, y)) * T(2.0 / 255.0) - 1;
    }
    int gradient(int x, int y) const { return int(hash(x, y) & 7); }
};

// 8 个单位梯度：四个轴向、四个对角
template <class T>
inline void gradientVector(int k, T& gx, T& gy) {
    constexpr double s = 0.70710678118654752;
    constexpr double kG[8][2] = {{1, 0}, {s, s}, {0, 1}, {-s, s}, {-1, 0}, {-s, -s}, {0, -1}, {s, -s}};
    gx = T(kG[k][0]), gy = T(kG[k][1]);
}
// #endregion lattice

template <class T>
struct Sample {
    T value, dx, dy;  // 值与梯度（解析导数）
};

// #region value
template <class T, class Lattice>
T valueNoise(const Lattice& lat, T x, T y, FadeCurve f = FadeCurve::Quintic) {
    T x0 = std::floor(x), y0 = std::floor(y);
    int ix = int(x0), iy = int(y0);
    T u = fade(f, x - x0), v = fade(f, y - y0);
    T a = lat.template value<T>(ix, iy), b = lat.template value<T>(ix + 1, iy);
    T c = lat.template value<T>(ix, iy + 1), d = lat.template value<T>(ix + 1, iy + 1);
    // 双线性插值：先沿 x 插两次，再沿 y 插一次
    T top = a + (b - a) * u, bottom = c + (d - c) * u;
    return top + (bottom - top) * v;
}
// #endregion value

// #region perlin
template <class T, class Lattice>
T perlinNoise(const Lattice& lat, T x, T y, FadeCurve f = FadeCurve::Quintic) {
    T x0 = std::floor(x), y0 = std::floor(y);
    int ix = int(x0), iy = int(y0);
    T fx = x - x0, fy = y - y0;  // 在格子里的位置，[0, 1)
    // 每个角的贡献：该角的梯度 · 从该角指向采样点的向量
    auto corner = [&](int cx, int cy, T dx, T dy) {
        T gx, gy;
        gradientVector(lat.gradient(cx, cy), gx, gy);
        return gx * dx + gy * dy;
    };
    T n00 = corner(ix, iy, fx, fy), n10 = corner(ix + 1, iy, fx - 1, fy);
    T n01 = corner(ix, iy + 1, fx, fy - 1), n11 = corner(ix + 1, iy + 1, fx - 1, fy - 1);
    T u = fade(f, fx), v = fade(f, fy);
    T top = n00 + (n10 - n00) * u, bottom = n01 + (n11 - n01) * u;
    return top + (bottom - top) * v;
}
// #endregion perlin

// #region derivatives
// 同时求值和解析梯度：地形法线、坡度、光照都要用到。
// 把插值展开成 k0 + k1·u + k2·v + k3·u·v，再对 x、y 求导
template <class T, class Lattice>
Sample<T> valueNoiseD(const Lattice& lat, T x, T y, FadeCurve f = FadeCurve::Quintic) {
    T x0 = std::floor(x), y0 = std::floor(y);
    int ix = int(x0), iy = int(y0);
    T tx = x - x0, ty = y - y0;
    T u = fade(f, tx), v = fade(f, ty), du = fadeD(f, tx), dv = fadeD(f, ty);
    T a = lat.template value<T>(ix, iy), b = lat.template value<T>(ix + 1, iy);
    T c = lat.template value<T>(ix, iy + 1), d = lat.template value<T>(ix + 1, iy + 1);
    T k1 = b - a, k2 = c - a, k3 = a - b - c + d;
    return {a + k1 * u + k2 * v + k3 * u * v, du * (k1 + k3 * v), dv * (k2 + k3 * u)};
}

template <class T, class Lattice>
Sample<T> perlinNoiseD(const Lattice& lat, T x, T y, FadeCurve f = FadeCurve::Quintic) {
    T x0 = std::floor(x), y0 = std::floor(y);
    int ix = int(x0), iy = int(y0);
    T fx = x - x0, fy = y - y0;
    T g[4][2];
    gradientVector(lat.gradient(ix, iy), g[0][0], g[0][1]);
    gradientVector(lat.gradient(ix + 1, iy), g[1][0], g[1][1]);
    gradientVector(lat.gradient(ix, iy + 1), g[2][0], g[2][1]);
    gradientVector(lat.gradient(ix + 1, iy + 1), g[3][0], g[3][1]);
    T n00 = g[0][0] * fx + g[0][1] * fy, n10 = g[1][0] * (fx - 1) + g[1][1] * fy;
    T n01 = g[2][0] * fx + g[2][1] * (fy - 1), n11 = g[3][0] * (fx - 1) + g[3][1] * (fy - 1);
    T u = fade(f, fx), v = fade(f, fy), du = fadeD(f, fx), dv = fadeD(f, fy);
    T k1 = n10 - n00, k2 = n01 - n00, k3 = n00 - n10 - n01 + n11;
    T value = n00 + k1 * u + k2 * v + k3 * u * v;
    // 两部分：梯度本身被插值（每个角的 n 对坐标的导数就是它的梯度），加上权重 u、v 变化带来的项
    T dx = g[0][0] + u * (g[1][0] - g[0][0]) + v * (g[2][0] - g[0][0]) + u * v * (g[0][0] - g[1][0] - g[2][0] + g[3][0]) + du * (k1 + k3 * v);
    T dy = g[0][1] + u * (g[1][1] - g[0][1]) + v * (g[2][1] - g[0][1]) + u * v * (g[0][1] - g[1][1] - g[2][1] + g[3][1]) + dv * (k2 + k3 * u);
    return {value, dx, dy};
}
// #endregion derivatives

// #region one-d
// 一维版本（图 4.1a 用）：a、b 是两端格点上的值；g0、g1 是两端的斜率，t ∈ [0, 1]
template <class T>
T valueSegment(T a, T b, T t, FadeCurve f) {
    return a + (b - a) * fade(f, t);
}
template <class T>
T gradientSegment(T g0, T g1, T t, FadeCurve f) {
    T n0 = g0 * t, n1 = g1 * (t - 1);  // 左端的直线、右端的直线
    return n0 + (n1 - n0) * fade(f, t);
}
// #endregion one-d

}  // namespace ga::noise
