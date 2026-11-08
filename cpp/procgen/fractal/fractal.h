#pragma once

// Simplex 噪声、Worley（蜂窝）噪声与分形布朗运动
//   - Simplex：把平面切成三角形，每个点只受所在三角形 3 个角的影响；每个角的贡献 (r0² - |d|²)⁴ · (g·d) 随距离衰减到 0，
//     不需要沿坐标轴插值，所以没有正方形格子的方向性
//   - Worley：每个格子里放一个随机的特征点，求采样点到最近（F1）、第二近（F2）特征点的距离
//   - fBm：把频率逐层翻倍、振幅逐层减半的多层噪声加起来，得到"大起伏上叠着小起伏"的自相似结构

#include <cmath>
#include <cstdint>

#include "../noise/noise.h"

namespace ga::noise {

// #region simplex
// 斜切：把正方形格子沿对角线压扁，得到等边三角形网格
constexpr double kSkew = 0.36602540378443865;    // F2 = (√3 - 1) / 2
constexpr double kUnskew = 0.21132486540518713;  // G2 = (3 - √3) / 6
// 让输出的最大值为 1 的缩放系数：由 noise_bench 的 simplex 节数值搜索得到，测试中穷举验证
constexpr double kSimplexScale = 99.204334582718712;

// r2 是每个角的影响半径的平方。0.5 正好是三角形的高的平方：再大，三角形外的第 4 个角也该有贡献，结果会出现断层
template <class T, class Lattice>
Sample<T> simplexNoiseD(const Lattice& lat, T x, T y, T r2 = T(0.5)) {
    T s = (x + y) * T(kSkew);
    T fi = std::floor(x + s), fj = std::floor(y + s);
    int i = int(fi), j = int(fj);
    T t = (fi + fj) * T(kUnskew);
    T x0 = x - (fi - t), y0 = y - (fj - t);  // 相对第一个角（斜切格子的左下角）
    int i1 = x0 > y0 ? 1 : 0, j1 = 1 - i1;   // 下三角形还是上三角形：决定第二个角
    T dx[3] = {x0, x0 - i1 + T(kUnskew), x0 - 1 + 2 * T(kUnskew)};
    T dy[3] = {y0, y0 - j1 + T(kUnskew), y0 - 1 + 2 * T(kUnskew)};
    int ci[3] = {i, i + i1, i + 1}, cj[3] = {j, j + j1, j + 1};
    Sample<T> out{0, 0, 0};
    for (int k = 0; k < 3; ++k) {
        // 半径外的角贡献为 0。用 max 而不是 if 跳过：是否跳过几乎随机，分支预测会频繁失败
        T a = std::max(T(0), r2 - dx[k] * dx[k] - dy[k] * dy[k]);
        T gx, gy;
        gradientVector(lat.gradient(ci[k], cj[k]), gx, gy);
        T dot = gx * dx[k] + gy * dy[k], a2 = a * a, a4 = a2 * a2;
        out.value += a4 * dot;
        // d/dp [a⁴ (g·d)] = a⁴ g + 4a³ (g·d) · (-2d)
        out.dx += a4 * gx - 8 * a2 * a * dot * dx[k];
        out.dy += a4 * gy - 8 * a2 * a * dot * dy[k];
    }
    const T sc = T(kSimplexScale);
    return {out.value * sc, out.dx * sc, out.dy * sc};
}

template <class T, class Lattice>
T simplexNoise(const Lattice& lat, T x, T y) {
    return simplexNoiseD(lat, x, y).value;
}
// #endregion simplex

// #region worley
// 格子 (i, j) 里的特征点：位置由哈希决定，均匀分布在格子内
template <class T, class Lattice>
void featurePoint(const Lattice& lat, int i, int j, T& px, T& py) {
    uint32_t h = lat.hash(i, j), h2 = mix32(h ^ 0x9e3779b9u);
    px = i + T(h >> 8) * T(1.0 / 16777216.0);
    py = j + T(h2 >> 8) * T(1.0 / 16777216.0);
}

template <class T>
struct CellSample {
    T f1 = 1e30, f2 = 1e30;  // 到最近、第二近特征点的距离
    int cellX = 0, cellY = 0;  // 最近的特征点所在的格子（可以当作 Voronoi 区域的编号）
    int cell2X = 0, cell2Y = 0;  // 第二近的特征点所在的格子
    int searched = 0;          // 检查了多少个格子
};

// searchRadius = 1：只查周围 3×3 个格子（常见写法，偶尔会出错）；
// searchRadius = 0：精确搜索，一圈一圈向外，直到外圈不可能比当前的 F2 更近
template <class T, class Lattice>
CellSample<T> worley(const Lattice& lat, T x, T y, int searchRadius = 0) {
    T fx = std::floor(x), fy = std::floor(y);
    int ix = int(fx), iy = int(fy);
    CellSample<T> out;
    // 先比较距离的平方，最后再开方（std::hypot 为了防溢出写得很慢）
    auto visit = [&](int i, int j) {
        T px, py;
        featurePoint(lat, i, j, px, py);
        T d = (px - x) * (px - x) + (py - y) * (py - y);
        ++out.searched;
        if (d < out.f1) out.f2 = out.f1, out.cell2X = out.cellX, out.cell2Y = out.cellY, out.f1 = d, out.cellX = i, out.cellY = j;
        else if (d < out.f2) out.f2 = d, out.cell2X = i, out.cell2Y = j;
    };
    // 采样点到所在格子边界的最短距离：第 r 圈的格子离采样点至少 (r - 1) + edge
    T edge = std::min(std::min(x - fx, 1 - (x - fx)), std::min(y - fy, 1 - (y - fy)));
    for (int r = 0;; ++r) {
        if (searchRadius > 0 && r > searchRadius) break;
        if (searchRadius == 0 && r >= 2 && (T(r - 1) + edge) * (T(r - 1) + edge) >= out.f2) break;
        if (r == 0) {
            visit(ix, iy);
            continue;
        }
        for (int k = -r; k < r; ++k) {  // 第 r 圈：四条边，每条 2r 个格子
            visit(ix + k, iy - r), visit(ix + r, iy + k);
            visit(ix - k, iy + r), visit(ix - r, iy - k);
        }
    }
    out.f1 = std::sqrt(out.f1), out.f2 = std::sqrt(out.f2);
    return out;
}
// #endregion worley

// #region fbm
struct FbmParams {
    int octaves = 6;
    double lacunarity = 2.0;  // 每层频率乘以多少
    double gain = 0.5;        // 每层振幅乘以多少
    bool offsets = true;      // 每层平移一个非整数的偏移，避免各层的格点重合
};

// 第 k 层的偏移：取无理数倍，保证不落在格点上，各层之间也不重合
inline void octaveOffset(int k, double& ox, double& oy) {
    ox = k * 17.320508075688772 + 0.1, oy = k * 31.622776601683793 + 0.7;
}

// base(x, y) 是一层噪声。返回各层之和除以振幅之和，范围不超过 base 的范围
template <class T, class F>
T fbm(F&& base, T x, T y, const FbmParams& p) {
    T sum = 0, norm = 0, amp = 1, freq = 1;
    for (int k = 0; k < p.octaves; ++k) {
        double ox = 0, oy = 0;
        if (p.offsets) octaveOffset(k, ox, oy);
        sum += amp * base(x * freq + T(ox), y * freq + T(oy));
        norm += amp;
        amp *= T(p.gain), freq *= T(p.lacunarity);
    }
    return sum / norm;
}

// 山脊（ridged）：每层取 1 - |n|，噪声的零线变成尖锐的山脊；平方让山脊更窄
template <class T, class F>
T ridged(F&& base, T x, T y, const FbmParams& p) {
    return fbm([&](T u, T v) {
        T r = 1 - std::abs(base(u, v));
        return r * r;
    }, x, y, p);
}
// #endregion fbm

}  // namespace ga::noise
