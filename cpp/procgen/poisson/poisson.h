#pragma once

// 泊松圆盘采样：任意两点的距离都不小于 r，而且点尽量铺满
//   - 投飞镖（dart throwing）：随机撒候选点，和已有的点都离得够远才接受；连续失败若干次就停
//   - Bridson（2007）：从一个点出发，在活跃点周围的环带 [r, 2r] 里试 k 个候选，一个都放不下就把它移出活跃表
//   - 对照：白噪声（均匀随机）与抖动网格（每格一个随机点）
//   - 测量：最近邻距离、还能再放点的面积比例、功率谱
// 加速结构：边长 r/√2 的网格，每格最多一个点；检查一个候选只需看周围 5×5 格

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

#include "../../foundations/random/random.h"
#include "vec2.h"

namespace ga::poisson {

// #region grid
// 背景网格：格子边长 r/√2，对角线恰好是 r，所以一个格子里最多一个点。
// 与候选点距离小于 r 的点只可能在周围 ±2 格内
class Grid {
public:
    Grid(float w, float h, float r) : r_(r), cell_(r / std::sqrt(2.0f)) {
        nx_ = std::max(1, int(std::ceil(w / cell_))), ny_ = std::max(1, int(std::ceil(h / cell_)));
        idx_.assign(size_t(nx_) * ny_, -1);
    }
    int cellX(float x) const { return std::clamp(int(x / cell_), 0, nx_ - 1); }
    int cellY(float y) const { return std::clamp(int(y / cell_), 0, ny_ - 1); }
    bool fits(Vec2 p, const std::vector<Vec2>& pts) const {
        int cx = cellX(p.x), cy = cellY(p.y);
        for (int y = std::max(0, cy - 2); y <= std::min(ny_ - 1, cy + 2); ++y)
            for (int x = std::max(0, cx - 2); x <= std::min(nx_ - 1, cx + 2); ++x) {
                int i = idx_[size_t(y) * nx_ + x];
                if (i < 0) continue;
                float dx = pts[i].x - p.x, dy = pts[i].y - p.y;
                if (dx * dx + dy * dy < r_ * r_) return false;
            }
        return true;
    }
    void insert(Vec2 p, int i) { idx_[size_t(cellY(p.y)) * nx_ + cellX(p.x)] = i; }
    float cell() const { return cell_; }
    int nx() const { return nx_; }
    int ny() const { return ny_; }
    int at(int x, int y) const { return idx_[size_t(y) * nx_ + x]; }

private:
    float r_, cell_;
    int nx_ = 1, ny_ = 1;
    std::vector<int> idx_;
};
// #endregion grid

// #region dart
// 投飞镖：连续 maxFailures 次候选都放不下就停。useGrid = false 时逐个比较所有已有的点（O(n) 一次），结果完全相同
inline std::vector<Vec2> dartThrowing(float w, float h, float r, Pcg32& rng, long long maxFailures, bool useGrid = true,
                                      long long* attempts = nullptr) {
    std::vector<Vec2> pts;
    Grid g(w, h, r);
    long long fails = 0, tries = 0;
    while (fails < maxFailures) {
        Vec2 p{rng.nextFloat() * w, rng.nextFloat() * h};
        ++tries;
        bool ok = true;
        if (useGrid) ok = g.fits(p, pts);
        else
            for (const Vec2& q : pts)
                if ((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) < r * r) {
                    ok = false;
                    break;
                }
        if (!ok) {
            ++fails;
            continue;
        }
        fails = 0;
        g.insert(p, int(pts.size()));
        pts.push_back(p);
    }
    if (attempts) *attempts = tries;
    return pts;
}
// #endregion dart

// #region bridson
enum class Annulus {
    Area,    // 在环带 [r, 2r] 里按面积均匀取（Bridson 原文）
    Radius,  // 半径在 [r, 2r] 里均匀取（很多教程的写法，偏向内圈）
    Circle,  // 全部放在半径 r(1+ε) 的圆上，角度等间隔、起点随机（Martin Roberts 的改进）
};
enum class Pick { Random, Oldest, Newest };  // 从活跃表里选哪个点

struct BridsonParams {
    int k = 30;  // 每个活跃点最多试几个候选
    Annulus annulus = Annulus::Area;
    Pick pick = Pick::Random;
};

// 动画用：每次尝试的记录
struct Trial {
    int from;      // 活跃点
    Vec2 p;        // 候选
    bool accepted;
};

inline std::vector<Vec2> bridson(float w, float h, float r, Pcg32& rng, const BridsonParams& prm, long long* attempts = nullptr,
                                 std::vector<Trial>* log = nullptr) {
    std::vector<Vec2> pts;
    std::vector<int> active;
    Grid g(w, h, r);
    auto add = [&](Vec2 p) {
        g.insert(p, int(pts.size()));
        active.push_back(int(pts.size()));
        pts.push_back(p);
    };
    add({rng.nextFloat() * w, rng.nextFloat() * h});
    long long tries = 0;
    while (!active.empty()) {
        size_t slot = prm.pick == Pick::Random ? rng.below(uint32_t(active.size())) : prm.pick == Pick::Oldest ? 0 : active.size() - 1;
        int a = active[slot];
        Vec2 c = pts[a];
        bool placed = false;
        float phase = rng.nextFloat() * 2 * kPi;
        for (int j = 0; j < prm.k && !placed; ++j) {
            float rad, ang;
            if (prm.annulus == Annulus::Area) rad = r * std::sqrt(1 + 3 * rng.nextFloat()), ang = rng.nextFloat() * 2 * kPi;
            else if (prm.annulus == Annulus::Radius) rad = r * (1 + rng.nextFloat()), ang = rng.nextFloat() * 2 * kPi;
            else rad = r * 1.0001f, ang = phase + 2 * kPi * j / prm.k;
            Vec2 p{c.x + rad * std::cos(ang), c.y + rad * std::sin(ang)};
            ++tries;
            bool ok = p.x >= 0 && p.y >= 0 && p.x < w && p.y < h && g.fits(p, pts);
            if (log) log->push_back({a, p, ok});
            if (ok) add(p), placed = true;
        }
        if (!placed) {  // 移出活跃表：Oldest 要保持顺序，其余用末尾元素填空
            if (prm.pick == Pick::Oldest) active.erase(active.begin());
            else active[slot] = active.back(), active.pop_back();
        }
    }
    if (attempts) *attempts = tries;
    return pts;
}
// #endregion bridson

// #region baselines
inline std::vector<Vec2> whiteNoise(float w, float h, int n, Pcg32& rng) {
    std::vector<Vec2> pts(n);
    for (Vec2& p : pts) p = {rng.nextFloat() * w, rng.nextFloat() * h};
    return pts;
}

// 抖动网格：边长 s 的格子里各放一个随机点
inline std::vector<Vec2> jitteredGrid(float w, float h, float s, Pcg32& rng) {
    std::vector<Vec2> pts;
    for (int y = 0; y < int(h / s); ++y)
        for (int x = 0; x < int(w / s); ++x) pts.push_back({(x + rng.nextFloat()) * s, (y + rng.nextFloat()) * s});
    return pts;
}
// #endregion baselines

// #region measure
// 覆盖率：n 个直径为 r 的圆盘（互不重叠）占的面积比例。随机序贯吸附的极限约为 0.547
inline double packingDensity(size_t n, float w, float h, float r) { return n * double(kPi) * r * r / 4 / (double(w) * h); }

// 每个点到最近的另一个点的距离（网格加速：先按边长 cell 分桶，由近到远一圈圈地找）
inline std::vector<float> nearestNeighbor(const std::vector<Vec2>& pts, float w, float h, float cell) {
    int nx = std::max(1, int(std::ceil(w / cell))), ny = std::max(1, int(std::ceil(h / cell)));
    std::vector<std::vector<int>> b(size_t(nx) * ny);
    auto cx = [&](float x) { return std::clamp(int(x / cell), 0, nx - 1); };
    auto cy = [&](float y) { return std::clamp(int(y / cell), 0, ny - 1); };
    for (int i = 0; i < int(pts.size()); ++i) b[size_t(cy(pts[i].y)) * nx + cx(pts[i].x)].push_back(i);
    std::vector<float> out(pts.size(), INFINITY);
    for (int i = 0; i < int(pts.size()); ++i) {
        int x0 = cx(pts[i].x), y0 = cy(pts[i].y);
        float best2 = INFINITY;
        for (int ring = 0; ring <= std::max(nx, ny); ++ring) {
            // 第 ring 圈之外的点距离至少 ring * cell
            if (best2 <= (ring - 1) * cell * (ring - 1) * cell && ring > 0) break;
            for (int y = y0 - ring; y <= y0 + ring; ++y)
                for (int x = x0 - ring; x <= x0 + ring; ++x) {
                    if (std::max(std::abs(x - x0), std::abs(y - y0)) != ring || x < 0 || y < 0 || x >= nx || y >= ny) continue;
                    for (int j : b[size_t(y) * nx + x]) {
                        if (j == i) continue;
                        float dx = pts[j].x - pts[i].x, dy = pts[j].y - pts[i].y;
                        best2 = std::min(best2, dx * dx + dy * dy);
                    }
                }
        }
        out[i] = std::sqrt(best2);
    }
    return out;
}

// 还能再放点的面积比例：在 res × res 的细格点上检查，离所有点都至少 r 的格点占多少。0 表示"极大"（再也放不下）
inline double uncoveredFraction(const std::vector<Vec2>& pts, float w, float h, float r, int res) {
    Grid g(w, h, r);
    for (int i = 0; i < int(pts.size()); ++i) g.insert(pts[i], i);  // 只用于泊松圆盘点集：每格最多一个点
    long long open = 0, total = 0;
    for (int y = 0; y < res; ++y)
        for (int x = 0; x < res; ++x) {
            Vec2 p{(x + 0.5f) * w / res, (y + 0.5f) * h / res};
            open += g.fits(p, pts), ++total;
        }
    return double(open) / total;
}

// 功率谱（周期图）：P(f) = |Σ exp(-2πi f·p)|² / n，f = (kx / w, ky / h)，kx, ky ∈ [-m, m]。
// 结果按 (ky + m) * (2m + 1) + (kx + m) 排列。白噪声的期望处处为 1；P(0) = n
inline std::vector<float> periodogram(const std::vector<Vec2>& pts, float w, float h, int m) {
    int side = 2 * m + 1;
    std::vector<float> out(size_t(side) * side);
    // exp(-2πi kx x / w) = e^{kx}，按 kx 递推，避免每个频率都算三角函数
    std::vector<std::complex<double>> ex(pts.size() * side), ey(pts.size() * side);
    for (size_t i = 0; i < pts.size(); ++i)
        for (int k = -m; k <= m; ++k) {
            ex[i * side + k + m] = std::polar(1.0, -2 * M_PI * k * pts[i].x / w);
            ey[i * side + k + m] = std::polar(1.0, -2 * M_PI * k * pts[i].y / h);
        }
    for (int ky = 0; ky < side; ++ky)
        for (int kx = 0; kx < side; ++kx) {
            std::complex<double> s = 0;
            for (size_t i = 0; i < pts.size(); ++i) s += ex[i * side + kx] * ey[i * side + ky];
            out[size_t(ky) * side + kx] = float(std::norm(s) / pts.size());
        }
    return out;
}

// 径向平均：按 |k| 四舍五入分组。返回 bins[0..m]
inline std::vector<double> radialAverage(const std::vector<float>& P, int m) {
    int side = 2 * m + 1;
    std::vector<double> sum(m + 1, 0), cnt(m + 1, 0);
    for (int ky = -m; ky <= m; ++ky)
        for (int kx = -m; kx <= m; ++kx) {
            int b = int(std::lround(std::sqrt(double(kx * kx + ky * ky))));
            if (b > m || (kx == 0 && ky == 0)) continue;
            sum[b] += P[size_t(ky + m) * side + kx + m], cnt[b] += 1;
        }
    for (int b = 0; b <= m; ++b) sum[b] = cnt[b] ? sum[b] / cnt[b] : 0;
    return sum;
}
// #endregion measure

}  // namespace ga::poisson
