#pragma once

// 用噪声生成地形
//   - 高度图：在网格上采样 fBm，再用重映射（幂次）、岛屿遮罩调整高度分布
//   - 域扭曲：先用一个噪声扰动采样坐标，再采样另一个噪声，得到被"揉过"的形状
//   - 导数抑制：坡度越大的地方，后面几层的细节越弱，模仿侵蚀后"山坡光滑、山脊粗糙"的样子
//   - 热侵蚀：坡度超过休止角的地方，材料往低处滑落
//   - 水力侵蚀：模拟大量水滴沿坡流下，在快的地方冲刷、慢的地方沉积
//   - D8 汇流：每个格子流向最陡的下坡邻居，累加得到每个格子的上游面积（河网）

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "../fractal/fractal.h"

namespace ga::terrain {

using noise::FbmParams;
using noise::Sample;

struct Heightmap {
    int w = 0, h = 0;
    std::vector<float> v;

    Heightmap() = default;
    Heightmap(int w_, int h_, float fill = 0) : w(w_), h(h_), v(size_t(w_) * h_, fill) {}
    float& at(int x, int y) { return v[size_t(y) * w + x]; }
    float at(int x, int y) const { return v[size_t(y) * w + x]; }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
    double sum() const {
        double s = 0;
        for (float x : v) s += x;
        return s;
    }
};

// #region shape
// e 在 [0, 1]：幂次大于 1 时低处被压平、高处变陡（平原多、山峰少），小于 1 时相反
inline float redistribute(float e, float exponent) { return std::pow(std::clamp(e, 0.0f, 1.0f), exponent); }

// 岛屿遮罩：nx、ny 是到地图中心的归一化坐标（边缘为 ±1）。
// 用 1 - d² 作为"理想的岛屿形状"，按 mix 把高度往它拉；边缘一定低于海面
inline float island(float e, float nx, float ny, float mix) {
    float d2 = std::min(1.0f, nx * nx + ny * ny);
    return e + (1 - d2 - e) * mix;
}
// #endregion shape

// #region warp
// 域扭曲：q = (f(p), f(p + 偏移))，再返回 f(p + strength · q)。
// 两个分量用不同的偏移，避免 q 的两个分量相同（那样只会沿对角线扭曲）
template <class T, class F>
T warped(F&& f, T x, T y, T strength) {
    T qx = f(x, y), qy = f(x + T(5.2), y + T(1.3));
    return f(x + strength * qx, y + strength * qy);
}
// #endregion warp

// #region damped
// 用导数抑制细节的 fBm：累计前面各层的梯度 d，第 k 层的贡献除以 (1 + k·|d|²)。
// d 是"未抑制的部分和"的真实梯度（链式法则：第 k 层的梯度要乘 振幅 × 频率）；
// gain = 0.5、lacunarity = 2 时 振幅 × 频率 = 1，和常见写法里直接累加噪声导数一样
template <class T, class F>
T dampedFbm(F&& baseD, T x, T y, const FbmParams& p, T k = 1) {
    T sum = 0, norm = 0, amp = 1, freq = 1, dx = 0, dy = 0;
    for (int o = 0; o < p.octaves; ++o) {
        double ox = 0, oy = 0;
        if (p.offsets) noise::octaveOffset(o, ox, oy);
        Sample<T> s = baseD(x * freq + T(ox), y * freq + T(oy));
        dx += amp * freq * s.dx, dy += amp * freq * s.dy;
        sum += amp * s.value / (1 + k * (dx * dx + dy * dy));
        norm += amp;
        amp *= T(p.gain), freq *= T(p.lacunarity);
    }
    return sum / norm;
}
// #endregion damped

// #region thermal
enum class Neighborhood {
    Four,         // 上下左右
    Eight,        // 加上对角，但对角也按距离 1 算（常见写法）
    EightScaled,  // 对角按距离 √2 算
};

inline int neighborCount(Neighborhood n) { return n == Neighborhood::Four ? 4 : 8; }
constexpr int kDx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
constexpr int kDy[8] = {0, 1, 0, -1, 1, 1, -1, -1};
inline float neighborDist(Neighborhood n, int k) { return n == Neighborhood::EightScaled && k >= 4 ? 1.41421356f : 1.0f; }

// 一轮热侵蚀。talus 是允许的最大坡度（高度差 / 距离）。
// 对每个格子，找出超过休止角的下坡邻居，超出量 e_i = (h - h_i) - talus · dist_i；
// 搬走 rate · max(e_i)，按 e_i 的比例分给这些邻居。
// 所有格子都根据本轮开始时的高度计算，最后一起更新（结果与遍历顺序无关）。返回本轮搬运的总量
inline double thermalStep(Heightmap& hm, std::vector<float>& delta, float talus, Neighborhood nb, float rate = 0.5f) {
    delta.assign(hm.v.size(), 0.0f);
    const int nn = neighborCount(nb);
    double moved = 0;
    for (int y = 0; y < hm.h; ++y)
        for (int x = 0; x < hm.w; ++x) {
            float h = hm.at(x, y), ex[8], total = 0, emax = 0;
            for (int k = 0; k < nn; ++k) {
                int qx = x + kDx[k], qy = y + kDy[k];
                ex[k] = 0;
                if (!hm.inside(qx, qy)) continue;
                float e = h - hm.at(qx, qy) - talus * neighborDist(nb, k);
                if (e > 0) ex[k] = e, total += e, emax = std::max(emax, e);
            }
            if (total <= 0) continue;
            float amount = rate * emax;
            for (int k = 0; k < nn; ++k) {
                if (ex[k] <= 0) continue;
                float m = amount * ex[k] / total;
                delta[size_t(y) * hm.w + x] -= m;
                delta[size_t(y + kDy[k]) * hm.w + x + kDx[k]] += m;
            }
            moved += amount;
        }
    for (size_t i = 0; i < hm.v.size(); ++i) hm.v[i] += delta[i];
    return moved;
}
// #endregion thermal

// #region sample
// 双线性插值的高度与梯度：格点 (x, y) 存在 at(x, y)，0 ≤ px < w - 1，0 ≤ py < h - 1
inline Sample<float> heightAndGradient(const Heightmap& hm, float px, float py) {
    int x = int(px), y = int(py);
    float u = px - x, v = py - y;
    float h00 = hm.at(x, y), h10 = hm.at(x + 1, y), h01 = hm.at(x, y + 1), h11 = hm.at(x + 1, y + 1);
    return {h00 * (1 - u) * (1 - v) + h10 * u * (1 - v) + h01 * (1 - u) * v + h11 * u * v,
            (h10 - h00) * (1 - v) + (h11 - h01) * v, (h01 - h00) * (1 - u) + (h11 - h10) * u};
}
// #endregion sample

// #region droplet
struct DropletParams {
    float inertia = 0.05f;      // 方向保留多少上一步的方向（0：完全沿梯度；1：直线）
    float capacity = 4.0f;      // 携带泥沙的能力 = capacity × 下降高度 × 速度 × 水量
    float minSlope = 0.01f;     // 计算携带能力时下降高度至少按这个值算，平地上也能带一点泥沙
    float erosion = 0.3f;       // 每步把"还能带多少"的这个比例冲刷起来
    float deposition = 0.3f;    // 每步把"带多了的部分"的这个比例放下
    float evaporation = 0.01f;  // 每步蒸发的水量比例
    float gravity = 4.0f;
    int radius = 3;             // 冲刷时影响的半径（格子），避免挖出一个格子宽的深坑
    int maxSteps = 64;
    bool floorAtNext = true;    // 冲刷时不把格子挖到比水滴下一步更低（见 erode）；关掉用于对比
};

struct DropletStats {
    int steps = 0;
    double eroded = 0, deposited = 0;  // 从地面取走的、放回地面的
    double carriedOff = 0;             // 水滴消失时还带着的泥沙（离开了地面）
};

// 模拟一个水滴，从 (px, py) 出发
inline void simulateDroplet(Heightmap& hm, float px, float py, const DropletParams& p, DropletStats& st) {
    float dirX = 0, dirY = 0, speed = 1, water = 1, sediment = 0;
    auto deposit = [&](float x0, float y0, float amount) {  // 按双线性权重放回 4 个角：权重和为 1
        int x = int(x0), y = int(y0);
        float u = x0 - x, v = y0 - y;
        hm.at(x, y) += amount * (1 - u) * (1 - v), hm.at(x + 1, y) += amount * u * (1 - v);
        hm.at(x, y + 1) += amount * (1 - u) * v, hm.at(x + 1, y + 1) += amount * u * v;
        st.deposited += amount;
    };
    // 按距离加权，从半径内的格子取走；返回实际取走的量。
    // 每个格子最多挖到 floor（水滴下一步所在的高度）：已经比它低的格子不再挖。
    // 没有这一条时，刷子会继续加深水滴旁边的沟底，沟越深后来的水滴落差越大、冲得越狠，最终发散
    auto erode = [&](float x0, float y0, float amount, float floor) {
        int cx = int(x0), cy = int(y0), r = p.radius;
        float wsum = 0, w[64 * 64];
        int n = 0;
        for (int y = cy - r + 1; y <= cy + r; ++y)
            for (int x = cx - r + 1; x <= cx + r; ++x, ++n) {
                float d = std::hypot(x - x0, y - y0);
                w[n] = hm.inside(x, y) ? std::max(0.0f, float(r) - d) : 0.0f;
                wsum += w[n];
            }
        float taken = 0;
        n = 0;
        for (int y = cy - r + 1; y <= cy + r; ++y)
            for (int x = cx - r + 1; x <= cx + r; ++x, ++n) {
                if (w[n] <= 0) continue;
                float t = amount * w[n] / wsum;
                if (p.floorAtNext) t = std::min(t, std::max(0.0f, hm.at(x, y) - floor));
                hm.at(x, y) -= t, taken += t;
            }
        st.eroded += taken;
        return taken;
    };
    for (int step = 0; step < p.maxSteps; ++step) {
        Sample<float> s = heightAndGradient(hm, px, py);
        // 新方向：保留一部分旧方向，其余沿下坡方向
        dirX = dirX * p.inertia - s.dx * (1 - p.inertia);
        dirY = dirY * p.inertia - s.dy * (1 - p.inertia);
        float len = std::hypot(dirX, dirY);
        if (len < 1e-12f) break;  // 完全平坦：停下
        dirX /= len, dirY /= len;
        float nx = px + dirX, ny = py + dirY;
        ++st.steps;
        if (nx < 0 || ny < 0 || nx >= hm.w - 1 || ny >= hm.h - 1) break;  // 流出地图
        float nh = heightAndGradient(hm, nx, ny).value, dh = nh - s.value;
        float cap = std::max(-dh, p.minSlope) * speed * water * p.capacity;
        if (dh > 0 || sediment > cap) {
            // 上坡：用泥沙填平来时的坑（最多填到 dh）；带多了：放下一部分
            float amount = dh > 0 ? std::min(dh, sediment) : (sediment - cap) * p.deposition;
            sediment -= amount;
            deposit(px, py, amount);
        } else {
            // 冲刷量不超过下降的高度，否则会在身后挖出比前方更低的坑
            float amount = std::min((cap - sediment) * p.erosion, -dh);
            sediment += erode(px, py, amount, nh);
        }
        speed = std::sqrt(std::max(0.0f, speed * speed - dh * p.gravity));
        water *= 1 - p.evaporation;
        px = nx, py = ny;
    }
    st.carriedOff += sediment;
}
// #endregion droplet

// #region flow
// D8 汇流：每个格子流向"高度差 / 距离"最大的下坡邻居（没有下坡邻居就是洼地）。
// 按高度从高到低处理，把每个格子的面积加到它的下游，得到每个格子的上游面积
struct FlowResult {
    std::vector<int> down;     // 下游格子的下标，-1 表示没有（洼地或边界外流）
    std::vector<float> area;   // 上游面积（格子数，含自身）
    int pits = 0;              // 内部的洼地（不在边界上、比 8 个邻居都低或相等）
    double drainedOut = 0;     // 最终流到地图边界的面积占比
};

inline FlowResult flow(const Heightmap& hm) {
    const int n = hm.w * hm.h;
    FlowResult r;
    r.down.assign(n, -1), r.area.assign(n, 1.0f);
    for (int y = 0; y < hm.h; ++y)
        for (int x = 0; x < hm.w; ++x) {
            float best = 0, h = hm.at(x, y);
            for (int k = 0; k < 8; ++k) {
                int qx = x + kDx[k], qy = y + kDy[k];
                if (!hm.inside(qx, qy)) continue;
                float s = (h - hm.at(qx, qy)) / (k >= 4 ? 1.41421356f : 1.0f);
                if (s > best) best = s, r.down[size_t(y) * hm.w + x] = qy * hm.w + qx;
            }
            bool edge = x == 0 || y == 0 || x == hm.w - 1 || y == hm.h - 1;
            if (r.down[size_t(y) * hm.w + x] < 0 && !edge) ++r.pits;
        }
    std::vector<int> order(n);
    for (int i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) { return hm.v[a] > hm.v[b]; });
    for (int i : order)
        if (r.down[i] >= 0) r.area[r.down[i]] += r.area[i];
    double out = 0;
    for (int y = 0; y < hm.h; ++y)
        for (int x = 0; x < hm.w; ++x) {
            int i = y * hm.w + x;
            bool edge = x == 0 || y == 0 || x == hm.w - 1 || y == hm.h - 1;
            if (edge && r.down[i] < 0) out += r.area[i];
        }
    r.drainedOut = out / n;
    return r;
}
// #endregion flow

}  // namespace ga::terrain
