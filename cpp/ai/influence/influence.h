#pragma once

// 影响力图（influence map）：把"哪里是谁的地盘、哪里危险"画在一张网格上。
// 每个单位向周围散发影响力，离得越远越弱；AI 不再逐个比较单位，而是直接查网格。
//
//   - stampBrute / stamp：按直线距离线性衰减，不考虑墙。暴力版对每个格子累加所有单位，
//     盖章版预先算好一个"印章"，逐个单位盖上去。两者对每个格子按相同的单位顺序累加，结果逐位相同
//   - propagate：迭代传播，每一轮每个格子取"邻居的值 × 衰减"的最大值。影响力只能沿可通行的格子扩散，
//     会绕过墙。收敛后等于"沿最短路径衰减"，与对每个单位跑一次 Dijkstra（geodesicBrute）逐位相同
//   - combine：己方、敌方两张图组合出势力、紧张度、薄弱度
//   - safePath：把敌方影响力当作额外的代价，用 A* 找一条避开火力的路

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <vector>

namespace ga::influence {

// #region map
struct Source {
    int x, y;
    float strength;
};

struct Map {
    int w = 0, h = 0;
    std::vector<uint8_t> wall;  // 1 = 墙
    Map() = default;
    Map(int w_, int h_) : w(w_), h(h_), wall(static_cast<size_t>(w_) * h_, 0) {}
    int index(int x, int y) const { return y * w + x; }
    bool open(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h && !wall[index(x, y)]; }
};

using Field = std::vector<float>;  // 每个格子一个值，按行存放
// #endregion

// #region stamp
// 线性衰减：距离为 0 时等于强度，距离达到 radius 时降为 0
inline float falloff(int dx, int dy, float radius) {
    return std::max(0.0f, 1 - std::sqrt(float(dx * dx + dy * dy)) / radius);
}

// 暴力：每个格子把所有单位的贡献加起来。O(格子数 × 单位数)
inline void stampBrute(const Map& m, const std::vector<Source>& sources, float radius, Field& out) {
    out.assign(static_cast<size_t>(m.w) * m.h, 0);
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x) {
            float v = 0;
            for (const Source& s : sources) {
                float f = falloff(x - s.x, y - s.y, radius);
                if (f > 0) v += s.strength * f;
            }
            out[m.index(x, y)] = v;
        }
}

// 盖章：衰减只取决于偏移量，预先算成一张表，每个单位只碰自己半径内的格子。O(单位数 × 半径²)
class Stamper {
public:
    explicit Stamper(float radius) : radius_(radius), r_(static_cast<int>(std::ceil(radius))) {
        int side = 2 * r_ + 1;
        kernel_.resize(static_cast<size_t>(side) * side);
        for (int dy = -r_; dy <= r_; ++dy)
            for (int dx = -r_; dx <= r_; ++dx) kernel_[(dy + r_) * side + dx + r_] = falloff(dx, dy, radius);
    }

    void stamp(const Map& m, const std::vector<Source>& sources, Field& out) const {
        out.assign(static_cast<size_t>(m.w) * m.h, 0);
        int side = 2 * r_ + 1;
        for (const Source& s : sources) {
            int y0 = std::max(s.y - r_, 0), y1 = std::min(s.y + r_, m.h - 1);
            int x0 = std::max(s.x - r_, 0), x1 = std::min(s.x + r_, m.w - 1);
            for (int y = y0; y <= y1; ++y) {
                const float* k = &kernel_[(y - s.y + r_) * side - s.x + r_];
                float* row = &out[m.index(0, y)];
                for (int x = x0; x <= x1; ++x)
                    if (k[x] > 0) row[x] += s.strength * k[x];
            }
        }
    }

    float radius() const { return radius_; }

private:
    float radius_;
    int r_;
    std::vector<float> kernel_;
};
// #endregion

// #region neighbors
// 8 邻域，斜向移动不能切过墙角。直走一格衰减 decay，斜走一格衰减 decay^√2
template <class F>
inline void forEachNeighbor(const Map& m, int x, int y, F&& visit) {
    static constexpr int dx[] = {1, -1, 0, 0, 1, 1, -1, -1}, dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int k = 0; k < 8; ++k) {
        int nx = x + dx[k], ny = y + dy[k];
        if (!m.open(nx, ny)) continue;
        if (k >= 4 && (!m.open(nx, y) || !m.open(x, ny))) continue;
        visit(nx, ny, k >= 4);
    }
}

struct Decay {
    float straight, diagonal;
    float floor;  // 低于它就当作 0，传播到此为止
    explicit Decay(float perCell, float floor_ = 0.01f)
        : straight(perCell), diagonal(std::pow(perCell, std::sqrt(2.0f))), floor(floor_) {}
};
// #endregion

// #region propagate
// 源头：每个单位所在的格子取它的强度（同一格有多个单位时取最大）
inline void seed(const Map& m, const std::vector<Source>& sources, Field& out) {
    out.assign(static_cast<size_t>(m.w) * m.h, 0);
    for (const Source& s : sources)
        if (m.open(s.x, s.y)) out[m.index(s.x, s.y)] = std::max(out[m.index(s.x, s.y)], s.strength);
}

// 一轮传播（Jacobi）：新值 = max(源头, 每个邻居的旧值 × 衰减)，再按 momentum 向新值靠拢。
// momentum = 1 时直接取新值；小于 1 时影响力变化得更平滑，但也更迟钝。返回本轮的最大变化量
inline float propagateStep(const Map& m, const Field& seeds, const Decay& d, float momentum, Field& cur, Field& scratch) {
    scratch.resize(cur.size());
    float change = 0;
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x) {
            int i = m.index(x, y);
            if (m.wall[i]) {
                scratch[i] = 0;
                continue;
            }
            float best = seeds[i];
            forEachNeighbor(m, x, y, [&](int nx, int ny, bool diag) {
                float v = cur[m.index(nx, ny)] * (diag ? d.diagonal : d.straight);
                if (v >= d.floor) best = std::max(best, v);
            });
            float next = cur[i] + (best - cur[i]) * momentum;
            change = std::max(change, std::abs(next - cur[i]));
            scratch[i] = next;
        }
    cur.swap(scratch);
    return change;
}

// 一直传播到不再变化，返回用了多少轮
inline int propagate(const Map& m, const std::vector<Source>& sources, const Decay& d, Field& out) {
    Field seeds, scratch;
    seed(m, sources, seeds);
    out = seeds;
    int rounds = 0;
    while (propagateStep(m, seeds, d, 1, out, scratch) > 0) ++rounds;
    return rounds;
}
// #endregion

// #region sweep
// 更快的收敛：Gauss-Seidel 原地更新，正向（从左上到右下）、反向交替扫描。
// 正向扫描时左边和上边的邻居已经是本轮的新值，影响力一趟就能传到对角；没有墙时两趟就收敛
inline int propagateSweeps(const Map& m, const std::vector<Source>& sources, const Decay& d, Field& out) {
    Field seeds;
    seed(m, sources, seeds);
    out = seeds;
    int passes = 0;
    for (bool changed = true; changed; ++passes) {
        changed = false;
        bool forward = passes % 2 == 0;
        for (int k = 0; k < m.w * m.h; ++k) {
            int i = forward ? k : m.w * m.h - 1 - k;
            int x = i % m.w, y = i / m.w;
            if (m.wall[i]) continue;
            float best = out[i];
            forEachNeighbor(m, x, y, [&](int nx, int ny, bool diag) {
                float v = out[m.index(nx, ny)] * (diag ? d.diagonal : d.straight);
                if (v >= d.floor) best = std::max(best, v);
            });
            if (best != out[i]) out[i] = best, changed = true;
        }
    }
    return passes;
}
// #endregion

// #region geodesic
// 对照：对每个单位跑一次 Dijkstra。不过这里比较的不是"距离最短"，而是"剩下的影响力最大"：
// 每走一步乘以衰减，值只会变小，所以可以像 Dijkstra 一样，每次取出当前值最大的格子，它的值就是最终值。
// 浮点乘法是单调的（x ≤ y 时 x·f ≤ y·f），这个论证对舍入后的结果同样成立，因此与迭代传播逐位相同
inline void geodesicBrute(const Map& m, const std::vector<Source>& sources, const Decay& d, Field& out) {
    out.assign(static_cast<size_t>(m.w) * m.h, 0);
    Field best;
    using Item = std::pair<float, int>;
    for (const Source& s : sources) {
        if (!m.open(s.x, s.y)) continue;
        best.assign(out.size(), 0);
        std::priority_queue<Item> open;  // 最大堆
        best[m.index(s.x, s.y)] = s.strength;
        open.push({s.strength, m.index(s.x, s.y)});
        while (!open.empty()) {
            auto [v, i] = open.top();
            open.pop();
            if (v < best[i]) continue;  // 过期的条目
            forEachNeighbor(m, i % m.w, i / m.w, [&](int nx, int ny, bool diag) {
                float nv = v * (diag ? d.diagonal : d.straight);
                int j = m.index(nx, ny);
                if (nv >= d.floor && nv > best[j]) best[j] = nv, open.push({nv, j});
            });
        }
        for (size_t i = 0; i < out.size(); ++i) out[i] = std::max(out[i], best[i]);
    }
}
// #endregion

// #region combine
// 己方影响力 A、敌方影响力 B：
//   势力（influence）  = A - B ：正数是我方地盘，负数是敌方地盘，接近 0 的地方是前线
//   紧张度（tension）   = A + B ：双方都在这里投入了力量，冲突最激烈
//   薄弱度（vulnerability）= 紧张度 - |势力| = 2·min(A, B)：双方势均力敌、谁都没占上风的地方
enum class View { Ally, Enemy, Influence, Tension, Vulnerability };

inline float combine(float ally, float enemy, View v) {
    switch (v) {
    case View::Ally: return ally;
    case View::Enemy: return enemy;
    case View::Influence: return ally - enemy;
    case View::Tension: return ally + enemy;
    case View::Vulnerability: return ally + enemy - std::abs(ally - enemy);
    }
    return 0;
}
// #endregion

// #region safe-path
// 避开火力的路径：走一步的代价 = 步长 × (1 + danger × 这一格的敌方影响力)。
// danger = 0 时就是普通的最短路；越大越愿意绕远路换取安全。启发函数是八方向距离，代价不小于步长，所以仍然可采纳
struct PathResult {
    std::vector<int> cells;  // 从起点到终点的格子下标；找不到时为空
    float cost = 0;
};

inline PathResult safePath(const Map& m, const Field& threat, int start, int goal, float danger, float heuristicWeight = 1) {
    const float kDiag = std::sqrt(2.0f);
    auto h = [&](int i) {
        float dx = std::abs(i % m.w - goal % m.w), dy = std::abs(i / m.w - goal / m.w);
        return (std::max(dx, dy) + (kDiag - 1) * std::min(dx, dy)) * heuristicWeight;
    };
    std::vector<float> g(m.wall.size(), 1e30f);
    std::vector<int> parent(m.wall.size(), -1);
    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
    g[start] = 0;
    open.push({h(start), start});
    while (!open.empty()) {
        auto [f, i] = open.top();
        open.pop();
        if (f > g[i] + h(i)) continue;
        if (i == goal) break;
        forEachNeighbor(m, i % m.w, i / m.w, [&](int nx, int ny, bool diag) {
            int j = m.index(nx, ny);
            float step = (diag ? kDiag : 1.0f) * (1 + danger * threat[j]);
            if (g[i] + step < g[j]) g[j] = g[i] + step, parent[j] = i, open.push({g[j] + h(j), j});
        });
    }
    PathResult r;
    if (parent[goal] < 0 && goal != start) return r;
    for (int i = goal; i != -1; i = parent[i]) r.cells.push_back(i);
    std::reverse(r.cells.begin(), r.cells.end());
    r.cost = g[goal];
    return r;
}
// #endregion

}  // namespace ga::influence
