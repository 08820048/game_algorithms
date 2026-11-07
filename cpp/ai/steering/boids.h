#pragma once

// Boids（Craig Reynolds 1987）：每只鸟只看得见附近的同伴，只遵守三条规则：
//   分离（separation）：别和邻居挤在一起
//   对齐（alignment）  ：和邻居朝同一个方向飞
//   聚合（cohesion）   ：向邻居的中心靠拢
// 没有领头的，也没有人知道整个鸟群的形状，鸟群却会自己成形、分开、再合拢。
//
// 邻居查询：
//   - forcesBrute：每只鸟检查所有其他鸟，O(n²)，作为对照
//   - FlockGrid：格子边长不小于感知半径，邻居一定在周围 3×3 个格子里。
//     每帧用计数排序把鸟按格子重新排列，同一行相邻三个格子在数组里是连续的一段，
//     而且按行从上到下，下标严格递增：网格版与暴力版按完全相同的顺序累加，结果逐位相同

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "steering.h"

namespace ga::boids {

using steering::truncate;

// #region params
struct Boid {
    Vec2 pos, vel;
    int id = 0;  // 鸟在数组里的位置每帧都会变，id 用来认出同一只鸟
};

struct Params {
    float radius = 48;          // 感知半径：只有这么近的鸟才算邻居
    float separation = 22;      // 分离半径：比这更近才互相推开
    float fovCos = -0.5f;       // 视野：与前进方向夹角的余弦不小于它才看得见（-0.5 即前方 240°）
    float wSeparation = 1.6f, wAlignment = 1.0f, wCohesion = 0.5f;
    float maxSpeed = 110, minSpeed = 50, maxForce = 260;
    float reaction = 0.3f;      // 反应时间，含义见 steering.h
    float width = 800, height = 450, margin = 40;  // 场地；离边界不到 margin 时被推回来
};
// #endregion

// #region neighbor
// 一只鸟从所有邻居那里收集的信息
struct Accum {
    Vec2 away;        // 分离：远离每个太近的邻居，越近权重越大
    Vec2 heading;     // 对齐：邻居速度之和
    Vec2 offset;      // 聚合：邻居相对位置之和
    int count = 0;
};

// 判断 other 是不是 self 的邻居，是就累加进 acc。暴力版与网格版调用的是同一个函数
inline void addNeighbor(Accum& acc, const Boid& self, Vec2 fwd, const Boid& other, const Params& p) {
    Vec2 d = other.pos - self.pos;
    float d2 = lengthSq(d);
    if (d2 >= p.radius * p.radius) return;
    if (dot(fwd, d) < p.fovCos * std::sqrt(d2)) return;  // 在背后的视野盲区里
    acc.heading += other.vel;
    acc.offset += d;
    ++acc.count;
    if (d2 < p.separation * p.separation && d2 > 0) {  // 越近推得越用力：分离半径处为 0，贴在一起时为 1
        float dist = std::sqrt(d2);
        acc.away -= d * ((p.separation - dist) / (p.separation * dist));
    }
}
// #endregion

// #region rules
// 三条规则各自给出一个"想要的速度"，转向力 = (想要的速度 - 当前速度) / 反应时间，
// 分别截断到力的上限后加权相加；最后加上把鸟推回场地的力
inline Vec2 steer(const Boid& b, const Accum& acc, const Params& p) {
    auto toward = [&](Vec2 dir) { return truncate((normalized(dir) * p.maxSpeed - b.vel) / p.reaction, p.maxForce); };
    Vec2 force{};
    if (acc.count > 0) {
        force += truncate(acc.away, 1) * (p.maxForce * p.wSeparation);  // 分离直接是推力，不经过"想要的速度"
        if (lengthSq(acc.heading) > 0) force += toward(acc.heading) * p.wAlignment;
        force += acc.offset / (acc.count * p.radius) * (p.maxForce * p.wCohesion);  // 聚合像一根弹簧：离中心越远拉得越紧
    }
    Vec2 wall{};
    if (b.pos.x < p.margin) wall.x += (p.margin - b.pos.x) / p.margin;
    if (b.pos.x > p.width - p.margin) wall.x -= (b.pos.x - (p.width - p.margin)) / p.margin;
    if (b.pos.y < p.margin) wall.y += (p.margin - b.pos.y) / p.margin;
    if (b.pos.y > p.height - p.margin) wall.y -= (b.pos.y - (p.height - p.margin)) / p.margin;
    return force + wall * (p.maxForce * 2);
}

inline Vec2 forward(const Boid& b) { return normalized(b.vel); }
// #endregion

// #region brute
// 对照：每只鸟检查所有其他鸟，下标从小到大
inline void forcesBrute(const std::vector<Boid>& boids, const Params& p, std::vector<Vec2>& out) {
    int n = static_cast<int>(boids.size());
    out.assign(n, {});
    for (int i = 0; i < n; ++i) {
        Accum acc;
        Vec2 fwd = forward(boids[i]);
        for (int j = 0; j < n; ++j)
            if (j != i) addNeighbor(acc, boids[i], fwd, boids[j], p);
        out[i] = steer(boids[i], acc, p);
    }
}
// #endregion

class FlockGrid {
public:
    // #region grid-build
    // 计数排序：数出每个格子有几只鸟，前缀和得到每个格子在数组里的起点，再把鸟搬过去。
    // 排序是稳定的；鸟每帧只移动一两个像素，上一帧的顺序大体还能用
    void build(std::vector<Boid>& boids, const Params& p) {
        cell_ = std::max(p.radius, 1.0f);
        cols_ = std::max(1, static_cast<int>(std::ceil(p.width / cell_)));
        rows_ = std::max(1, static_cast<int>(std::ceil(p.height / cell_)));
        start_.assign(static_cast<size_t>(cols_) * rows_ + 1, 0);
        cellOf_.resize(boids.size());
        for (size_t i = 0; i < boids.size(); ++i) ++start_[(cellOf_[i] = cellIndex(boids[i].pos)) + 1];
        std::partial_sum(start_.begin(), start_.end(), start_.begin());
        scratch_.resize(boids.size());
        std::vector<int> fill(start_.begin(), start_.end() - 1);
        for (size_t i = 0; i < boids.size(); ++i) scratch_[fill[cellOf_[i]]++] = boids[i];
        boids.swap(scratch_);
    }
    // #endregion

    // #region grid-query
    // 邻居只可能在周围 3×3 个格子里。同一行的三个格子在数组中首尾相接，一次遍历一整段
    void forces(const std::vector<Boid>& boids, const Params& p, std::vector<Vec2>& out) const {
        int n = static_cast<int>(boids.size());
        out.resize(n);
        for (int i = 0; i < n; ++i) {
            const Boid& b = boids[i];
            Vec2 fwd = forward(b);
            int cx = cellX(b.pos.x), cy = cellY(b.pos.y);
            int x0 = std::max(cx - 1, 0), x1 = std::min(cx + 1, cols_ - 1);
            Accum acc;
            for (int y = std::max(cy - 1, 0); y <= std::min(cy + 1, rows_ - 1); ++y)
                for (int j = start_[y * cols_ + x0], end = start_[y * cols_ + x1 + 1]; j < end; ++j)
                    if (j != i) addNeighbor(acc, b, fwd, boids[j], p);
            out[i] = steer(b, acc, p);
        }
    }
    // #endregion

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    float cellSize() const { return cell_; }

    // 场地外的鸟被"夹"进最外圈的格子。夹取不会让相距不到一格的两只鸟落到隔了一格的格子里，结果依然正确
    int cellX(float x) const { return std::clamp(static_cast<int>(std::floor(x / cell_)), 0, cols_ - 1); }
    int cellY(float y) const { return std::clamp(static_cast<int>(std::floor(y / cell_)), 0, rows_ - 1); }
    int cellIndex(Vec2 v) const { return cellY(v.y) * cols_ + cellX(v.x); }

private:
    float cell_ = 1;
    int cols_ = 1, rows_ = 1;
    std::vector<int> start_, cellOf_;
    std::vector<Boid> scratch_;
};

// #region step
// 一帧：先根据所有鸟当前的状态算出全部的力，再统一移动。
// 如果边算边移动，排在后面的鸟看到的是邻居"这一帧之后"的位置，结果会依赖数组顺序
inline void integrate(std::vector<Boid>& boids, const std::vector<Vec2>& forces, const Params& p, float dt) {
    for (size_t i = 0; i < boids.size(); ++i) {
        Boid& b = boids[i];
        b.vel = truncate(b.vel + truncate(forces[i], p.maxForce) * dt, p.maxSpeed);
        float s = length(b.vel);
        if (s < p.minSpeed) b.vel = s > 1e-6f ? b.vel * (p.minSpeed / s) : Vec2{p.minSpeed, 0};  // 鸟不会悬停
        b.pos += b.vel * dt;
    }
}
// #endregion

// ---- 观察鸟群的几个指标 ----

// 有序度（polarization）：所有鸟的单位速度取平均后的长度。1 = 全部同向，接近 0 = 各飞各的
inline float polarization(const std::vector<Boid>& boids) {
    Vec2 sum{};
    for (const Boid& b : boids) sum += normalized(b.vel);
    return boids.empty() ? 0 : length(sum) / boids.size();
}

// 分成了几群：相距不到 link 的鸟算同一群（并查集，O(n²)，只用于统计）
inline int groups(const std::vector<Boid>& boids, float link) {
    int n = static_cast<int>(boids.size());
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&](int x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (lengthSq(boids[i].pos - boids[j].pos) < link * link) parent[find(i)] = find(j);
    int count = 0;
    for (int i = 0; i < n; ++i) count += find(i) == i;
    return count;
}

// 每只鸟到最近同伴的距离的平均值与最小值
inline void nearestStats(const std::vector<Boid>& boids, float* mean, float* min) {
    double sum = 0;
    float lo = 1e9f;
    for (size_t i = 0; i < boids.size(); ++i) {
        float best = 1e9f;
        for (size_t j = 0; j < boids.size(); ++j)
            if (j != i) best = std::min(best, lengthSq(boids[i].pos - boids[j].pos));
        best = std::sqrt(best);
        sum += best, lo = std::min(lo, best);
    }
    *mean = boids.empty() ? 0 : float(sum / boids.size());
    *min = lo;
}

inline std::vector<Boid> randomFlock(int n, const Params& p, Pcg32& rng) {
    std::vector<Boid> boids(n);
    for (int i = 0; i < n; ++i) {
        boids[i].pos = {p.margin + rng.nextFloat() * (p.width - 2 * p.margin), p.margin + rng.nextFloat() * (p.height - 2 * p.margin)};
        boids[i].vel = fromAngle(rng.nextFloat() * 2 * kPi) * (p.minSpeed + rng.nextFloat() * (p.maxSpeed - p.minSpeed));
        boids[i].id = i;
    }
    return boids;
}

}  // namespace ga::boids
