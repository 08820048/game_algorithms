#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "cost_grid.h"
#include "vec2.h"

namespace ga {

constexpr int kFlowUnreached = std::numeric_limits<int>::max();

// 8 方向邻居。斜走不允许切墙角；代价 = 格子代价 × (直走 10 / 斜走 14)
template <class F>
void forEachNeighbor8(const CostGrid& grid, Cell c, F&& visit) {
    static constexpr Cell kDirs[8] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
    for (Cell d : kDirs) {
        Cell n{c.x + d.x, c.y + d.y};
        if (!grid.passable(n)) continue;
        bool diagonal = d.x != 0 && d.y != 0;
        if (diagonal && (!grid.passable({c.x + d.x, c.y}) || !grid.passable({c.x, c.y + d.y}))) continue;
        visit(n, grid.cost(n) * (diagonal ? 14 : 10));
    }
}

// #region integration
// 积分场：从目标出发做一次 Dijkstra，得到每个格子走到目标的最小代价。
// 和 1.2 的 Dijkstra 地图是同一个东西，只是换成了 8 方向。
inline std::vector<int> integrationField(const CostGrid& grid, const std::vector<Cell>& goals) {
    std::vector<int> cost(grid.size(), kFlowUnreached);
    using Entry = std::pair<int, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    for (Cell g : goals) {
        if (!grid.passable(g)) continue;
        cost[grid.index(g)] = 0;
        open.push({0, grid.index(g)});
    }
    while (!open.empty()) {
        auto [c, i] = open.top();
        open.pop();
        if (c > cost[i]) continue;
        forEachNeighbor8(grid, grid.cellAt(i), [&](Cell n, int step) {
            int ni = grid.index(n);
            if (c + step < cost[ni]) {
                cost[ni] = c + step;
                open.push({cost[ni], ni});
            }
        });
    }
    return cost;
}
// #endregion

// #region flow
// 流场：每个格子指向积分值最小的邻居。目标格子和不可达的格子方向为零。
// 积分场除了目标之外没有"局部最小值"，所以顺着箭头走一定能到达目标。
inline std::vector<Vec2> flowField(const CostGrid& grid, const std::vector<int>& integration) {
    std::vector<Vec2> flow(grid.size(), Vec2{0, 0});
    for (int i = 0; i < grid.size(); ++i) {
        if (integration[i] == kFlowUnreached || integration[i] == 0) continue;
        Cell c = grid.cellAt(i), best = c;
        int bestCost = integration[i];
        forEachNeighbor8(grid, c, [&](Cell n, int) {
            if (integration[grid.index(n)] < bestCost) {
                bestCost = integration[grid.index(n)];
                best = n;
            }
        });
        flow[i] = normalized(Vec2{static_cast<float>(best.x - c.x), static_cast<float>(best.y - c.y)});
    }
    return flow;
}
// #endregion

// #region sample
// 在任意位置采样流场方向（位置以格子为单位，格子 (x, y) 覆盖 [x, x+1) × [y, y+1)）。
// 对周围 4 个格子的方向做双线性插值，单位转向更平滑，不会在格子边界上突然折返。
inline Vec2 sampleFlow(const CostGrid& grid, const std::vector<Vec2>& flow, Vec2 p) {
    float u = p.x - 0.5f, v = p.y - 0.5f;
    int x0 = static_cast<int>(std::floor(u)), y0 = static_cast<int>(std::floor(v));
    float fx = u - x0, fy = v - y0;
    Vec2 sum{0, 0};
    float weightSum = 0;
    for (int k = 0; k < 4; ++k) {
        Cell c{x0 + (k & 1), y0 + (k >> 1)};
        if (!grid.passable(c)) continue;  // 墙没有方向，不参与插值
        Vec2 d = flow[grid.index(c)];
        if (d == Vec2{0, 0}) continue;
        float w = ((k & 1) ? fx : 1 - fx) * ((k >> 1) ? fy : 1 - fy);
        sum += d * w;
        weightSum += w;
    }
    if (weightSum < 1e-4f) {  // 周围都没有方向：退回所在格子自己的方向
        Cell c{static_cast<int>(std::floor(p.x)), static_cast<int>(std::floor(p.y))};
        return grid.passable(c) ? flow[grid.index(c)] : Vec2{0, 0};
    }
    return normalized(sum);
}
// #endregion

// #region agents
struct Agent {
    Vec2 pos;  // 以格子为单位
    Vec2 vel;
};

struct CrowdParams {
    float radius = 0.16f;     // 单位半径（格子）
    float maxSpeed = 3.0f;    // 平地上的最大速度（格子 / 秒）
    float maxForce = 12.0f;   // 最大转向加速度
    float separation = 6.0f;  // 分离力强度
};

// 推进一帧：每个单位朝流场方向加速，同时和附近的单位保持距离，最后从墙里推出来
inline void stepCrowd(const CostGrid& grid, const std::vector<Vec2>& flow, Cell goal, std::vector<Agent>& agents,
                      const CrowdParams& prm, float dt) {
    const Vec2 goalCenter{goal.x + 0.5f, goal.y + 0.5f};
    std::vector<Vec2> accel(agents.size(), Vec2{0, 0});
    for (size_t i = 0; i < agents.size(); ++i) {
        Agent& a = agents[i];
        Cell cell{static_cast<int>(std::floor(a.pos.x)), static_cast<int>(std::floor(a.pos.y))};
        float speed = prm.maxSpeed / (grid.passable(cell) ? static_cast<float>(grid.cost(cell)) : 1.0f);  // 地形减速

        // 1. 期望速度：沿流场；进入目标格子后直接奔向目标中心并减速
        Vec2 desired = sampleFlow(grid, flow, a.pos) * speed;
        if (cell == goal) desired = (goalCenter - a.pos) * 2.0f;

        // 2. 分离：离得太近的单位互相推开（O(n²)，单位很多时应改用空间哈希，见第二卷）
        Vec2 push{0, 0};
        for (size_t j = 0; j < agents.size(); ++j) {
            if (j == i) continue;
            Vec2 d = a.pos - agents[j].pos;
            float dist2 = dot(d, d), range = 2.5f * prm.radius;
            if (dist2 > range * range || dist2 < 1e-8f) continue;
            float dist = std::sqrt(dist2);
            push += d / dist * ((range - dist) / range);
        }
        Vec2 steer = desired - a.vel + push * prm.separation;
        float len = length(steer);
        if (len > prm.maxForce) steer = steer * (prm.maxForce / len);
        accel[i] = steer;
    }
    for (size_t i = 0; i < agents.size(); ++i) {
        Agent& a = agents[i];
        a.vel += accel[i] * dt;
        a.pos += a.vel * dt;

        // 3. 与墙和地图边界碰撞：把圆从周围的墙格子里推出去
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                Cell c{static_cast<int>(std::floor(a.pos.x)) + dx, static_cast<int>(std::floor(a.pos.y)) + dy};
                if (grid.passable(c)) continue;
                Vec2 closest{std::clamp(a.pos.x, float(c.x), float(c.x + 1)), std::clamp(a.pos.y, float(c.y), float(c.y + 1))};
                Vec2 d = a.pos - closest;
                float dist = length(d);
                if (dist >= prm.radius) continue;
                Vec2 n = dist > 1e-5f ? d / dist : normalized(a.pos - Vec2{c.x + 0.5f, c.y + 0.5f});
                a.pos = closest + n * prm.radius;
                a.vel -= n * std::min(0.0f, dot(a.vel, n));  // 去掉朝墙里的速度分量
            }
        }
    }
}
// #endregion

}  // namespace ga
