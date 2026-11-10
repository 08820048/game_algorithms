#pragma once

// 元胞自动机洞穴
//   - 随机填充：每个内部格子以概率 p 是墙，外圈永远是墙
//   - 平滑：每一轮按邻域里墙的数量重新决定每个格子。经典的"4-5 规则"等价于 3×3 多数表决
//   - 区域：洪水填充标记连通的地面；保留最大的一块，或者用最短的隧道把其余各块连上来
//   - 测量：张开半径（离最近的墙的切比雪夫距离）、边界长度
// 地图类型、BFS、死胡同、绕路比等沿用上一章的 dungeon.h：Wall 是墙，Room 是洞穴地面，Corridor 是挖出的隧道

#include <algorithm>
#include <cstdint>
#include <vector>

#include "../dungeon/dungeon.h"

namespace ga::cave {

using dungeon::Corridor;
using dungeon::Map;
using dungeon::Room;
using dungeon::Wall;

// #region fill
inline void randomFill(Map& m, Pcg32& rng, float wallProb) {
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x) {
            bool border = x == 0 || y == 0 || x == m.w - 1 || y == m.h - 1;
            m.at(x, y) = border || rng.nextFloat() < wallProb ? Wall : Room;
        }
}
// #endregion fill

// #region step
// 规则：n1 是 3×3 内墙的数量（含自己），n2 是 5×5 内墙的数量（含自己），地图外算墙。
//   wall1 = 5, wall2 = -1：经典 4-5 规则（墙有 ≥4 个墙邻居就保留，地面有 ≥5 个墙邻居就变墙），即 3×3 多数表决
//   wall2 = 2：再加上"n2 ≤ 2 也变墙"——在大片空地中间种出柱子（Jim Babcock 的变体）
struct Rule {
    int wall1 = 5;
    int wall2 = -1;
};

inline bool ruleSaysWall(const Rule& r, int n1, int n2) { return n1 >= r.wall1 || (r.wall2 >= 0 && n2 <= r.wall2); }

// 同步更新：所有格子都按上一轮的状态计算（先复制一份再数）。外圈保持是墙。返回改变的格子数。
// 内部格子的 3×3 邻域一定在地图内，直接数 9 个格子；5×5 只在规则用到时才数，越界的算墙
inline int step(Map& m, const Rule& rule) {
    const Map src = m;
    auto wall = [&](int x, int y) { return int(src.at(x, y) == Wall); };
    int changed = 0;
    for (int y = 1; y < m.h - 1; ++y)
        for (int x = 1; x < m.w - 1; ++x) {
            int n1 = 0;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) n1 += wall(x + dx, y + dy);
            int n2 = 99;
            if (rule.wall2 >= 0) {
                n2 = 0;
                for (int dy = -2; dy <= 2; ++dy)
                    for (int dx = -2; dx <= 2; ++dx) n2 += !src.inside(x + dx, y + dy) || wall(x + dx, y + dy);
            }
            uint8_t v = ruleSaysWall(rule, n1, n2) ? Wall : Room;
            changed += v != m.at(x, y);
            m.at(x, y) = v;
        }
    return changed;
}

// 原地更新（常见的错误写法）：按行扫描，直接改写同一张地图，于是左边和上边的邻居已经是"这一轮"的新值
inline int stepInPlace(Map& m, const Rule& rule) {
    auto wall = [&](int x, int y) { return !m.inside(x, y) || m.at(x, y) == Wall; };
    int changed = 0;
    for (int y = 1; y < m.h - 1; ++y)
        for (int x = 1; x < m.w - 1; ++x) {
            int n1 = 0, n2 = 0;
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx) {
                    int v = wall(x + dx, y + dy);
                    n2 += v;
                    if (std::abs(dx) <= 1 && std::abs(dy) <= 1) n1 += v;
                }
            uint8_t v = ruleSaysWall(rule, n1, n2) ? Wall : Room;
            changed += v != m.at(x, y);
            m.at(x, y) = v;
        }
    return changed;
}
// #endregion step

// 反复平滑直到出现重复的状态。返回轮数和周期（1 是不动点，2 是两个状态来回振荡）；maxIter 内没有重复时周期为 0
struct Settle {
    int iterations = 0, period = 0;
};
inline Settle settle(Map& m, const Rule& rule, int maxIter) {
    std::vector<uint8_t> prev2, prev1 = m.t;
    for (int i = 1; i <= maxIter; ++i) {
        step(m, rule);
        if (m.t == prev1) return {i, 1};
        if (m.t == prev2) return {i, 2};
        prev2 = std::move(prev1), prev1 = m.t;
    }
    return {maxIter, 0};
}

// #region regions
// 连通的地面区域（4 邻域）。label 对墙是 -1；size[k] 是第 k 块的格子数
struct Regions {
    std::vector<int> label, size;
    int largest() const { return int(std::max_element(size.begin(), size.end()) - size.begin()); }
};

inline Regions labelRegions(const Map& m) {
    Regions r;
    r.label.assign(m.t.size(), -1);
    std::vector<int> q;
    for (int i = 0; i < int(m.t.size()); ++i) {
        if (m.t[i] == Wall || r.label[i] >= 0) continue;
        int id = int(r.size.size());
        q.assign(1, i), r.label[i] = id;
        for (size_t h = 0; h < q.size(); ++h) {
            int x = q[h] % m.w, y = q[h] / m.w;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dungeon::kDx4[k], ny = y + dungeon::kDy4[k];
                if (m.floor(nx, ny) && r.label[ny * m.w + nx] < 0) r.label[ny * m.w + nx] = id, q.push_back(ny * m.w + nx);
            }
        }
        r.size.push_back(int(q.size()));
    }
    return r;
}

// 只保留最大的一块地面，其余填成墙。返回填掉的地面格子数
inline int keepLargest(Map& m) {
    Regions r = labelRegions(m);
    if (r.size.empty()) return 0;
    int keep = r.largest(), removed = 0;
    for (size_t i = 0; i < m.t.size(); ++i)
        if (r.label[i] >= 0 && r.label[i] != keep) m.t[i] = Wall, ++removed;
    return removed;
}

// 把小于 minSize 的墙块（4 邻域、不接触外圈的"柱子"）填成地面。返回填掉的墙格子数
inline int removePillars(Map& m, int minSize) {
    std::vector<char> seen(m.t.size(), 0);
    std::vector<int> q;
    int removed = 0;
    for (int i = 0; i < int(m.t.size()); ++i) {
        if (m.t[i] != Wall || seen[i]) continue;
        q.assign(1, i), seen[i] = 1;
        bool border = false;
        for (size_t h = 0; h < q.size(); ++h) {
            int x = q[h] % m.w, y = q[h] / m.w;
            border |= x == 0 || y == 0 || x == m.w - 1 || y == m.h - 1;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dungeon::kDx4[k], ny = y + dungeon::kDy4[k];
                if (m.inside(nx, ny) && m.at(nx, ny) == Wall && !seen[ny * m.w + nx]) seen[ny * m.w + nx] = 1, q.push_back(ny * m.w + nx);
            }
        }
        if (!border && int(q.size()) < minSize)
            for (int c : q) m.t[c] = Room, ++removed;
    }
    return removed;
}

// 把所有区域连到最大的一块上：从主区域出发做多源 BFS（墙和地面都能走），
// 第一次碰到别的区域的格子时，沿父指针把路上的墙挖成隧道，那一块就并入了主区域。重复直到只剩一块。
// BFS 距离就是曼哈顿距离，所以每次挖的墙数 = 主区域与最近的那块之间的最小曼哈顿距离 - 1。
// 这相当于在区域之间做 Prim 最小生成树（上一章房间与走廊用的也是它）。返回挖掉的墙格子数
inline int connectRegions(Map& m, std::vector<int>* tunnels = nullptr) {
    Regions r = labelRegions(m);
    if (r.size.size() <= 1) return 0;
    std::vector<char> main(m.t.size(), 0);
    int keep = r.largest();
    for (size_t i = 0; i < m.t.size(); ++i) main[i] = r.label[i] == keep;
    int carved = 0, left = int(r.size.size()) - 1;
    std::vector<int> parent(m.t.size()), q;
    while (left > 0) {
        std::fill(parent.begin(), parent.end(), -2);
        q.clear();
        for (int i = 0; i < int(m.t.size()); ++i)
            if (main[i]) parent[i] = -1, q.push_back(i);
        int hit = -1;
        for (size_t h = 0; h < q.size() && hit < 0; ++h) {
            int c = q[h], x = c % m.w, y = c / m.w;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dungeon::kDx4[k], ny = y + dungeon::kDy4[k];
                if (nx < 1 || ny < 1 || nx >= m.w - 1 || ny >= m.h - 1) continue;  // 不挖外圈（最短路总在两端的外接矩形内，这一行只是防御）
                int n = ny * m.w + nx;
                if (parent[n] != -2) continue;
                parent[n] = c;
                if (m.t[n] != Wall) {
                    hit = n;
                    break;
                }
                q.push_back(n);
            }
        }
        for (int c = parent[hit]; !main[c]; c = parent[c]) {  // 挖隧道
            m.t[c] = Corridor, main[c] = 1, ++carved;
            if (tunnels) tunnels->push_back(c);
        }
        // 碰到的那一块整体并入主区域
        int id = r.label[hit];
        for (size_t i = 0; i < m.t.size(); ++i)
            if (r.label[i] == id) main[i] = 1;
        --left;
    }
    return carved;
}
// #endregion regions

// #region measure
// 每个地面格子离最近的墙（含地图外）的切比雪夫距离：8 邻域多源 BFS。紧贴墙的格子是 1
inline std::vector<int> openRadius(const Map& m) {
    std::vector<int> d(m.t.size(), -1), q;
    for (int i = 0; i < int(m.t.size()); ++i)
        if (m.t[i] == Wall) d[i] = 0, q.push_back(i);
    for (int y = 0; y < m.h; ++y)  // 外圈上的地面紧贴地图外，距离 1。必须排在所有墙之后，队列才按距离有序
        for (int x = 0; x < m.w; ++x)
            if (d[y * m.w + x] < 0 && (x == 0 || y == 0 || x == m.w - 1 || y == m.h - 1)) d[y * m.w + x] = 1, q.push_back(y * m.w + x);
    for (size_t h = 0; h < q.size(); ++h) {
        int c = q[h], x = c % m.w, y = c / m.w;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                int nx = x + dx, ny = y + dy;
                if (!m.inside(nx, ny) || d[ny * m.w + nx] >= 0) continue;
                d[ny * m.w + nx] = d[c] + 1, q.push_back(ny * m.w + nx);
            }
    }
    return d;
}

// 地面与墙相邻的边数 / 地面格子数：越小边界越平滑
inline double edgePerFloor(const Map& m) {
    long long e = 0;
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x)
            if (m.floor(x, y))
                for (int k = 0; k < 4; ++k) e += !m.floor(x + dungeon::kDx4[k], y + dungeon::kDy4[k]);
    int f = m.floorCount();
    return f ? double(e) / f : 0;
}
// #endregion measure

}  // namespace ga::cave
