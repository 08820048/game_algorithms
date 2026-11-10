#pragma once

// 地牢生成
//   - BSP：把地图递归地一分为二，每个叶子里放一个房间，再沿着分割树把兄弟子树的房间用走廊连起来
//   - 随机游走（醉汉走法）：一个"挖掘者"随机走动，走过的墙变成地面，直到地面达到目标比例
//   - 房间与走廊：随机放置不重叠的房间，用最小生成树连通，再加回几条边形成环路
//   - 分析：连通分量、死胡同、BFS 距离、直径（精确值与两次 BFS 的近似）、绕路比

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <queue>
#include <utility>
#include <vector>

#include "../../foundations/random/random.h"

namespace ga::dungeon {

enum Tile : uint8_t { Wall = 0, Room = 1, Corridor = 2 };  // Room 与 Corridor 都是地面，分开只为了着色和统计

struct Rect {
    int x = 0, y = 0, w = 0, h = 0;
    int cx() const { return x + w / 2; }
    int cy() const { return y + h / 2; }
    // 两个矩形之间至少隔 pad 个格子时返回 false
    bool overlaps(const Rect& o, int pad) const {
        return x - pad < o.x + o.w && o.x - pad < x + w && y - pad < o.y + o.h && o.y - pad < y + h;
    }
};

struct Map {
    int w = 0, h = 0;
    std::vector<uint8_t> t;
    Map(int w_, int h_) : w(w_), h(h_), t(size_t(w_) * h_, Wall) {}
    uint8_t& at(int x, int y) { return t[size_t(y) * w + x]; }
    uint8_t at(int x, int y) const { return t[size_t(y) * w + x]; }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
    bool floor(int x, int y) const { return inside(x, y) && at(x, y) != Wall; }
    int floorCount() const {
        int n = 0;
        for (uint8_t v : t) n += v != Wall;
        return n;
    }
};

// #region carve
inline void carveRoom(Map& m, const Rect& r) {
    for (int y = r.y; y < r.y + r.h; ++y)
        for (int x = r.x; x < r.x + r.w; ++x) m.at(x, y) = Room;
}

// L 形走廊：从 (ax, ay) 先水平再竖直（或反过来）走到 (bx, by)。只把墙改成走廊，不覆盖房间
inline void carveCorridor(Map& m, int ax, int ay, int bx, int by, bool horizontalFirst) {
    auto dig = [&](int x, int y) {
        if (m.at(x, y) == Wall) m.at(x, y) = Corridor;
    };
    if (horizontalFirst) {  // (ax, ay) → (bx, ay) → (bx, by)
        for (int x = std::min(ax, bx); x <= std::max(ax, bx); ++x) dig(x, ay);
        for (int y = std::min(ay, by); y <= std::max(ay, by); ++y) dig(bx, y);
    } else {  // (ax, ay) → (ax, by) → (bx, by)
        for (int y = std::min(ay, by); y <= std::max(ay, by); ++y) dig(ax, y);
        for (int x = std::min(ax, bx); x <= std::max(ax, bx); ++x) dig(x, by);
    }
}
// #endregion carve

// #region bsp
struct BspParams {
    int minLeaf = 8;          // 叶子的最小边长
    float minRatio = 0.3f;    // 切割位置在 [minRatio, 1 - minRatio] 之间
    bool longAxis = true;     // 沿长边切；false 时在允许的方向里随机选
    bool closest = true;      // 连接兄弟子树时选距离最近的一对房间；false 时各随机选一个
    int margin = 1;           // 房间与叶子边界的距离
    int minRoom = 3;          // 房间的最小边长
    int maxDepth = 99;        // 最多切几层（常见写法是固定层数，而不是切到不能再切）
};

struct BspNode {
    Rect area;
    int left = -1, right = -1, parent = -1, depth = 0;
    Rect room{};       // 只有叶子有房间
    bool vertical = false;  // 内部节点：true 表示竖着切（切 x）
};

struct Link {
    int ax, ay, bx, by;
    bool horizontalFirst;
};

struct BspResult {
    std::vector<BspNode> nodes;  // 按创建顺序（广度优先），nodes[0] 是整张地图
    std::vector<int> leaves;
    std::vector<Link> corridors;  // 按连接顺序（自底向上）
};

inline BspResult generateBsp(Map& m, Pcg32& rng, const BspParams& p) {
    BspResult r;
    r.nodes.push_back({Rect{1, 1, m.w - 2, m.h - 2}});  // 外圈留一层墙
    for (size_t i = 0; i < r.nodes.size(); ++i) {
        Rect a = r.nodes[i].area;
        bool deep = r.nodes[i].depth >= p.maxDepth;
        bool canV = !deep && a.w >= 2 * p.minLeaf, canH = !deep && a.h >= 2 * p.minLeaf;
        if (!canV && !canH) {
            r.leaves.push_back(int(i));
            continue;
        }
        bool vertical;
        if (canV && canH) {
            if (p.longAxis && a.w > a.h * 1.25f) vertical = true;
            else if (p.longAxis && a.h > a.w * 1.25f) vertical = false;
            else vertical = rng.chance(0.5);
        } else {
            vertical = canV;
        }
        int len = vertical ? a.w : a.h;
        int lo = std::max(p.minLeaf, int(len * p.minRatio + 0.5f)), hi = std::min(len - p.minLeaf, int(len * (1 - p.minRatio) + 0.5f));
        if (lo > hi) lo = hi = len / 2;
        int cut = rng.range(lo, hi);
        Rect A = a, B = a;
        if (vertical) A.w = cut, B.x += cut, B.w -= cut;
        else A.h = cut, B.y += cut, B.h -= cut;
        int d = r.nodes[i].depth + 1;
        r.nodes[i].vertical = vertical;
        r.nodes[i].left = int(r.nodes.size());
        r.nodes.push_back({A, -1, -1, int(i), d});
        r.nodes[i].right = int(r.nodes.size());
        r.nodes.push_back({B, -1, -1, int(i), d});
    }
    // 每个叶子里放一个随机大小、随机位置的房间
    for (int li : r.leaves) {
        Rect a = r.nodes[li].area;
        int maxW = a.w - 2 * p.margin, maxH = a.h - 2 * p.margin;
        Rect room;
        room.w = rng.range(std::min(p.minRoom, maxW), maxW);
        room.h = rng.range(std::min(p.minRoom, maxH), maxH);
        room.x = a.x + p.margin + rng.range(0, maxW - room.w);
        room.y = a.y + p.margin + rng.range(0, maxH - room.h);
        r.nodes[li].room = room;
        carveRoom(m, room);
    }
    // 自底向上：每个内部节点把左右子树各一个房间连起来。节点按广度优先创建，倒序遍历就是先子后父
    auto roomsUnder = [&](int n) {
        std::vector<int> out, stack{n};
        while (!stack.empty()) {
            int k = stack.back();
            stack.pop_back();
            if (r.nodes[k].left < 0) out.push_back(k);
            else stack.push_back(r.nodes[k].left), stack.push_back(r.nodes[k].right);
        }
        return out;
    };
    for (int i = int(r.nodes.size()) - 1; i >= 0; --i) {
        if (r.nodes[i].left < 0) continue;
        std::vector<int> L = roomsUnder(r.nodes[i].left), R = roomsUnder(r.nodes[i].right);
        int a = L[rng.below(uint32_t(L.size()))], b = R[rng.below(uint32_t(R.size()))];
        if (p.closest) {
            int best = 1 << 30;
            for (int u : L)
                for (int v : R) {
                    const Rect &ru = r.nodes[u].room, &rv = r.nodes[v].room;
                    int d = std::abs(ru.cx() - rv.cx()) + std::abs(ru.cy() - rv.cy());
                    if (d < best) best = d, a = u, b = v;
                }
        }
        const Rect &ra = r.nodes[a].room, &rb = r.nodes[b].room;
        Link l{ra.cx(), ra.cy(), rb.cx(), rb.cy(), rng.chance(0.5)};
        carveCorridor(m, l.ax, l.ay, l.bx, l.by, l.horizontalFirst);
        r.corridors.push_back(l);
    }
    return r;
}
// #endregion bsp

// #region walk
struct WalkParams {
    double coverage = 0.4;  // 目标：内部格子中地面所占的比例
    int restartEvery = 0;   // > 0：每走这么多步，跳到一个随机的已有地面格子重新开始
    long long maxSteps = 50'000'000;
};

// 返回走了多少步。外圈一层永远是墙
inline long long randomWalk(Map& m, Pcg32& rng, const WalkParams& p, std::vector<int>* trace = nullptr) {
    const int target = int(p.coverage * (m.w - 2) * (m.h - 2) + 0.5);
    int x = m.w / 2, y = m.h / 2, carved = 0;
    std::vector<int> floorCells;
    auto dig = [&] {
        if (m.at(x, y) == Wall) {
            m.at(x, y) = Corridor, ++carved;
            floorCells.push_back(y * m.w + x);
            if (trace) trace->push_back(y * m.w + x);
        }
    };
    dig();
    const int dx[4] = {1, 0, -1, 0}, dy[4] = {0, 1, 0, -1};
    long long steps = 0;
    while (carved < target && steps < p.maxSteps) {
        if (p.restartEvery > 0 && steps > 0 && steps % p.restartEvery == 0) {
            int c = floorCells[rng.below(uint32_t(floorCells.size()))];
            x = c % m.w, y = c / m.w;
        }
        int k = int(rng.below(4));
        int nx = x + dx[k], ny = y + dy[k];
        ++steps;
        if (nx < 1 || ny < 1 || nx > m.w - 2 || ny > m.h - 2) continue;  // 撞到边界：这一步作废
        x = nx, y = ny;
        dig();
    }
    return steps;
}
// #endregion walk

// #region rooms
struct RoomsParams {
    int attempts = 300;   // 尝试放置多少次
    int minSize = 4, maxSize = 11;
    int spacing = 2;      // 房间之间至少隔几个格子
    double loops = 0.15;  // 在生成树之外再加回的边数，占房间数的比例
};

struct Edge {
    int a, b, len;
};

struct RoomsResult {
    std::vector<Rect> rooms;
    std::vector<Edge> tree, extra;
};

inline int roomDistance(const Rect& a, const Rect& b) { return std::abs(a.cx() - b.cx()) + std::abs(a.cy() - b.cy()); }

// Prim：完全图上 O(n²)，房间数不多时比先建边表再排序的 Kruskal 更简单
inline std::vector<Edge> minimumSpanningTree(const std::vector<Rect>& rooms) {
    const int n = int(rooms.size());
    std::vector<Edge> tree;
    if (n == 0) return tree;
    std::vector<int> best(n, 1 << 30), from(n, -1);
    std::vector<char> in(n, 0);
    best[0] = 0;
    for (int it = 0; it < n; ++it) {
        int u = -1;
        for (int i = 0; i < n; ++i)
            if (!in[i] && (u < 0 || best[i] < best[u])) u = i;
        in[u] = 1;
        if (from[u] >= 0) tree.push_back({from[u], u, best[u]});
        for (int v = 0; v < n; ++v) {
            int d = roomDistance(rooms[u], rooms[v]);
            if (!in[v] && d < best[v]) best[v] = d, from[v] = u;
        }
    }
    return tree;
}

inline RoomsResult generateRooms(Map& m, Pcg32& rng, const RoomsParams& p) {
    RoomsResult r;
    for (int k = 0; k < p.attempts; ++k) {
        Rect c;
        c.w = rng.range(p.minSize, p.maxSize), c.h = rng.range(p.minSize, p.maxSize);
        c.x = rng.range(1, m.w - 1 - c.w), c.y = rng.range(1, m.h - 1 - c.h);
        bool ok = true;
        for (const Rect& o : r.rooms) ok = ok && !c.overlaps(o, p.spacing);
        if (ok) r.rooms.push_back(c);
    }
    for (const Rect& room : r.rooms) carveRoom(m, room);
    r.tree = minimumSpanningTree(r.rooms);
    // 加回最短的若干条非树边：形成环路，玩家不必总是原路返回
    std::vector<Edge> rest;
    const int n = int(r.rooms.size());
    std::vector<char> inTree(size_t(n) * n, 0);
    for (const Edge& e : r.tree) inTree[size_t(e.a) * n + e.b] = inTree[size_t(e.b) * n + e.a] = 1;
    for (int a = 0; a < n; ++a)
        for (int b = a + 1; b < n; ++b)
            if (!inTree[size_t(a) * n + b]) rest.push_back({a, b, roomDistance(r.rooms[a], r.rooms[b])});
    std::sort(rest.begin(), rest.end(), [](const Edge& x, const Edge& y) { return x.len < y.len; });
    int want = std::min(int(rest.size()), int(p.loops * n + 0.5));
    r.extra.assign(rest.begin(), rest.begin() + want);
    for (const auto* list : {&r.tree, &r.extra})
        for (const Edge& e : *list) {
            const Rect &a = r.rooms[e.a], &b = r.rooms[e.b];
            carveCorridor(m, a.cx(), a.cy(), b.cx(), b.cy(), rng.chance(0.5));
        }
    return r;
}
// #endregion rooms

// #region analyze
constexpr int kDx4[4] = {1, 0, -1, 0}, kDy4[4] = {0, 1, 0, -1};

// 从 start 出发的 BFS 距离（4 邻域），到不了的是 -1
inline std::vector<int> bfs(const Map& m, int start) {
    std::vector<int> d(m.t.size(), -1);
    std::vector<int> q{start};
    d[start] = 0;
    for (size_t h = 0; h < q.size(); ++h) {
        int c = q[h], x = c % m.w, y = c / m.w;
        for (int k = 0; k < 4; ++k) {
            int nx = x + kDx4[k], ny = y + kDy4[k];
            if (m.floor(nx, ny) && d[ny * m.w + nx] < 0) d[ny * m.w + nx] = d[c] + 1, q.push_back(ny * m.w + nx);
        }
    }
    return d;
}

// 连通分量：每遇到一个还没标记的地面格子，就从它开始做一次洪水填充
inline int components(const Map& m) {
    std::vector<char> seen(m.t.size(), 0);
    std::vector<int> q;
    int n = 0;
    for (int i = 0; i < int(m.t.size()); ++i) {
        if (m.t[i] == Wall || seen[i]) continue;
        ++n;
        q.assign(1, i), seen[i] = 1;
        for (size_t h = 0; h < q.size(); ++h) {
            int x = q[h] % m.w, y = q[h] / m.w;
            for (int k = 0; k < 4; ++k) {
                int nx = x + kDx4[k], ny = y + kDy4[k];
                if (m.floor(nx, ny) && !seen[ny * m.w + nx]) seen[ny * m.w + nx] = 1, q.push_back(ny * m.w + nx);
            }
        }
    }
    return n;
}

// 死胡同：只有一个地面邻居的地面格子
inline int deadEnds(const Map& m) {
    int n = 0;
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x) {
            if (!m.floor(x, y)) continue;
            int k = 0;
            for (int d = 0; d < 4; ++d) k += m.floor(x + kDx4[d], y + kDy4[d]);
            n += k == 1;
        }
    return n;
}

struct Diameter {
    int length = 0, a = -1, b = -1;
};

// 两次 BFS：从任意格子出发找到最远的 a，再从 a 出发找最远的 b。在树上是精确的，一般的图上只是下界
inline Diameter doubleSweep(const Map& m, int start) {
    auto farthest = [&](int s) {
        std::vector<int> d = bfs(m, s);
        int best = s;
        for (size_t i = 0; i < d.size(); ++i)
            if (d[i] > d[best]) best = int(i);
        return std::make_pair(best, d[best]);
    };
    auto [a, da] = farthest(start);
    auto [b, db] = farthest(a);
    (void)da;
    return {db, a, b};
}

// 精确直径：从每个地面格子做一次 BFS，O(地面格子数²)
inline Diameter exactDiameter(const Map& m) {
    Diameter best;
    for (int i = 0; i < int(m.t.size()); ++i) {
        if (m.t[i] == Wall) continue;
        std::vector<int> d = bfs(m, i);
        for (size_t j = 0; j < d.size(); ++j)
            if (d[j] > best.length) best = {d[j], i, int(j)};
    }
    return best;
}
// 绕路比：随机取成对的地面格子（曼哈顿距离至少 minDist），BFS 距离 / 曼哈顿距离的平均值。
// 1 表示几乎可以直线走到；越大说明越要绕路（树状的地牢常常要原路返回）
inline double detourRatio(const Map& m, Pcg32& rng, int sources, int targetsPerSource, int minDist) {
    std::vector<int> cells;
    for (int i = 0; i < int(m.t.size()); ++i)
        if (m.t[i] != Wall) cells.push_back(i);
    double sum = 0;
    int n = 0;
    for (int s = 0; s < sources; ++s) {
        int a = cells[rng.below(uint32_t(cells.size()))];
        std::vector<int> d = bfs(m, a);
        for (int t = 0, tries = 0; t < targetsPerSource && tries < targetsPerSource * 20; ++tries) {
            int b = cells[rng.below(uint32_t(cells.size()))];
            int man = std::abs(a % m.w - b % m.w) + std::abs(a / m.w - b / m.w);
            if (man < minDist || d[b] < 0) continue;
            sum += double(d[b]) / man, ++n, ++t;
        }
    }
    return n ? sum / n : 0;
}
// #endregion analyze

}  // namespace ga::dungeon
