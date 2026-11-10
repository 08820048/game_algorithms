#include "dungeon.h"

#include <cstdio>
#include <numeric>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::dungeon;

// ---- 参照写法 ----
struct UnionFind {
    std::vector<int> p;
    explicit UnionFind(int n) : p(n) { std::iota(p.begin(), p.end(), 0); }
    int find(int x) { return p[x] == x ? x : p[x] = find(p[x]); }
    bool unite(int a, int b) {
        a = find(a), b = find(b);
        if (a == b) return false;
        p[a] = b;
        return true;
    }
};

static int componentsUF(const Map& m) {
    UnionFind uf(int(m.t.size()));
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x) {
            if (!m.floor(x, y)) continue;
            if (m.floor(x + 1, y)) uf.unite(y * m.w + x, y * m.w + x + 1);
            if (m.floor(x, y + 1)) uf.unite(y * m.w + x, (y + 1) * m.w + x);
        }
    int n = 0;
    for (int i = 0; i < int(m.t.size()); ++i) n += m.t[i] != Wall && uf.find(i) == i;
    return n;
}

// Floyd–Warshall 求所有点对最短路，取最大的有限值
static int diameterFloyd(const Map& m) {
    std::vector<int> cells;
    for (int i = 0; i < int(m.t.size()); ++i)
        if (m.t[i] != Wall) cells.push_back(i);
    const int n = int(cells.size()), INF = 1 << 28;
    std::vector<int> d(size_t(n) * n, INF);
    for (int i = 0; i < n; ++i) {
        d[size_t(i) * n + i] = 0;
        for (int j = 0; j < n; ++j) {
            int a = cells[i], b = cells[j];
            if (std::abs(a % m.w - b % m.w) + std::abs(a / m.w - b / m.w) == 1) d[size_t(i) * n + j] = 1;
        }
    }
    for (int k = 0; k < n; ++k)
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) d[size_t(i) * n + j] = std::min(d[size_t(i) * n + j], d[size_t(i) * n + k] + d[size_t(k) * n + j]);
    int best = 0;
    for (int v : d)
        if (v < INF) best = std::max(best, v);
    return best;
}

static int kruskalWeight(const std::vector<Rect>& rooms) {
    std::vector<Edge> e;
    for (int a = 0; a < int(rooms.size()); ++a)
        for (int b = a + 1; b < int(rooms.size()); ++b) e.push_back({a, b, roomDistance(rooms[a], rooms[b])});
    std::sort(e.begin(), e.end(), [](const Edge& x, const Edge& y) { return x.len < y.len; });
    UnionFind uf(int(rooms.size()));
    int w = 0;
    for (const Edge& x : e)
        if (uf.unite(x.a, x.b)) w += x.len;
    return w;
}

// 完美迷宫（递归回溯）：地面构成一棵树
static Map perfectMaze(int cw, int ch, Pcg32& rng) {
    Map m(cw * 2 + 1, ch * 2 + 1);
    std::vector<char> seen(size_t(cw) * ch, 0);
    std::vector<int> stack{0};
    seen[0] = 1, m.at(1, 1) = Corridor;
    while (!stack.empty()) {
        int c = stack.back(), x = c % cw, y = c / cw;
        int opts[4], n = 0;
        for (int k = 0; k < 4; ++k) {
            int nx = x + kDx4[k], ny = y + kDy4[k];
            if (nx >= 0 && ny >= 0 && nx < cw && ny < ch && !seen[ny * cw + nx]) opts[n++] = k;
        }
        if (!n) {
            stack.pop_back();
            continue;
        }
        int k = opts[rng.below(uint32_t(n))], nx = x + kDx4[k], ny = y + kDy4[k];
        seen[ny * cw + nx] = 1;
        m.at(2 * x + 1 + kDx4[k], 2 * y + 1 + kDy4[k]) = Corridor, m.at(2 * nx + 1, 2 * ny + 1) = Corridor;
        stack.push_back(ny * cw + nx);
    }
    return m;
}

static bool borderIsWall(const Map& m) {
    for (int x = 0; x < m.w; ++x)
        if (m.at(x, 0) != Wall || m.at(x, m.h - 1) != Wall) return false;
    for (int y = 0; y < m.h; ++y)
        if (m.at(0, y) != Wall || m.at(m.w - 1, y) != Wall) return false;
    return true;
}

static void testCarve() {
    Pcg32 rng(1);
    for (int k = 0; k < 500; ++k) {
        Map m(30, 20);
        int ax = rng.range(1, 28), ay = rng.range(1, 18), bx = rng.range(1, 28), by = rng.range(1, 18);
        bool hf = rng.chance(0.5);
        carveCorridor(m, ax, ay, bx, by, hf);
        CHECK(m.floorCount() == std::abs(ax - bx) + std::abs(ay - by) + 1);
        std::vector<int> d = bfs(m, ay * m.w + ax);
        CHECK(d[by * m.w + bx] == std::abs(ax - bx) + std::abs(ay - by));  // 走廊本身就是一条最短路
        // 拐角在 (bx, ay) 或 (ax, by)
        CHECK(m.floor(hf ? bx : ax, hf ? ay : by));
    }
    // 走廊不覆盖房间
    Map m(20, 10);
    carveRoom(m, {5, 2, 4, 4});
    carveCorridor(m, 1, 3, 15, 3, true);
    CHECK(m.at(6, 3) == Room && m.at(2, 3) == Corridor && m.at(12, 3) == Corridor);
}

static void testBsp() {
    for (bool longAxis : {true, false})
        for (bool closest : {true, false})
            for (uint64_t seed = 1; seed <= 40; ++seed) {
                Map m(72, 44);
                Pcg32 rng(seed);
                BspParams p;
                p.longAxis = longAxis, p.closest = closest;
                BspResult r = generateBsp(m, rng, p);
                // 叶子恰好覆盖内部区域一次；每个内部节点的两个孩子恰好拼成它
                std::vector<int> cover(m.t.size(), 0);
                for (int li : r.leaves) {
                    const Rect& a = r.nodes[li].area;
                    CHECK(a.w >= p.minLeaf && a.h >= p.minLeaf);
                    CHECK(a.w < 2 * p.minLeaf && a.h < 2 * p.minLeaf);  // 不能再分才是叶子
                    for (int y = a.y; y < a.y + a.h; ++y)
                        for (int x = a.x; x < a.x + a.w; ++x) ++cover[y * m.w + x];
                    // 房间在叶子内，离边界至少 margin
                    const Rect& room = r.nodes[li].room;
                    CHECK(room.w >= p.minRoom && room.h >= p.minRoom);
                    CHECK(room.x >= a.x + p.margin && room.y >= a.y + p.margin);
                    CHECK(room.x + room.w <= a.x + a.w - p.margin && room.y + room.h <= a.y + a.h - p.margin);
                }
                for (int y = 0; y < m.h; ++y)
                    for (int x = 0; x < m.w; ++x) CHECK(cover[y * m.w + x] == (x >= 1 && y >= 1 && x < m.w - 1 && y < m.h - 1 ? 1 : 0));
                for (const BspNode& n : r.nodes) {
                    if (n.left < 0) continue;
                    const Rect &A = r.nodes[n.left].area, &B = r.nodes[n.right].area;
                    CHECK(A.w * A.h + B.w * B.h == n.area.w * n.area.h);
                    if (n.vertical) CHECK(A.x + A.w == B.x && A.y == B.y && A.h == n.area.h);
                    else CHECK(A.y + A.h == B.y && A.x == B.x && A.w == n.area.w);
                }
                // 走廊数 = 内部节点数 = 叶子数 - 1；整张图连通；外圈是墙
                CHECK(r.corridors.size() == r.leaves.size() - 1);
                CHECK(components(m) == 1);
                CHECK(borderIsWall(m));
            }
    // 限制层数：叶子的深度都不超过 maxDepth，叶子数不超过 2^maxDepth
    for (uint64_t seed = 1; seed <= 20; ++seed) {
        Map m(72, 44);
        Pcg32 rng(seed);
        BspParams p;
        p.maxDepth = 3;
        BspResult r = generateBsp(m, rng, p);
        CHECK(r.leaves.size() <= 8);
        for (int li : r.leaves) CHECK(r.nodes[li].depth <= 3);
        CHECK(components(m) == 1);
    }
    // 同一个种子得到同样的地图
    Map a(60, 40), b(60, 40);
    Pcg32 r1(7), r2(7);
    generateBsp(a, r1, {}), generateBsp(b, r2, {});
    CHECK(a.t == b.t);
}

static void testWalk() {
    for (int restart : {0, 50})
        for (double cov : {0.1, 0.3, 0.5})
            for (uint64_t seed = 1; seed <= 10; ++seed) {
                Map m(50, 30);
                Pcg32 rng(seed);
                WalkParams p;
                p.coverage = cov, p.restartEvery = restart;
                std::vector<int> trace;
                long long steps = randomWalk(m, rng, p, &trace);
                int target = int(cov * 48 * 28 + 0.5);
                CHECK(m.floorCount() == target);
                CHECK(int(trace.size()) == target);
                CHECK(steps >= target - 1);
                CHECK(components(m) == 1);
                CHECK(borderIsWall(m));
            }
}

static void testRooms() {
    for (uint64_t seed = 1; seed <= 40; ++seed) {
        Map m(80, 50);
        Pcg32 rng(seed);
        RoomsParams p;
        RoomsResult r = generateRooms(m, rng, p);
        const int n = int(r.rooms.size());
        CHECK(n >= 10);
        // 两两之间至少隔 spacing 格：逐格检查两个房间的距离（切比雪夫意义下的间隙）
        for (int i = 0; i < n; ++i) {
            const Rect& a = r.rooms[i];
            CHECK(a.x >= 1 && a.y >= 1 && a.x + a.w <= m.w - 1 && a.y + a.h <= m.h - 1);
            for (int j = i + 1; j < n; ++j) {
                const Rect& b = r.rooms[j];
                int gapX = std::max(b.x - (a.x + a.w), a.x - (b.x + b.w));
                int gapY = std::max(b.y - (a.y + a.h), a.y - (b.y + b.h));
                CHECK(std::max(gapX, gapY) >= p.spacing);
            }
        }
        // 生成树：n - 1 条边，总长与 Kruskal 相同
        CHECK(int(r.tree.size()) == n - 1);
        int w = 0;
        for (const Edge& e : r.tree) w += e.len;
        CHECK(w == kruskalWeight(r.rooms));
        UnionFind uf(n);
        int merged = 0;
        for (const Edge& e : r.tree) merged += uf.unite(e.a, e.b);
        CHECK(merged == n - 1);
        // 额外的边都不在树里，数量按比例
        CHECK(int(r.extra.size()) == int(p.loops * n + 0.5));
        for (const Edge& e : r.extra)
            for (const Edge& t : r.tree) CHECK(!((e.a == t.a && e.b == t.b) || (e.a == t.b && e.b == t.a)));
        CHECK(components(m) == 1);
        CHECK(borderIsWall(m));
    }
}

static void testAnalysis() {
    Pcg32 rng(3);
    // 连通分量与并查集一致
    for (int k = 0; k < 200; ++k) {
        Map m(24, 16);
        for (int y = 1; y < 15; ++y)
            for (int x = 1; x < 23; ++x) m.at(x, y) = rng.chance(0.55) ? Corridor : Wall;
        CHECK(components(m) == componentsUF(m));
    }
    // 直径：精确值与 Floyd–Warshall 一致；两次 BFS 不超过精确值
    int sweepExact = 0, trials = 0;
    for (int k = 0; k < 40; ++k) {
        Map m(18, 12);
        Pcg32 r(100 + k);
        if (k % 2) {
            RoomsParams p;
            p.minSize = 2, p.maxSize = 4, p.spacing = 1, p.attempts = 40, p.loops = 0.5;
            generateRooms(m, r, p);
        } else {
            WalkParams p;
            p.coverage = 0.45;
            randomWalk(m, r, p);
        }
        Diameter e = exactDiameter(m);
        CHECK(e.length == diameterFloyd(m));
        std::vector<int> d = bfs(m, e.a);
        CHECK(d[e.b] == e.length);
        int start = 0;
        while (m.t[start] == Wall) ++start;
        Diameter s = doubleSweep(m, start);
        CHECK(s.length <= e.length);
        sweepExact += s.length == e.length, ++trials;
    }
    CHECK(sweepExact < trials);  // 有环的图上两次 BFS 不总是精确
    // 树（完美迷宫）上两次 BFS 是精确的
    for (int k = 0; k < 30; ++k) {
        Map m = perfectMaze(9, 6, rng);
        CHECK(components(m) == 1);
        CHECK(doubleSweep(m, 1 * m.w + 1).length == exactDiameter(m).length);
        CHECK(exactDiameter(m).length == diameterFloyd(m));
    }
    // 死胡同：十字形有 4 个，一条直线有 2 个，一个房间没有
    Map plus(9, 9);
    carveCorridor(plus, 1, 4, 7, 4, true), carveCorridor(plus, 4, 1, 4, 7, true);
    CHECK(deadEnds(plus) == 4);
    Map line(9, 3);
    carveCorridor(line, 1, 1, 7, 1, true);
    CHECK(deadEnds(line) == 2);
    Map room(9, 9);
    carveRoom(room, {1, 1, 7, 7});
    CHECK(deadEnds(room) == 0);
    Map dot(5, 5);  // 孤立的一个格子没有通路，不算死胡同
    dot.at(2, 2) = Room;
    CHECK(deadEnds(dot) == 0);
    // 绕路比：空旷的房间里正好是 1；迷宫里大于 1
    Map open(30, 20);
    carveRoom(open, {1, 1, 28, 18});
    CHECK(detourRatio(open, rng, 20, 20, 5) == 1.0);
    Map maze = perfectMaze(15, 10, rng);
    CHECK(detourRatio(maze, rng, 20, 20, 5) > 1.5);
}

int main() {
    testCarve();
    testBsp();
    testWalk();
    testRooms();
    testAnalysis();
    return report("dungeon_test");
}
