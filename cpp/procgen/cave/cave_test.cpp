#include "cave.h"

#include <cstdio>
#include <map>
#include <numeric>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::cave;

// ---- 参照写法 ----
static bool wallAt(const Map& m, int x, int y) { return !m.inside(x, y) || m.at(x, y) == Wall; }

// 逐格数邻居、写到新地图的同步更新
static Map stepNaive(const Map& m, const Rule& r) {
    Map out = m;
    for (int y = 1; y < m.h - 1; ++y)
        for (int x = 1; x < m.w - 1; ++x) {
            int n1 = 0, n2 = 0;
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx) {
                    n2 += wallAt(m, x + dx, y + dy);
                    if (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1) n1 += wallAt(m, x + dx, y + dy);
                }
            out.at(x, y) = (n1 >= r.wall1 || (r.wall2 >= 0 && n2 <= r.wall2)) ? Wall : Room;
        }
    return out;
}

// 文献里的说法：墙有 ≥4 个墙邻居（不含自己）就保留，地面有 ≥5 个墙邻居就变墙
static Map stepFourFive(const Map& m) {
    Map out = m;
    for (int y = 1; y < m.h - 1; ++y)
        for (int x = 1; x < m.w - 1; ++x) {
            int n = 0;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    if (dx || dy) n += wallAt(m, x + dx, y + dy);
            out.at(x, y) = m.at(x, y) == Wall ? (n >= 4 ? Wall : Room) : (n >= 5 ? Wall : Room);
        }
    return out;
}

struct UnionFind {
    std::vector<int> p;
    explicit UnionFind(int n) : p(n) { std::iota(p.begin(), p.end(), 0); }
    int find(int x) { return p[x] == x ? x : p[x] = find(p[x]); }
    void unite(int a, int b) { p[find(a)] = find(b); }
};

static Map randomMap(int w, int h, uint64_t seed, float p) {
    Map m(w, h);
    Pcg32 rng{seed};
    randomFill(m, rng, p);
    return m;
}

static void testStep() {
    const Rule rules[] = {{5, -1}, {5, 2}, {4, -1}, {6, -1}, {5, 5}};
    int changedTotal = 0;
    for (int s = 0; s < 60; ++s) {
        int w = 3 + s % 17, h = 3 + (s * 7) % 13;
        for (const Rule& r : rules) {
            Map m = randomMap(w, h, 100 + s, 0.3f + 0.03f * (s % 8));
            for (int it = 0; it < 4; ++it) {
                Map want = stepNaive(m, r);
                int diff = 0;
                for (size_t i = 0; i < m.t.size(); ++i) diff += m.t[i] != want.t[i];
                int changed = step(m, r);
                CHECK(m.t == want.t);
                CHECK(changed == diff);
                changedTotal += changed;
            }
        }
        // 4-5 规则 = 3×3 多数表决
        Map a = randomMap(w, h, 900 + s, 0.45f);
        Map b = stepFourFive(a);
        step(a, Rule{});
        CHECK(a.t == b.t);
    }
    CHECK(changedTotal > 1000);
    // 外圈不变
    Map m = randomMap(20, 12, 7, 0.0f);  // 内部全是地面
    step(m, Rule{5, 2});
    for (int x = 0; x < 20; ++x) CHECK(m.at(x, 0) == Wall && m.at(x, 11) == Wall);
    CHECK(m.at(10, 6) == Wall);  // 5×5 内全是地面 → 种出柱子
    CHECK(m.at(2, 2) == Room);   // n1 = 0，但 5×5 里有 9 格墙
    CHECK(m.at(1, 1) == Wall);   // 角落：3×3 里有 5 格墙
}

static void testSettle() {
    // Goles–Olivos：对称阈值网络的同步更新最终是不动点或周期 2。用完整历史检测环长
    int period2 = 0, n = 0;
    for (int s = 0; s < 400; ++s) {
        Map m = randomMap(4 + s % 9, 4 + (s / 9) % 7, 3000 + s, 0.35f + 0.05f * (s % 5));
        Map copy = m;
        std::map<std::vector<uint8_t>, int> seen;
        int cycle = 0;
        for (int i = 0; i < 300; ++i) {
            auto it = seen.find(m.t);
            if (it != seen.end()) {
                cycle = i - it->second;
                break;
            }
            seen[m.t] = i;
            step(m, Rule{});
        }
        CHECK(cycle == 1 || cycle == 2);
        period2 += cycle == 2, ++n;
        Settle st = settle(copy, Rule{}, 300);
        CHECK(st.period == cycle);
    }
    std::printf("  settle: %d/%d small grids ended in a 2-cycle\n", period2, n);

    // 同步更新的不动点对原地更新也是不动点
    for (int s = 0; s < 30; ++s) {
        Map m = randomMap(40, 30, 50 + s, 0.45f);
        Settle r = settle(m, Rule{}, 200);
        if (r.period != 1) continue;
        CHECK(stepInPlace(m, Rule{}) == 0);
    }
    // 原地更新确实和同步更新不同
    Map a = randomMap(40, 30, 9, 0.45f), b = a;
    step(a, Rule{});
    stepInPlace(b, Rule{});
    CHECK(a.t != b.t);
}

static void testRegions() {
    for (int s = 0; s < 80; ++s) {
        int w = 5 + s % 23, h = 5 + (s * 3) % 19;
        Map m = randomMap(w, h, 500 + s, 0.4f + 0.01f * (s % 10));
        if (s % 2) step(m, Rule{});
        Regions r = labelRegions(m);
        UnionFind uf(w * h);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                if (!m.floor(x, y)) continue;
                if (m.floor(x + 1, y)) uf.unite(y * w + x, y * w + x + 1);
                if (m.floor(x, y + 1)) uf.unite(y * w + x, (y + 1) * w + x);
            }
        // 同一块 ⇔ 同一个根；大小一致
        std::map<int, int> rootSize;
        for (int i = 0; i < w * h; ++i) {
            CHECK((r.label[i] < 0) == (m.t[i] == Wall));
            if (m.t[i] != Wall) ++rootSize[uf.find(i)];
        }
        CHECK(int(rootSize.size()) == int(r.size.size()));
        CHECK(int(r.size.size()) == dungeon::components(m));
        for (int i = 0; i < w * h; ++i)
            for (int j = i + 1; j < w * h && m.t[i] != Wall; j += 7)
                if (m.t[j] != Wall) CHECK((r.label[i] == r.label[j]) == (uf.find(i) == uf.find(j)));
        for (int i = 0; i < w * h; ++i)
            if (m.t[i] != Wall) CHECK(r.size[r.label[i]] == rootSize[uf.find(i)]);

        // keepLargest
        Map k = m;
        int removed = keepLargest(k);
        if (!r.size.empty()) {
            CHECK(dungeon::components(k) == 1);
            CHECK(k.floorCount() == r.size[r.largest()]);
            CHECK(removed == m.floorCount() - k.floorCount());
        }

        // removePillars：与并查集求出的墙块比较
        for (int minSize : {1, 2, 4, 9, 1 << 20}) {  // 最后一个比外圈还大：外圈的墙也不能被填
            Map p = m;
            int filled = removePillars(p, minSize);
            UnionFind wu(w * h);
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x) {
                    if (m.at(x, y) != Wall) continue;
                    if (x + 1 < w && m.at(x + 1, y) == Wall) wu.unite(y * w + x, y * w + x + 1);
                    if (y + 1 < h && m.at(x, y + 1) == Wall) wu.unite(y * w + x, (y + 1) * w + x);
                }
            std::map<int, int> size;
            std::map<int, bool> border;
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x)
                    if (m.at(x, y) == Wall) {
                        int rt = wu.find(y * w + x);
                        ++size[rt];
                        if (x == 0 || y == 0 || x == w - 1 || y == h - 1) border[rt] = true;
                    }
            int want = 0;
            for (int i = 0; i < w * h; ++i) {
                if (m.t[i] != Wall) {
                    CHECK(p.t[i] == m.t[i]);
                    continue;
                }
                int rt = wu.find(i);
                bool fill = !border[rt] && size[rt] < minSize;
                CHECK((p.t[i] != Wall) == fill);
                want += fill;
            }
            CHECK(filled == want);
        }
    }
}

static void testConnect() {
    // 两块区域：挖的墙数 = 最小曼哈顿距离 - 1
    int checked = 0;
    for (int s = 0; s < 3000 && checked < 60; ++s) {
        Map m = randomMap(10 + s % 12, 8 + s % 9, 7000 + s, 0.5f);
        step(m, Rule{});
        Regions r = labelRegions(m);
        if (r.size.size() != 2) continue;
        ++checked;
        int best = 1 << 30;
        for (size_t i = 0; i < m.t.size(); ++i)
            for (size_t j = 0; j < m.t.size(); ++j)
                if (r.label[i] == 0 && r.label[j] == 1)
                    best = std::min(best, std::abs(int(i % m.w) - int(j % m.w)) + std::abs(int(i / m.w) - int(j / m.w)));
        Map c = m;
        std::vector<int> tunnel;
        int carved = connectRegions(c, &tunnel);
        CHECK(carved == best - 1);
        CHECK(int(tunnel.size()) == carved);
        CHECK(dungeon::components(c) == 1);
    }
    CHECK(checked >= 30);
    // 多块区域：结果连通，只把墙改成隧道，外圈不动
    for (int s = 0; s < 60; ++s) {
        Map m = randomMap(30 + s, 20 + s / 2, 8000 + s, 0.5f);
        step(m, Rule{});
        step(m, Rule{});
        Map c = m;
        int carved = connectRegions(c);
        int floorBefore = m.floorCount();
        if (floorBefore == 0) continue;
        CHECK(dungeon::components(c) == 1);
        int changed = 0;
        for (size_t i = 0; i < m.t.size(); ++i) {
            if (m.t[i] != c.t[i]) {
                CHECK(m.t[i] == Wall && c.t[i] == Corridor);
                ++changed;
            }
            int x = int(i % m.w), y = int(i / m.w);
            if (x == 0 || y == 0 || x == m.w - 1 || y == m.h - 1) CHECK(c.t[i] == Wall);
        }
        CHECK(changed == carved);
        CHECK(connectRegions(c) == 0);
    }
}

static void testMeasure() {
    for (int s = 0; s < 40; ++s) {
        int w = 4 + s % 21, h = 4 + (s * 5) % 17;
        Map m = randomMap(w, h, 200 + s, 0.25f + 0.02f * (s % 10));
        if (s % 3 == 0)  // 也测外圈有地面的情况
            for (int x = 0; x < w; ++x) m.at(x, 0) = Room;
        std::vector<int> d = openRadius(m);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                if (m.at(x, y) == Wall) {
                    CHECK(d[y * w + x] == 0);
                    continue;
                }
                int best = std::min({x + 1, y + 1, w - x, h - y});  // 地图外
                for (int yy = 0; yy < h; ++yy)
                    for (int xx = 0; xx < w; ++xx)
                        if (m.at(xx, yy) == Wall) best = std::min(best, std::max(std::abs(xx - x), std::abs(yy - y)));
                CHECK(d[y * w + x] == best);
            }
    }
    Map one(3, 3);
    one.at(1, 1) = Room;
    CHECK(edgePerFloor(one) == 4.0);
    Map two(4, 3);
    two.at(1, 1) = two.at(2, 1) = Room;
    CHECK(edgePerFloor(two) == 3.0);
}

int main() {
    testStep();
    testSettle();
    testRegions();
    testConnect();
    testMeasure();
    return report("cave_test");
}
