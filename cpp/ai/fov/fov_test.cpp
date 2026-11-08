#include "fov.h"

#include <cstdio>
#include <numeric>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;
using namespace ga::fov;

static Grid randomGrid(Pcg32& rng, int w, int h, double density) {
    Grid g(w, h);
    for (auto& c : g.wall) c = rng.nextDouble() < density;
    return g;
}

// ---- 墙的可见性：暴力枚举视线 ----
// 坐标放大一倍：格子中心在偶数点，角在奇数点。视线 O + t·d，t >= 0
struct Vec {
    int64_t x, y;
};

// 线段 O + t·d（0 <= t <= sN/sD）是否被挡：穿过某堵墙（除了 skip）的内部，或从两墙之间的角点穿过
static bool segmentBlocked(const Grid& g, Vec O, Vec d, int64_t sN, int64_t sD, int skipX, int skipY) {
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x) {
            if (!g.wall[g.index(x, y)] || (x == skipX && y == skipY)) continue;
            int64_t loN = 0, loD = 1, hiN = sN, hiD = sD;
            bool ok = true;
            auto clip = [&](int64_t p0, int64_t dir, int64_t lo, int64_t hi) {
                if (dir == 0) {
                    ok &= lo < p0 && p0 < hi;
                    return;
                }
                int64_t a = lo - p0, b = hi - p0;
                if (dir < 0) a = -a, b = -b, dir = -dir, std::swap(a, b);
                if ((__int128)a * loD > (__int128)loN * dir) loN = a, loD = dir;
                if ((__int128)b * hiD < (__int128)hiN * dir) hiN = b, hiD = dir;
            };
            clip(O.x, d.x, 2 * x - 1, 2 * x + 1);
            clip(O.y, d.y, 2 * y - 1, 2 * y + 1);
            if (ok && (__int128)loN * hiD < (__int128)hiN * loD) return true;
        }
    // 地图外也是墙：视线不会走出地图（起点在地图内，目标墙也在地图内），这里只需检查角点
    if (d.x != 0 && d.y != 0) {
        int64_t k = std::gcd(std::llabs(d.x), std::llabs(d.y)), sx = d.x > 0 ? 1 : -1, sy = d.y > 0 ? 1 : -1;
        Vec step{d.x / k, d.y / k};
        for (int64_t i = 1; (__int128)i * sD < (__int128)sN * k; ++i) {  // t = i/k < s
            int64_t px = O.x + step.x * i, py = O.y + step.y * i;
            if (!(px & 1) || !(py & 1)) continue;
            if (g.blocked(int((px + sx) >> 1), int((py - sy) >> 1)) && g.blocked(int((px - sx) >> 1), int((py + sy) >> 1))) return true;
        }
    }
    return false;
}

// 墙 (wx, wy) 是否可见：存在一条视线到达它的内部
static bool wallVisibleBrute(const Grid& g, int ox, int oy, int wx, int wy) {
    Vec O{2 * ox, 2 * oy};
    // 关键方向：指向所有墙角的方向。相邻关键方向之间，"被挡"的状态不变
    std::vector<Vec> dirs;
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x)
            if (g.wall[g.index(x, y)])
                for (int cy : {-1, 1})
                    for (int cx : {-1, 1}) dirs.push_back({2 * x + cx - O.x, 2 * y + cy - O.y});
    auto half = [](Vec v) { return v.y > 0 || (v.y == 0 && v.x > 0) ? 0 : 1; };
    std::sort(dirs.begin(), dirs.end(), [&](Vec a, Vec b) {
        if (half(a) != half(b)) return half(a) < half(b);
        return a.x * b.y - a.y * b.x > 0;
    });
    std::vector<Vec> cand = dirs;
    for (size_t i = 0; i < dirs.size(); ++i) {
        Vec a = dirs[i], b = dirs[(i + 1) % dirs.size()];
        if (a.x * b.y - a.y * b.x > 0) cand.push_back({a.x + b.x, a.y + b.y});  // 夹在两者之间的方向
    }
    for (Vec d : cand) {
        if (d.x == 0 && d.y == 0) continue;
        // 视线在墙内部的 t 区间 (tin, tout)
        int64_t loN = 0, loD = 1, hiN = 1 << 30, hiD = 1;
        bool ok = true;
        auto clip = [&](int64_t p0, int64_t dir, int64_t lo, int64_t hi) {
            if (dir == 0) {
                ok &= lo < p0 && p0 < hi;
                return;
            }
            int64_t a = lo - p0, b = hi - p0;
            if (dir < 0) a = -a, b = -b, dir = -dir, std::swap(a, b);
            if (a * loD > loN * dir) loN = a, loD = dir;
            if (b * hiD < hiN * dir) hiN = b, hiD = dir;
        };
        clip(O.x, d.x, 2 * wx - 1, 2 * wx + 1);
        clip(O.y, d.y, 2 * wy - 1, 2 * wy + 1);
        if (!ok || loN * hiD >= hiN * loD) continue;
        // 停在墙内部的中点：t = (tin + tout) / 2
        int64_t mN = loN * hiD + hiN * loD, mD = 2 * loD * hiD;
        if (!segmentBlocked(g, O, d, mN, mD, wx, wy)) return true;
    }
    return false;
}

int main() {
    // ---- 阴影投射与暴力定义逐格一致：地面看视线，墙看是否有视线到达内部 ----
    {
        Pcg32 rng(1);
        long long floors = 0, walls = 0, visibleFloors = 0, visibleWalls = 0;
        for (int t = 0; t < 1500; ++t) {
            int w = 3 + rng.below(14), h = 3 + rng.below(14);
            Grid g = randomGrid(rng, w, h, rng.nextDouble() * 0.45);
            int ox = rng.below(w), oy = rng.below(h);
            g.wall[g.index(ox, oy)] = 0;
            int radius = 1 + rng.below(18);
            std::vector<uint8_t> sc, br;
            shadowcast(g, ox, oy, radius, sc);
            fovBrute(g, ox, oy, radius, br);
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x) {
                    int i = g.index(x, y);
                    int dx = x - ox, dy = y - oy;
                    if (!g.wall[i]) {
                        CHECK(sc[i] == br[i]);
                        ++floors, visibleFloors += br[i];
                    } else {
                        bool expect = dx * dx + dy * dy <= radius * radius && wallVisibleBrute(g, ox, oy, x, y);
                        CHECK(bool(sc[i]) == expect);
                        ++walls, visibleWalls += expect;
                    }
                }
        }
        std::printf("shadowcast == brute force on %lld floor cells (%lld visible) and %lld wall cells (%lld visible)\n", floors,
                    visibleFloors, walls, visibleWalls);
    }

    // ---- 对称：地面格子 A 看得见 B，当且仅当 B 看得见 A ----
    {
        Pcg32 rng(2);
        long long pairs = 0, asymShadow = 0, asymClassic = 0, asymRay = 0;
        for (int t = 0; t < 60; ++t) {
            Grid g = randomGrid(rng, 20, 20, 0.25);
            int n = g.w * g.h;
            std::vector<std::vector<uint8_t>> sc(n), cl(n), ry(n);
            for (int i = 0; i < n; ++i)
                if (!g.wall[i]) {
                    shadowcast(g, i % g.w, i / g.w, 12, sc[i]);
                    classicShadowcast(g, i % g.w, i / g.w, 12, cl[i]);
                    rayCast(g, i % g.w, i / g.w, 12, ry[i]);
                }
            for (int a = 0; a < n; ++a)
                for (int b = a + 1; b < n; ++b) {
                    if (g.wall[a] || g.wall[b]) continue;
                    ++pairs;
                    asymShadow += sc[a][b] != sc[b][a];
                    asymClassic += cl[a][b] != cl[b][a];
                    asymRay += ry[a][b] != ry[b][a];
                }
        }
        CHECK(asymShadow == 0);
        std::printf("symmetry over %lld floor pairs: shadowcast %lld asymmetric, classic %lld, ray casting %lld\n", pairs, asymShadow,
                    asymClassic, asymRay);
    }

    // ---- 几个手工例子 ----
    {
        // 斜对角相接的两堵墙之间不能看穿；只有一堵时可以擦着角看过去
        Grid g(5, 5);
        g.wall[g.index(2, 1)] = g.wall[g.index(1, 2)] = 1;
        CHECK(!canSee(g, 1, 1, 2, 2));          // 从两墙之间的角点穿过
        g.wall[g.index(1, 2)] = 0;
        CHECK(canSee(g, 1, 1, 2, 2));  // 只剩一堵：擦过角点
        CHECK(canSee(g, 1, 1, 4, 4));
        // 一堵墙后面的格子看不见，旁边的看得见
        Grid h(7, 3);
        h.wall[h.index(3, 1)] = 1;
        CHECK(!canSee(h, 0, 1, 6, 1));
        CHECK(canSee(h, 0, 0, 6, 0));
        std::vector<uint8_t> out;
        shadowcast(h, 0, 1, 10, out);
        CHECK(out[h.index(3, 1)] && !out[h.index(6, 1)]);
        // 半径：中心距离不超过半径
        Grid open(21, 21);
        shadowcast(open, 10, 10, 5, out);
        int count = 0;
        for (uint8_t v : out) count += v;
        int expect = 0;
        for (int y = -5; y <= 5; ++y)
            for (int x = -5; x <= 5; ++x) expect += x * x + y * y <= 25;
        CHECK(count == expect);
    }

    // ---- 感知：视野锥 ----
    {
        Grid g(10, 10);
        Viewer v{2.5f, 2.5f, 0.0f, 0.5f, 6};  // 朝 +x，半角约 28.6°
        CHECK(sees(g, v, 6, 2));
        CHECK(!sees(g, v, 2, 6));   // 正下方：不在锥内
        CHECK(!sees(g, v, 9, 2));   // 太远
        CHECK(sees(g, v, 2, 2));    // 自己所在的格子
        CHECK(sees(g, v, 7, 4));    // 偏离朝向 21.8°，在半角 28.6° 之内
        CHECK(!sees(g, v, 6, 5));   // 偏离 36.9°，在锥外（如果误用整个角度 57° 就会算进来）
        g.wall[g.index(4, 2)] = 1;
        CHECK(!sees(g, v, 6, 2));   // 被墙挡住
        float a = 0;
        for (int i = 0; i < 3; ++i) a = updateAwareness(a, true, 1, 6, 0.1f);
        float b = 0;
        for (int i = 0; i < 3; ++i) b = updateAwareness(b, true, 5, 6, 0.1f);
        CHECK(a > b && b > 0);  // 近处更快被发现
        for (int i = 0; i < 100; ++i) a = updateAwareness(a, false, 1, 6, 0.1f);
        CHECK(a == 0);
    }

    return report("fov_test");
}
