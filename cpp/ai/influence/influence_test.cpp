#include "influence.h"

#include <cstdio>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;
using namespace ga::influence;

static Map randomMap(Pcg32& rng, int w, int h, float wallChance) {
    Map m(w, h);
    for (auto& c : m.wall) c = rng.nextFloat() < wallChance;
    // 再加几道长墙，让绕路成为常态
    for (int k = 0; k < 3; ++k) {
        int x = rng.below(w), y = rng.below(h), len = 3 + rng.below(std::max(w, h));
        bool horizontal = rng.below(2);
        for (int t = 0; t < len; ++t) {
            int cx = horizontal ? x + t : x, cy = horizontal ? y : y + t;
            if (cx < w && cy < h) m.wall[m.index(cx, cy)] = 1;
        }
    }
    return m;
}

static std::vector<Source> randomSources(Pcg32& rng, const Map& m, int n) {
    std::vector<Source> s;
    for (int i = 0; i < n; ++i) s.push_back({int(rng.below(m.w)), int(rng.below(m.h)), 0.2f + rng.nextFloat() * 2});
    return s;
}

int main() {
    // ---- 衰减与盖章：与暴力逐格累加逐位相同 ----
    CHECK(falloff(0, 0, 5) == 1 && falloff(5, 0, 5) == 0 && falloff(3, 4, 10) == 0.5f);
    {
        Pcg32 rng(41);
        long long cells = 0;
        for (int t = 0; t < 300; ++t) {
            Map m(1 + rng.below(60), 1 + rng.below(60));
            float radius = 0.5f + rng.nextFloat() * 20;
            auto sources = randomSources(rng, m, rng.below(30));
            for (Source& s : sources)  // 有的单位在地图外：印章只盖住地图内的部分
                if (rng.below(8) == 0) s.x += (rng.below(2) ? 1 : -1) * int(rng.below(10) + m.w);
            Field a, b;
            stampBrute(m, sources, radius, a);
            Stamper(radius).stamp(m, sources, b);
            CHECK(a == b);
            cells += a.size();
        }
        std::printf("stamp: identical to brute force on %lld cells\n", cells);
    }

    // ---- 传播：收敛后与逐个单位的 Dijkstra 逐位相同；Jacobi 与正反扫描结果相同 ----
    {
        Pcg32 rng(42);
        long long cells = 0, jacobiRounds = 0, sweepPasses = 0;
        int maps = 0;
        for (int t = 0; t < 300; ++t) {
            Map m = randomMap(rng, 1 + rng.below(50), 1 + rng.below(50), rng.nextFloat() * 0.35f);
            Decay d(0.5f + rng.nextFloat() * 0.45f, rng.below(4) == 0 ? 0 : rng.nextFloat() * 0.1f);
            auto sources = randomSources(rng, m, rng.below(12));
            Field ref, jac, sw;
            geodesicBrute(m, sources, d, ref);
            jacobiRounds += propagate(m, sources, d, jac);
            sweepPasses += propagateSweeps(m, sources, d, sw);
            CHECK(jac == ref);
            CHECK(sw == ref);
            for (size_t i = 0; i < ref.size(); ++i) {
                if (m.wall[i]) CHECK(ref[i] == 0);  // 墙里没有影响力
                CHECK(ref[i] >= 0);
            }
            cells += ref.size(), ++maps;
        }
        std::printf("propagation: identical to per-source Dijkstra on %lld cells; %.1f Jacobi rounds vs %.1f sweeps on average\n", cells,
                    double(jacobiRounds) / maps, double(sweepPasses) / maps);
    }

    // ---- 墙会挡住影响力：墙后的格子只能绕路到达 ----
    {
        Map m(11, 5);
        for (int y = 0; y < 4; ++y) m.wall[m.index(5, y)] = 1;  // 一道墙，最下面一行留了口子
        Decay d(0.8f, 0);
        Field f;
        propagate(m, {{2, 0, 1}}, d, f);
        // 墙对面同一行的格子 (8, 0)：直线距离 6，绕路要先下到最底行再上来
        float straight = std::pow(0.8f, 6.0f);
        CHECK(f[m.index(8, 0)] < straight * 0.5f);
        Field s;
        stampBrute(m, {{2, 0, 1}}, 10, s);
        CHECK(s[m.index(8, 0)] > 0.3f);  // 盖章不管墙
        // 没有墙时，直线上的格子正好是 0.8 的整数次幂
        Map open(9, 1);
        Field g;
        propagate(open, {{0, 0, 1}}, d, g);
        float v = 1;
        for (int x = 0; x < 9; ++x, v *= 0.8f) CHECK(g[x] == v);
        // 斜走一格衰减 0.8^√2；不能切过墙角：(0,0) 到 (1,1) 时如果 (1,0) 是墙，只能走两步直线
        Map sq(3, 3);
        propagate(sq, {{0, 0, 1}}, d, g);
        CHECK(g[sq.index(1, 1)] == d.diagonal && std::abs(d.diagonal - std::pow(0.8f, std::sqrt(2.0f))) < 1e-6f);
        sq.wall[sq.index(1, 0)] = 1;
        propagate(sq, {{0, 0, 1}}, d, g);
        CHECK(g[sq.index(1, 1)] == 0.8f * 0.8f);
        geodesicBrute(sq, {{0, 0, 1}}, d, g);
        CHECK(g[sq.index(1, 1)] == 0.8f * 0.8f);
    }

    // ---- 动量：momentum < 1 时逐渐逼近收敛值 ----
    {
        Map m(20, 20);
        Decay d(0.85f);
        std::vector<Source> s = {{10, 10, 1}};
        Field target, seeds, cur, scratch;
        propagate(m, s, d, target);
        seed(m, s, seeds);
        cur.assign(seeds.size(), 0);
        for (int k = 0; k < 400; ++k) propagateStep(m, seeds, d, 0.3f, cur, scratch);
        float err = 0;
        for (size_t i = 0; i < cur.size(); ++i) err = std::max(err, std::abs(cur[i] - target[i]));
        CHECK(err < 1e-4f);
    }

    // ---- 组合 ----
    CHECK(combine(3, 1, View::Influence) == 2 && combine(3, 1, View::Tension) == 4 && combine(3, 1, View::Vulnerability) == 2);
    CHECK(combine(2, 2, View::Vulnerability) == 4 && combine(5, 0, View::Vulnerability) == 0);

    // ---- 避开火力的路径：A* 的代价等于 Dijkstra（启发函数为 0）；danger 越大，暴露越少、路越长 ----
    {
        Pcg32 rng(43);
        int trials = 0;
        double len0 = 0, lenK = 0, exp0 = 0, expK = 0;
        for (int t = 0; t < 200; ++t) {
            Map m = randomMap(rng, 40, 30, 0.15f);
            auto enemies = randomSources(rng, m, 4);
            Field threat;
            propagate(m, enemies, Decay(0.85f), threat);
            int start = m.index(0, rng.below(30)), goal = m.index(39, rng.below(30));
            m.wall[start] = m.wall[goal] = 0;
            PathResult p0 = safePath(m, threat, start, goal, 0), pk = safePath(m, threat, start, goal, 10);
            if (p0.cells.empty()) {
                CHECK(pk.cells.empty());
                continue;
            }
            // 最优性：A* 的代价与不用启发函数的 Dijkstra 相同
            PathResult dk = safePath(m, threat, start, goal, 10, 0);
            CHECK(std::abs(dk.cost - pk.cost) <= 1e-4f * dk.cost);
            auto measure = [&](const PathResult& p, double* len, double* exposure) {
                for (size_t k = 1; k < p.cells.size(); ++k) {
                    int a = p.cells[k - 1], b = p.cells[k];
                    bool diag = a % m.w != b % m.w && a / m.w != b / m.w;
                    *len += diag ? std::sqrt(2.0) : 1.0;
                    *exposure += (diag ? std::sqrt(2.0) : 1.0) * threat[b];
                    CHECK(!m.wall[b]);
                }
            };
            double l0 = 0, e0 = 0, lk = 0, ek = 0;
            measure(p0, &l0, &e0), measure(pk, &lk, &ek);
            CHECK(std::abs(p0.cost - l0) < 1e-3);         // danger = 0：代价就是长度
            CHECK(lk >= l0 - 1e-3 && ek <= e0 + 1e-3);     // 路更长，但暴露更少
            CHECK(pk.cost <= l0 + 10 * e0 + 1e-2);        // 最优性：不比"沿最短路走"更差
            len0 += l0, lenK += lk, exp0 += e0, expK += ek, ++trials;
        }
        std::printf("safe path, danger 10 (%d maps): length %.1f -> %.1f, exposure %.2f -> %.2f\n", trials, len0 / trials, lenK / trials,
                    exp0 / trials, expK / trials);
    }
    return report("influence_test");
}
