#include "poisson.h"

#include <cstdio>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::poisson;

// ---- 参照写法：逐对比较 ----
static float minPairDistance(const std::vector<Vec2>& p) {
    float best = INFINITY;
    for (size_t i = 0; i < p.size(); ++i)
        for (size_t j = i + 1; j < p.size(); ++j) best = std::min(best, std::hypot(p[i].x - p[j].x, p[i].y - p[j].y));
    return best;
}
static bool fitsBrute(Vec2 q, const std::vector<Vec2>& p, float r) {
    for (const Vec2& a : p)
        if ((a.x - q.x) * (a.x - q.x) + (a.y - q.y) * (a.y - q.y) < r * r) return false;
    return true;
}
static bool inside(const std::vector<Vec2>& p, float w, float h) {
    for (const Vec2& a : p)
        if (!(a.x >= 0 && a.y >= 0 && a.x < w && a.y < h)) return false;
    return true;
}

static void testGrid() {
    // 网格查询与逐个比较一致：随机点集、随机查询，包括贴着边界和恰好距离 r 附近的查询
    for (int s = 0; s < 20; ++s) {
        Pcg32 rng{uint64_t(10 + s)};
        float w = 5 + s, h = 3 + s * 0.7f, r = 0.4f + 0.05f * (s % 6);
        std::vector<Vec2> pts = dartThrowing(w, h, r, rng, 200);
        Grid g(w, h, r);
        for (int i = 0; i < int(pts.size()); ++i) g.insert(pts[i], i);
        int disagree = 0, blocked = 0;
        for (int q = 0; q < 4000; ++q) {
            Vec2 p{rng.nextFloat() * w, rng.nextFloat() * h};
            if (q % 4 == 0) {  // 恰好在某个点的 r 附近
                const Vec2& a = pts[rng.below(uint32_t(pts.size()))];
                float ang = rng.nextFloat() * 6.2831853f, d = r * (0.999f + 0.002f * rng.nextFloat());
                p = {std::clamp(a.x + d * std::cos(ang), 0.0f, w * 0.9999f), std::clamp(a.y + d * std::sin(ang), 0.0f, h * 0.9999f)};
            }
            bool brute = fitsBrute(p, pts, r);
            disagree += g.fits(p, pts) != brute, blocked += !brute;
        }
        CHECK(disagree == 0);
        CHECK(blocked > 500);
        // 恰好相距 r 是允许的（要求的是距离 ≥ r）
        std::vector<Vec2> two{{1, 1}};
        Grid g2(5, 5, 1);
        g2.insert(two[0], 0);
        CHECK(g2.fits({2, 1}, two));
        CHECK(g2.fits({1, 2}, two));
        CHECK(!g2.fits({1.9999f, 1}, two));
        // 每个格子最多一个点
        std::vector<int> cnt(size_t(g.nx()) * g.ny(), 0);
        for (const Vec2& a : pts) CHECK(++cnt[size_t(g.cellY(a.y)) * g.nx() + g.cellX(a.x)] == 1);
    }
}

static void testDart() {
    // 停止条件：maxFailures = 0 一个点也不放；= 1 时第一次失败就停
    {
        Pcg32 rng{1};
        long long t = 0;
        CHECK(dartThrowing(5, 5, 1, rng, 0, true, &t).empty());
        CHECK(t == 0);
        Pcg32 r1{1};
        std::vector<Vec2> p = dartThrowing(5, 5, 1, r1, 1, true, &t);
        CHECK(t == long(p.size()) + 1);
    }
    // 失败计数在成功后清零：40×40 上连续 300 次失败才停，还能放点的面积约 1%（不清零会停在约 34%）
    {
        Pcg32 rng{2};
        std::vector<Vec2> p = dartThrowing(40, 40, 1, rng, 300);
        CHECK(uncoveredFraction(p, 40, 40, 1, 400) < 0.05);
    }
    for (int s = 0; s < 10; ++s) {
        float w = 8 + s, h = 6, r = 0.5f + 0.1f * s;
        Pcg32 a{uint64_t(s)}, b{uint64_t(s)};
        long long ta = 0, tb = 0;
        std::vector<Vec2> pg = dartThrowing(w, h, r, a, 300, true, &ta);
        std::vector<Vec2> pn = dartThrowing(w, h, r, b, 300, false, &tb);
        CHECK(pg == pn);  // 网格加速不改变结果
        CHECK(ta == tb);
        CHECK(minPairDistance(pg) >= r);
        CHECK(inside(pg, w, h));
        CHECK(ta >= long(pg.size()) + 300);
    }
}

static void testBridson() {
    const Annulus ann[] = {Annulus::Area, Annulus::Radius, Annulus::Circle};
    const Pick picks[] = {Pick::Random, Pick::Oldest, Pick::Newest};
    for (int s = 0; s < 12; ++s)
        for (Annulus an : ann)
            for (Pick pk : picks) {
                float w = 6 + s, h = 4 + s * 0.5f, r = 0.35f + 0.03f * s;
                BridsonParams prm;
                prm.annulus = an, prm.pick = pk, prm.k = 1 + s * 3;
                Pcg32 rng{uint64_t(100 + s)};
                long long tries = 0;
                std::vector<Trial> log;
                std::vector<Vec2> p = bridson(w, h, r, rng, prm, &tries, &log);
                CHECK(minPairDistance(p) >= r);
                CHECK(inside(p, w, h));
                CHECK(int(log.size()) == tries);
                // 每个点（除第一个）都是一次被接受的尝试；每个点最终都在一轮 k 次全失败后被移出活跃表
                int acc = 0;
                for (const Trial& t : log) acc += t.accepted;
                CHECK(acc == int(p.size()) - 1);
                CHECK(tries >= acc + (long long)p.size() * prm.k);
                if (prm.k == 1) CHECK(tries == acc + (long long)p.size());
                // 候选到活跃点的距离在 [r, 2r] 内
                bool ringOk = true;
                for (const Trial& t : log) {
                    float d = std::hypot(t.p.x - p[t.from].x, t.p.y - p[t.from].y);
                    ringOk &= d >= r * 0.9999f && d <= 2 * r * 1.0001f;
                    if (an == Annulus::Circle) ringOk &= d <= r * 1.001f;
                }
                CHECK(ringOk);
                // 每一轮 k 次全失败后，那个点被移出活跃表，之后再也不会被选中；被选中的点一定已经放下
                std::vector<char> removed(p.size(), 0);
                bool reuseOk = true, circleOk = true;
                int placedSoFar = 1;
                for (size_t t = 0; t < log.size();) {
                    int from = log[t].from, n = 0;
                    reuseOk &= !removed[from] && from < placedSoFar;
                    double sx = 0, sy = 0;
                    bool acc2 = false;
                    while (t < log.size() && log[t].from == from && n < prm.k) {
                        float d = std::hypot(log[t].p.x - p[from].x, log[t].p.y - p[from].y);
                        sx += (log[t].p.x - p[from].x) / d, sy += (log[t].p.y - p[from].y) / d;
                        ++n;
                        if (log[t++].accepted) {
                            acc2 = true;
                            break;
                        }
                    }
                    if (acc2) ++placedSoFar;
                    else {
                        removed[from] = 1;
                        // circle：k 个候选等间隔分布在整个圆上，方向之和为 0
                        if (an == Annulus::Circle && prm.k >= 2) circleOk &= std::hypot(sx, sy) < 1e-3 * prm.k;
                    }
                }
                CHECK(reuseOk);
                CHECK(circleOk);
                for (char c : removed) CHECK(c);
                // 同一个种子结果相同
                Pcg32 again{uint64_t(100 + s)};
                CHECK(bridson(w, h, r, again, prm) == p);
            }
    // k 很大时接近极大：几乎没有还能放点的地方
    Pcg32 rng{7};
    BridsonParams big;
    big.k = 500;
    std::vector<Vec2> p = bridson(20, 20, 1, rng, big);
    CHECK(uncoveredFraction(p, 20, 20, 1, 400) < 0.002);
    // 环带取样：Area 的半径平方在 [r², 4r²] 上均匀，Radius 的半径在 [r, 2r] 上均匀
    for (Annulus an : {Annulus::Area, Annulus::Radius}) {
        BridsonParams prm;
        prm.annulus = an, prm.k = 30;
        std::vector<Trial> log;
        Pcg32 r2{3};
        std::vector<Vec2> q = bridson(30, 30, 1, r2, prm, nullptr, &log);
        double inner = 0;  // 半径 < 1.5 的比例：Area 应为 (2.25-1)/3 = 0.417，Radius 应为 0.5
        for (const Trial& t : log) inner += std::hypot(t.p.x - q[t.from].x, t.p.y - q[t.from].y) < 1.5f;
        inner /= log.size();
        CHECK(std::abs(inner - (an == Annulus::Area ? 1.25 / 3 : 0.5)) < 0.01);
    }
}

static void testMeasure() {
    for (int s = 0; s < 10; ++s) {
        Pcg32 rng{uint64_t(500 + s)};
        float w = 10 + s, h = 7;
        std::vector<Vec2> p = s % 2 ? whiteNoise(w, h, 50 + 20 * s, rng) : jitteredGrid(w, h, 0.7f, rng);
        std::vector<float> nn = nearestNeighbor(p, w, h, 0.3f + 0.2f * s);
        for (size_t i = 0; i < p.size(); ++i) {
            float best = INFINITY;
            for (size_t j = 0; j < p.size(); ++j)
                if (j != i) best = std::min(best, std::hypot(p[i].x - p[j].x, p[i].y - p[j].y));
            CHECK(std::abs(nn[i] - best) < 1e-5f);
        }
    }
    // 还能放点的比例：与逐个比较一致
    for (int s = 0; s < 6; ++s) {
        Pcg32 rng{uint64_t(900 + s)};
        float w = 9, h = 7, r = 0.8f;
        std::vector<Vec2> p = dartThrowing(w, h, r, rng, 20 + 40 * s);
        int res = 90, open = 0;
        for (int y = 0; y < res; ++y)
            for (int x = 0; x < res; ++x) open += fitsBrute({(x + 0.5f) * w / res, (y + 0.5f) * h / res}, p, r);
        CHECK(std::abs(uncoveredFraction(p, w, h, r, res) - double(open) / (res * res)) < 1e-12);
    }
    // 周期图：一个点处处为 1；两个点 P = |1 + e^{iφ}|² / 2 = 1 + cos(2π f·d)；P(0) = n
    {
        std::vector<Vec2> one{{0.3f, 0.7f}};
        for (float v : periodogram(one, 1, 1, 3)) CHECK(std::abs(v - 1) < 1e-5f);
        std::vector<Vec2> two{{0.1f, 0.2f}, {0.35f, 0.6f}};
        int m = 4, side = 9;
        std::vector<float> P = periodogram(two, 2, 1, m);
        for (int ky = -m; ky <= m; ++ky)
            for (int kx = -m; kx <= m; ++kx) {
                double ph = 2 * M_PI * (kx * 0.25 / 2 + ky * 0.4 / 1);
                CHECK(std::abs(P[size_t(ky + m) * side + kx + m] - (1 + std::cos(ph))) < 1e-4);
            }
        Pcg32 rng{1};
        std::vector<Vec2> many = whiteNoise(5, 5, 300, rng);
        std::vector<float> Q = periodogram(many, 5, 5, 6);
        CHECK(std::abs(Q[size_t(6) * 13 + 6] - 300) < 1e-2);
        // 白噪声的径向平均接近 1
        std::vector<double> rad = radialAverage(Q, 6);
        double mean = 0;
        for (int b = 1; b <= 6; ++b) mean += rad[b] / 6;
        CHECK(std::abs(mean - 1) < 0.25);
    }
    // 抖动网格：每个格子恰好一个点，按行排列
    {
        Pcg32 rng{4};
        float w = 10.5f, h = 6.2f, sz = 0.7f;
        std::vector<Vec2> p = jitteredGrid(w, h, sz, rng);
        int nx = int(w / sz), ny = int(h / sz);
        CHECK(int(p.size()) == nx * ny);
        for (int j = 0; j < int(p.size()); ++j)
            CHECK(int(p[j].x / sz) == j % nx && int(p[j].y / sz) == j / nx);
    }
    // 径向平均不含频率 0（那里 P = n，会把第 0 组拉得很大）
    {
        Pcg32 rng{5};
        std::vector<Vec2> p = whiteNoise(4, 4, 100, rng);
        std::vector<double> rad = radialAverage(periodogram(p, 4, 4, 3), 3);
        CHECK(rad[0] == 0);
    }
    CHECK(std::abs(packingDensity(100, 10, 10, 1) - M_PI / 4) < 1e-6);
}

int main() {
    testGrid();
    testDart();
    testBridson();
    testMeasure();
    return report("poisson_test");
}
