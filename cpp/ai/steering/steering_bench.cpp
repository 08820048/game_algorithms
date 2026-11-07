// 转向行为与 Boids：邻居查询的开销、三条规则的消融、浮点累加顺序与混沌、抵达的反应时间、避障的组合方式（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/steering/steering_bench.cpp -o steering_bench && ./steering_bench
#include <chrono>
#include <cstdio>
#include <numeric>

#include "boids.h"
#include "steering.h"

using namespace ga;
using namespace ga::boids;
using Clock = std::chrono::steady_clock;

// 对照组：网格只存下标，不搬动鸟。鸟在数组里的顺序与位置无关，查询时到处跳着读内存
struct IndexGrid {
    float cell;
    int cols, rows;
    std::vector<int> start, entries, cellOf;
    void build(const std::vector<Boid>& boids, const Params& p) {
        cell = p.radius;
        cols = int(std::ceil(p.width / cell)), rows = int(std::ceil(p.height / cell));
        start.assign(cols * rows + 1, 0);
        cellOf.resize(boids.size());
        for (size_t i = 0; i < boids.size(); ++i) ++start[(cellOf[i] = index(boids[i].pos)) + 1];
        std::partial_sum(start.begin(), start.end(), start.begin());
        entries.resize(boids.size());
        std::vector<int> fill(start.begin(), start.end() - 1);
        for (size_t i = 0; i < boids.size(); ++i) entries[fill[cellOf[i]]++] = int(i);
    }
    int cx(float x) const { return std::clamp(int(std::floor(x / cell)), 0, cols - 1); }
    int cy(float y) const { return std::clamp(int(std::floor(y / cell)), 0, rows - 1); }
    int index(Vec2 v) const { return cy(v.y) * cols + cx(v.x); }
    void forces(const std::vector<Boid>& boids, const Params& p, std::vector<Vec2>& out) const {
        out.resize(boids.size());
        for (size_t i = 0; i < boids.size(); ++i) {
            const Boid& b = boids[i];
            Vec2 fwd = forward(b);
            int x = cx(b.pos.x), y0 = cy(b.pos.y);
            int x0 = std::max(x - 1, 0), x1 = std::min(x + 1, cols - 1);
            Accum acc;
            for (int y = std::max(y0 - 1, 0); y <= std::min(y0 + 1, rows - 1); ++y)
                for (int k = start[y * cols + x0]; k < start[y * cols + x1 + 1]; ++k)
                    if (entries[k] != int(i)) addNeighbor(acc, b, fwd, boids[entries[k]], p);
            out[i] = steer(b, acc, p);
        }
    }
};

static Params scaledParams(int n) {  // 密度固定：每只鸟平均 2400 平方像素（演示中 150 只鸟在 800×450 的场地里）
    Params p;
    float scale = std::sqrt(n * 2400.0f / (800 * 450));
    p.width = 800 * scale, p.height = 450 * scale;
    return p;
}

template <class F>
static double minMs(int rounds, F&& f) {
    double best = 1e30;
    for (int r = 0; r < rounds; ++r) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
    }
    return best;
}

struct Sample {
    double pol = 0, groups = 0, nn = 0, close = 0;  // close：最近的同伴不到 4 像素的鸟的比例
};

static Sample observe(Params p, int seeds, int n) {
    Sample s;
    int samples = 0;
    for (int seed = 1; seed <= seeds; ++seed) {
        Pcg32 rng(seed);
        auto flock = randomFlock(n, p, rng);
        FlockGrid grid;
        std::vector<Vec2> f;
        for (int i = 1; i <= 60 * 60; ++i) {
            grid.build(flock, p), grid.forces(flock, p, f), integrate(flock, f, p, 1 / 60.0f);
            if (i >= 60 * 30 && i % 60 == 0) {  // 后 30 秒每秒采样一次
                float mean, min;
                nearestStats(flock, &mean, &min);
                int close = 0;
                for (size_t a = 0; a < flock.size(); ++a) {
                    float best = 1e9f;
                    for (size_t b = 0; b < flock.size(); ++b)
                        if (a != b) best = std::min(best, lengthSq(flock[a].pos - flock[b].pos));
                    close += best < 16;
                }
                s.pol += polarization(flock), s.groups += groups(flock, p.radius), s.nn += mean, s.close += double(close) / n;
                ++samples;
            }
        }
    }
    s.pol /= samples, s.groups /= samples, s.nn /= samples, s.close /= samples;
    return s;
}

int main() {
    // ---- 1) 邻居查询：暴力 vs 网格（只存下标）vs 网格（按格子重新排列） ----
    std::printf("1) one flock update (forces + integrate), constant density, flock warmed up 10 s; min of rounds, ms\n");
    std::printf("  %-7s %10s %14s %14s %12s\n", "boids", "brute", "grid, indices", "grid, sorted", "neighbors");
    for (int n : {100, 300, 1000, 3000, 10000, 30000}) {
        Params p = scaledParams(n);
        Pcg32 rng(n);
        auto flock = randomFlock(n, p, rng);
        FlockGrid grid;
        std::vector<Vec2> f;
        for (int i = 0; i < 600; ++i) grid.build(flock, p), grid.forces(flock, p, f), integrate(flock, f, p, 1 / 60.0f);
        // 下标网格的对照：把鸟的顺序打乱，模拟"数组顺序与位置无关"
        auto shuffled = flock;
        Pcg32 srng(1);
        shuffle(shuffled, srng);
        int rounds = n <= 1000 ? 40 : n <= 10000 ? 10 : 4;
        double brute = n <= 10000 ? minMs(rounds, [&] {
            auto b = flock;
            forcesBrute(b, p, f);
            integrate(b, f, p, 1 / 60.0f);
        }) : -1;
        IndexGrid ig;
        double indexed = minMs(rounds, [&] {
            auto b = shuffled;
            ig.build(b, p), ig.forces(b, p, f);
            integrate(b, f, p, 1 / 60.0f);
        });
        double sorted = minMs(rounds, [&] {
            auto b = shuffled;  // 从同样乱序的数组开始：第一次排序要搬动所有鸟
            grid.build(b, p), grid.forces(b, p, f);
            integrate(b, f, p, 1 / 60.0f);
        });
        double sortedWarm = minMs(rounds, [&] {
            auto b = flock;  // 上一帧已经排好序
            grid.build(b, p), grid.forces(b, p, f);
            integrate(b, f, p, 1 / 60.0f);
        });
        long long nb = 0;
        {  // 平均邻居数（感知半径内，不考虑视野）
            IndexGrid g2;
            g2.build(flock, p);
            for (size_t i = 0; i < flock.size(); ++i) {
                int x = g2.cx(flock[i].pos.x), y0 = g2.cy(flock[i].pos.y);
                for (int y = std::max(y0 - 1, 0); y <= std::min(y0 + 1, g2.rows - 1); ++y)
                    for (int k = g2.start[y * g2.cols + std::max(x - 1, 0)]; k < g2.start[y * g2.cols + std::min(x + 1, g2.cols - 1) + 1]; ++k)
                        nb += g2.entries[k] != int(i) && lengthSq(flock[g2.entries[k]].pos - flock[i].pos) < p.radius * p.radius;
            }
        }
        char bs[16];
        std::snprintf(bs, sizeof bs, brute < 0 ? "-" : "%.3f", brute);
        std::printf("  %-7d %10s %14.3f %14.3f %12.1f   (sorted, already in order: %.3f)\n", n, bs, indexed, sorted, double(nb) / n, sortedWarm);
    }

    // ---- 2) 消融：去掉一条规则 ----
    std::printf("\n2) ablation: 150 boids, 10 seeds, sampled every second during 30-60 s\n");
    std::printf("  %-28s %8s %8s %10s %10s\n", "variant", "order", "groups", "nn (px)", "<4px");
    struct V {
        const char* label;
        void (*edit)(Params&);
    };
    V variants[] = {
        {"all three rules", [](Params&) {}},
        {"no separation", [](Params& p) { p.wSeparation = 0; }},
        {"no alignment", [](Params& p) { p.wAlignment = 0; }},
        {"no cohesion", [](Params& p) { p.wCohesion = 0; }},
        {"alignment only", [](Params& p) { p.wSeparation = p.wCohesion = 0; }},
        {"360 deg view", [](Params& p) { p.fovCos = -1; }},
        {"120 deg view", [](Params& p) { p.fovCos = 0.5f; }},
        {"radius 24 (half)", [](Params& p) { p.radius = 24; }},
        {"radius 96 (double)", [](Params& p) { p.radius = 96; }},
    };
    for (const V& v : variants) {
        Params p;
        v.edit(p);
        Sample s = observe(p, 10, 150);
        std::printf("  %-28s %8.2f %8.1f %10.1f %9.1f%%\n", v.label, s.pol, s.groups, s.nn, 100 * s.close);
    }

    // ---- 3) 累加顺序与混沌：两份完全相同的鸟群，只是邻居的累加顺序不同 ----
    std::printf("\n3) divergence: same 150 boids, one run sums neighbours in grid order, the other in id order\n");
    {
        Params p;
        for (int seed = 1; seed <= 5; ++seed) {
            Pcg32 rng(seed);
            auto a = randomFlock(150, p, rng), b = a;
            FlockGrid grid;
            std::vector<Vec2> fa, fb;
            float t1 = -1, t10 = -1, firstDiff = -1;
            for (int i = 1; i <= 60 * 120 && t10 < 0; ++i) {
                grid.build(a, p), grid.forces(a, p, fa), integrate(a, fa, p, 1 / 60.0f);
                forcesBrute(b, p, fb), integrate(b, fb, p, 1 / 60.0f);  // b 始终按 id 排列
                double worst = 0;
                for (const Boid& x : a) worst = std::max(worst, double(distance(x.pos, b[x.id].pos)));
                if (worst > 0 && firstDiff < 0) firstDiff = i / 60.0f;
                if (worst > 1 && t1 < 0) t1 = i / 60.0f;
                if (worst > 10 && t10 < 0) t10 = i / 60.0f;
            }
            std::printf("  seed %d: first difference at %.2f s, some boid 1 px off at %.1f s, 10 px off at %.1f s\n", seed, firstDiff, t1, t10);
        }
    }

    // ---- 4) 抵达：反应时间与减速半径 ----
    std::printf("\n4) arrive from 400 px, starting at full speed: overshoot and time to settle within 1 px\n");
    std::printf("  %-10s %-12s %12s %12s\n", "reaction", "slow radius", "overshoot", "settle (s)");
    for (float reaction : {0.05f, 0.1f, 0.3f, 1.0f})
        for (float slow : {40.0f, 100.0f, 200.0f}) {
            steering::Agent a;
            a.reaction = reaction;
            a.pos = {0, 0}, a.vel = {120, 0};
            Vec2 target{400, 0};
            float past = 0, settle = -1;
            for (int i = 1; i <= 60 * 20; ++i) {
                steering::integrate(a, steering::arrive(a, target, slow), 1 / 60.0f);
                past = std::max(past, a.pos.x - target.x);
                bool in = distance(a.pos, target) < 1 && length(a.vel) < 1;
                if (in && settle < 0) settle = i / 60.0f;
                if (!in) settle = -1;
            }
            std::printf("  %-10.2f %-12.0f %10.1f px %12.2f\n", reaction, slow, past, settle);
        }

    // ---- 5) 避障：组合方式与探测距离 ----
    std::printf("\n5) obstacle field, 2000 runs: hit = touched an obstacle, stuck = not at goal after 20 s\n");
    std::printf("  %-26s %8s %8s %12s\n", "variant", "hit", "stuck", "time (s)");
    for (int variant = 0; variant < 6; ++variant) {
        bool priority = variant >= 2;
        float look = variant == 3 ? 0.1f : variant == 4 ? 0.8f : 0.3f;
        float avoidWeight = variant == 1 ? 3.0f : 1.0f;
        bool noAvoid = variant == 5;
        Pcg32 rng(36);
        int hits = 0, stuck = 0, trials = 2000;
        double time = 0;
        for (int t = 0; t < trials; ++t) {
            std::vector<steering::Circle> obs;
            while (obs.size() < 12) {
                steering::Circle c{{100 + rng.nextFloat() * 400, 40 + rng.nextFloat() * 520}, 15 + rng.nextFloat() * 30};
                bool ok = true;
                for (const auto& o : obs) ok &= distance(o.c, c.c) > o.r + c.r + 30;
                if (ok) obs.push_back(c);
            }
            Vec2 start{20, 40 + rng.nextFloat() * 520}, goal{580, 40 + rng.nextFloat() * 520};
            steering::Agent a;
            a.pos = start;
            bool hit = false;
            int i = 0;
            for (; i < 60 * 20 && distance(a.pos, goal) > 10; ++i) {
                Vec2 avoid = noAvoid ? Vec2{} : steering::avoidObstacles(a, obs, look), go = steering::arrive(a, goal, 60), f{};
                if (!priority) f = steering::weightedSum({{avoid, avoidWeight}, {go, 1}}, a.maxForce);
                else steering::accumulate(f, avoid, a.maxForce), steering::accumulate(f, go, a.maxForce);
                steering::integrate(a, f, 1 / 60.0f);
                for (const auto& o : obs) hit |= distance(a.pos, o.c) < o.r + a.radius * 0.5f;
            }
            hits += hit, stuck += distance(a.pos, goal) > 10;
            time += i / 60.0;
        }
        const char* labels[] = {"weighted sum 1:1", "weighted sum 3:1", "priority, look 0.3 s", "priority, look 0.1 s",
                                "priority, look 0.8 s", "no avoidance"};
        std::printf("  %-26s %7.1f%% %7.1f%% %12.2f\n", labels[variant], 100.0 * hits / trials, 100.0 * stuck / trials, time / trials);
    }
}
