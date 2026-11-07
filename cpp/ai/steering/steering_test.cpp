#include "steering.h"

#include <cstdio>

#include "boids.h"
#include "check.h"

using namespace ga;
using namespace ga::steering;

// 用 dt 模拟 seconds 秒，每帧调用 behavior 求力
template <class F>
static void run(Agent& a, float seconds, float dt, F&& behavior) {
    for (int i = 0, n = int(seconds / dt + 0.5f); i < n; ++i) integrate(a, behavior(a), dt);
}

int main() {
    // ---- 基础 ----
    CHECK(truncate({30, 40}, 10) == Vec2(6, 8) && truncate({3, 4}, 10) == Vec2(3, 4));
    {
        Agent a;
        a.pos = {100, 100};
        Vec2 s = seek(a, {200, 100}), f = flee(a, {200, 100});
        CHECK(s.x > 0 && s.y == 0 && f.x < 0 && f.y == 0);
        CHECK(flee(a, {200, 100}, 50) == Vec2());  // 威胁在恐慌半径以外
        // 速度与力都不会超过上限
        a.vel = {0, 120};
        for (int i = 0; i < 300; ++i) {
            Vec2 before = a.vel;
            integrate(a, seek(a, {-500, 300}) * 10, 1 / 60.0f);
            CHECK(length(a.vel) <= a.maxSpeed * 1.0001f && length(a.vel - before) <= a.maxForce / 60 * 1.0001f);
        }
    }

    // ---- 寻找会绕着目标打转，抵达会停下来 ----
    {
        Agent a;
        a.pos = {0, 0}, a.vel = {0, 60};
        Vec2 target{400, 0};
        run(a, 4, 1 / 60.0f, [&](const Agent& s) { return seek(s, target); });
        int flips = 0;  // 之后 6 秒里速度掉头了几次
        float far = 0;
        for (int i = 0; i < 360; ++i) {
            Vec2 before = a.vel;
            integrate(a, seek(a, target), 1 / 60.0f);
            flips += dot(before, a.vel) < 0, far = std::max(far, distance(a.pos, target));
        }
        std::printf("seek, 4-10 s: reversed %d times, up to %.1f px from target\n", flips, far);
        CHECK(flips >= 4 && far > 5);  // 冲过头、掉头、再冲过头，停不下来

        Agent b;
        b.pos = {0, 0}, b.vel = {0, 60};
        float maxPast = 0;  // 冲过目标的最大距离（沿 x 方向）
        for (int i = 0; i < 600; ++i) {
            integrate(b, arrive(b, target, 100), 1 / 60.0f);
            maxPast = std::max(maxPast, b.pos.x - target.x);
        }
        std::printf("arrive after 10 s: %.3f px from target, speed %.3f, overshoot %.2f px\n", distance(b.pos, target), length(b.vel), maxPast);
        CHECK(distance(b.pos, target) < 0.5f && length(b.vel) < 0.5f && maxPast < 0.5f);
    }

    // ---- 追逐比寻找更快追上直线运动的目标 ----
    {
        Pcg32 rng(35);
        int pursueWins = 0, trials = 1000, pursueMisses = 0;
        double seekTime = 0, pursueTime = 0;
        for (int t = 0; t < trials; ++t) {
            Vec2 start{rng.nextFloat() * 600, rng.nextFloat() * 600};
            Vec2 tvel = fromAngle(rng.nextFloat() * 2 * kPi) * (40 + rng.nextFloat() * 50);
            Vec2 tpos{300, 300};
            auto catchTime = [&](bool usePursue) {
                Agent a;
                a.pos = start;
                Vec2 p = tpos;
                for (int i = 0; i < 60 * 30; ++i) {
                    if (distance(a.pos, p) < 10) return i / 60.0f;
                    integrate(a, usePursue ? pursue(a, p, tvel) : seek(a, p), 1 / 60.0f);
                    p += tvel / 60.0f;
                }
                return 30.0f;
            };
            float s = catchTime(false), p = catchTime(true);
            pursueMisses += p >= 30;
            seekTime += s, pursueTime += p;
            pursueWins += p <= s;
        }
        std::printf("intercept: seek %.2f s, pursue %.2f s on average; pursue no slower in %d / %d\n", seekTime / trials,
                    pursueTime / trials, pursueWins, trials);
        CHECK(pursueMisses == 0 && pursueTime < seekTime && pursueWins > trials * 95 / 100);
    }

    // ---- 逃避：与追兵的距离不断拉大（逃跑者更快） ----
    {
        Agent prey, hunter;
        prey.pos = {300, 300}, prey.maxSpeed = 130;
        hunter.pos = {200, 300}, hunter.maxSpeed = 110;
        float prev = distance(prey.pos, hunter.pos), minDist = prev;
        for (int i = 0; i < 600; ++i) {
            Vec2 fp = evade(prey, hunter.pos, hunter.vel), fh = pursue(hunter, prey.pos, prey.vel);
            integrate(prey, fp, 1 / 60.0f), integrate(hunter, fh, 1 / 60.0f);
            minDist = std::min(minDist, distance(prey.pos, hunter.pos));
        }
        CHECK(distance(prey.pos, hunter.pos) > prev && minDist > 80);
    }

    // ---- 漫游：速度连续，不会停下 ----
    {
        Agent a;
        a.vel = {80, 0};
        Wander w;
        Pcg32 rng(3);
        for (int i = 0; i < 6000; ++i) {
            Vec2 before = a.vel;
            integrate(a, wander(a, w, 1 / 60.0f, rng), 1 / 60.0f);
            CHECK(length(a.vel - before) <= a.maxForce / 60 * 1.0001f);
        }
        CHECK(length(a.vel) > 60);
    }

    // ---- 避障：穿过随机障碍场。按优先级组合时从不撞上障碍 ----
    {
        Pcg32 rng(36);
        int hits[2] = {}, arrived[2] = {}, trials = 500;
        for (int t = 0; t < trials; ++t) {
            std::vector<Circle> obs;
            while (obs.size() < 12) {
                Circle c{{100 + rng.nextFloat() * 400, 40 + rng.nextFloat() * 520}, 15 + rng.nextFloat() * 30};
                bool ok = true;
                for (const Circle& o : obs) ok &= distance(o.c, c.c) > o.r + c.r + 30;  // 留出能通过的缝
                if (ok) obs.push_back(c);
            }
            Vec2 start{20, 40 + rng.nextFloat() * 520}, goal{580, 40 + rng.nextFloat() * 520};
            for (int mode = 0; mode < 2; ++mode) {
                Agent a;
                a.pos = start;
                bool hit = false;
                for (int i = 0; i < 60 * 20 && distance(a.pos, goal) > 10; ++i) {
                    Vec2 avoid = avoidObstacles(a, obs), go = arrive(a, goal, 60), f{};
                    if (mode == 0) f = weightedSum({{avoid, 1}, {go, 1}}, a.maxForce);
                    else accumulate(f, avoid, a.maxForce), accumulate(f, go, a.maxForce);
                    integrate(a, f, 1 / 60.0f);
                    for (const Circle& o : obs) hit |= distance(a.pos, o.c) < o.r + a.radius * 0.5f;
                }
                hits[mode] += hit;
                arrived[mode] += distance(a.pos, goal) <= 10;
            }
        }
        std::printf("obstacle field (%d runs): weighted sum hit %d, arrived %d; priority hit %d, arrived %d\n", trials, hits[0],
                    arrived[0], hits[1], arrived[1]);
        CHECK(hits[1] <= trials / 100 && hits[0] > trials / 4 && arrived[1] >= trials * 95 / 100);  // 加权相加经常撞上
    }

    // ---- Boids：网格版与暴力版的力逐位相同 ----
    {
        using namespace ga::boids;
        Pcg32 rng(37);
        boids::FlockGrid grid;
        long long checked = 0, neighbors = 0;
        for (int t = 0; t < 300; ++t) {
            Params p;
            p.radius = 10 + rng.nextFloat() * 90;
            p.separation = p.radius * rng.nextFloat();
            p.fovCos = rng.nextFloat() * 2 - 1;
            p.width = 100 + rng.nextFloat() * 700, p.height = 100 + rng.nextFloat() * 500;
            int n = rng.below(400);
            auto flock = randomFlock(n, p, rng);
            for (Boid& b : flock) {
                if (rng.below(10) == 0) b.pos = {rng.nextFloat() * p.width * 1.6f - p.width * 0.3f, rng.nextFloat() * p.height * 1.6f - p.height * 0.3f};  // 场地外
                if (rng.below(20) == 0 && n > 1) b.pos = flock[rng.below(n)].pos;  // 重合的两只鸟
                if (rng.below(30) == 0) b.vel = {};                                 // 速度为 0
            }
            grid.build(flock, p);
            // 重新排列后：仍是同一批鸟，并且按格子排好
            std::vector<int> seen(n, 0);
            for (int i = 0; i < n; ++i) {
                ++seen[flock[i].id];
                if (i) CHECK(grid.cellIndex(flock[i - 1].pos) <= grid.cellIndex(flock[i].pos));
            }
            for (int s : seen) CHECK(s == 1);
            std::vector<Vec2> fg, fb;
            grid.forces(flock, p, fg);
            forcesBrute(flock, p, fb);
            for (int i = 0; i < n; ++i) CHECK(fg[i] == fb[i]);  // 逐位相同，不是"足够接近"
            checked += n;
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j) neighbors += i != j && lengthSq(flock[i].pos - flock[j].pos) < p.radius * p.radius;
        }
        std::printf("boids: grid forces identical to brute force for %lld boids (%.1f neighbors in radius on average)\n", checked,
                    double(neighbors) / checked);
    }

    // ---- Boids：长时间模拟后，速度在范围内，鸟群有序且不挤成一团 ----
    {
        using namespace ga::boids;
        Params p;
        double pol = 0, nn = 0;
        float worst = 1e9f;
        for (int seed = 1; seed <= 5; ++seed) {
            Pcg32 rng(seed);
            auto flock = randomFlock(150, p, rng);
            boids::FlockGrid grid;
            std::vector<Vec2> f;
            for (int i = 0; i < 60 * 60; ++i) {
                grid.build(flock, p), grid.forces(flock, p, f), integrate(flock, f, p, 1 / 60.0f);
                for (const Boid& b : flock) {
                    float s = length(b.vel);
                    CHECK(s >= p.minSpeed * 0.999f && s <= p.maxSpeed * 1.001f);
                }
            }
            float mean, min;
            nearestStats(flock, &mean, &min);
            pol += polarization(flock), nn += mean, worst = std::min(worst, min);
        }
        std::printf("150 boids after 60 s (5 seeds): polarization %.2f, nearest neighbour %.1f px (closest pair %.1f px)\n", pol / 5, nn / 5, worst);
        CHECK(pol / 5 > 0.6 && nn / 5 > 8);
    }
    return report("steering_test");
}
