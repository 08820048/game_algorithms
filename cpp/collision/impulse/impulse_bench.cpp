// 冲量求解器的稳定性与开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/collision/impulse/impulse_bench.cpp -o imp_bench && ./imp_bench
#include <chrono>
#include <cmath>
#include <cstdio>

#include "impulse.h"

using namespace ga;
using Clock = std::chrono::steady_clock;
constexpr float kDt = 1.0f / 60;

static World groundWorld() {
    World w;
    w.bodies.push_back(makeBoxBody({0, -0.5f}, {50, 0.5f}, 0));
    return w;
}

struct StackResult {
    float topDrift;     // 顶部盒子的水平偏移
    float sink;         // 顶部盒子下沉了多少（每层允许 slop 的穿透，会累积）
    float penetration;  // 最后一秒的最大穿透
    bool collapsed;
};

static StackResult stack(int height, World::Options opt, float seconds = 10) {
    World w = groundWorld();
    w.opt = opt;
    for (int i = 0; i < height; ++i) w.bodies.push_back(makeBoxBody({0, 0.5f + i * 1.0f}, {0.5f, 0.5f}, 1));
    int steps = static_cast<int>(seconds * 60);
    float pen = 0;
    for (int s = 0; s < steps; ++s) {
        w.step(kDt);
        if (s >= steps - 60)
            for (const Arbiter& a : w.arbiters())
                for (int k = 0; k < a.count; ++k) pen = std::max(pen, a.points[k].depth);
    }
    const Body& top = w.bodies.back();
    return {std::abs(top.pos.x), (height - 0.5f) - top.pos.y, pen, top.pos.y < height - 1.5f};
}

int main() {
    std::printf("1) box stacks after 10 s: horizontal drift / sink of the top box in cm (X = collapsed)\n");
    std::printf("  %-8s %-6s", "height", "warm");
    for (int it : {1, 2, 4, 8, 16, 32}) std::printf(" %8d it", it);
    std::printf("\n");
    for (int h : {5, 10, 20})
        for (bool warm : {true, false}) {
            std::printf("  %-8d %-6s", h, warm ? "on" : "off");
            for (int it : {1, 2, 4, 8, 16, 32}) {
                World::Options o;
                o.iterations = it, o.warmStart = warm;
                StackResult r = stack(h, o);
                if (r.collapsed) std::printf(" %11s", "X");
                else std::printf(" %5.1f/%-5.1f", r.topDrift * 100, r.sink * 100);
            }
            std::printf("\n");
        }

    std::printf("\n2) clamping the accumulated impulse vs clamping each increment (10-box stack, 10 iterations, warm start)\n");
    for (bool acc : {true, false}) {
        World::Options o;
        o.accumulate = acc;
        StackResult r = stack(10, o);
        if (r.collapsed) std::printf("  %-22s collapsed\n", acc ? "accumulated (correct)" : "per increment");
        else std::printf("  %-22s drift %.1f cm, sink %.1f cm, max penetration %.1f mm\n", acc ? "accumulated (correct)" : "per increment",
                         r.topDrift * 100, r.sink * 100, r.penetration * 1000);
    }

    std::printf("\n3) Baumgarte factor: resting penetration (10-box stack) and pop-out speed of a box starting 0.3 m inside the ground\n");
    for (float beta : {0.0f, 0.05f, 0.1f, 0.2f, 0.5f, 0.8f, 1.0f}) {
        World::Options o;
        o.baumgarte = beta;
        StackResult r = stack(10, o);
        World w = groundWorld();
        w.opt = o;
        w.bodies.push_back(makeBoxBody({0, 0.2f}, {0.5f, 0.5f}, 1));
        float maxUp = 0, maxY = 0;
        for (int s = 0; s < 120; ++s) {
            w.step(kDt);
            maxUp = std::max(maxUp, w.bodies[1].vel.y);
            maxY = std::max(maxY, w.bodies[1].pos.y);
        }
        std::printf("  beta %.2f: stack %s, penetration %6.1f mm | pop-out speed %5.2f m/s, rises to %.2f m (rest 0.50)\n", beta,
                    r.collapsed ? "collapsed" : "stands   ", r.penetration * 1000, maxUp, maxY);
    }

    std::printf("\n4) bouncing ball: dropped from 2 m, restitution 0.8 (ideal bounce heights 1.28, 0.82, 0.52)\n");
    {
        World w = groundWorld();
        Body ball = makeCircleBody({0, 2.5f}, 0.5f, 1);
        ball.restitution = 0.8f;
        w.bodies.push_back(ball);
        float prevVy = 0, peak = 0;
        int bounces = 0;
        std::printf("  bounce heights:");
        for (int s = 0; s < 4000 && bounces < 3; ++s) {
            w.step(1.0f / 240);
            const Body& b = w.bodies[1];
            peak = std::max(peak, b.pos.y - 0.5f);
            if (prevVy > 0 && b.vel.y <= 0) std::printf(" %.3f", peak), ++bounces;  // 速度由正转负：到达最高点
            if (prevVy <= 0 && b.vel.y > 0) peak = 0;                                  // 刚弹起
            prevVy = b.vel.y;
        }
        std::printf("  (dt = 1/240)\n");
    }

    std::printf("\n5) Newton's cradle: 5 balls in contact (overlapping by 1 mm), restitution 1, the first moves at 1 m/s (ideal: 0 0 0 0 1)\n");
    for (int it : {1, 10, 100}) {
        World w;
        w.gravity = {0, 0};
        w.opt.iterations = it;
        w.opt.baumgarte = 0;
        w.opt.restitutionThreshold = 0;
        for (int i = 0; i < 5; ++i) {
            Body b = makeCircleBody({i * 0.999f, 0}, 0.5f, 1);
            b.restitution = 1;
            if (i == 0) b.vel = {1, 0};
            w.bodies.push_back(b);
        }
        for (int s = 0; s < 60; ++s) w.step(kDt);
        std::printf("  %3d iterations:", it);
        for (const Body& b : w.bodies) std::printf(" %6.3f", b.vel.x);
        std::printf("\n");
    }

    std::printf("\n6) pyramid, ms per step (brute-force pairs + solver)\n");
    for (auto [rows, iters] : {std::pair{10, 10}, {20, 10}, {30, 10}, {30, 30}}) {
        World w = groundWorld();
        w.opt.iterations = iters;
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < rows - r; ++c) w.bodies.push_back(makeBoxBody({(c - (rows - r) / 2.0f) * 1.05f, 0.5f + r * 1.0f}, {0.5f, 0.5f}, 1));
        for (int s = 0; s < 120; ++s) w.step(kDt);  // 先稳定下来
        auto t0 = Clock::now();
        for (int s = 0; s < 300; ++s) w.step(kDt);
        double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / 300;
        float topY = 0;
        for (const Body& b : w.bodies) topY = std::max(topY, b.pos.y);
        std::printf("  %2d rows (%3zu boxes), %2d iterations: %.3f ms/step, %zu contacts, top at %.2f m (ideal %.2f)\n", rows,
                    w.bodies.size() - 1, iters, ms, w.arbiters().size(), topY, rows - 0.5f);
    }
}
