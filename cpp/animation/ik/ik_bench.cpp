// 反向动力学：收敛速度、可达边界附近的减速、弯曲的分布、跟随移动目标、角度限制、开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/animation/ik/ik_bench.cpp -o ik_bench && ./ik_bench [section]
//   section：converge boundary shape tracking limits（缺省全部）
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../../foundations/random/random.h"
#include "ik.h"

using namespace ga;
using namespace ga::ik;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
static double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
static const Method kAll[4] = {Method::CCD, Method::FABRIK, Method::Transpose, Method::DLS};
static const char* kName[4] = {"CCD", "FABRIK", "J-transpose", "DLS"};

static Chain equalChain(int n, float limit = 0) {
    Chain c = straightChain({0, 0}, std::vector<float>(size_t(n), 1.0f / n), {0, 1});
    c.limit = limit;
    return c;
}
static void randomPose(Pcg32& r, Chain& c, float spread) {
    std::vector<float> a;
    for (int i = 0; i < c.bones(); ++i) a.push_back((r.nextFloat() * 2 - 1) * spread);
    applyAngles(c, a);
}
static double pct(std::vector<double> v, double q) {
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(q * double(v.size())))];
}

static void converge() {
    std::printf("== converge: sweeps to reach 1e-3 of chain length (cap 1000), 1000 reachable targets, random start pose\n");
    for (int n : {2, 3, 5, 10, 20}) {
        for (int k = 0; k < 4; ++k) {
            Pcg32 r(uint64_t(n) * 31 + 7);
            std::vector<double> its;
            int ok = 0;
            double ms = 0;
            for (int trial = 0; trial < 1000; ++trial) {
                Chain c = equalChain(n), t = c;
                randomPose(r, t, float(M_PI));
                randomPose(r, c, float(M_PI));
                auto t0 = Clock::now();
                SolveResult res = solve(c, t.end(), kAll[k], 1000, 1e-3f);
                ms += msSince(t0);
                its.push_back(res.iterations);
                ok += res.error <= 1e-3f;
            }
            std::printf("  n %-2d %-12s success %5.1f%%  sweeps median %4.0f p90 %4.0f p99 %4.0f  time/solve %.1f us\n", n, kName[k], ok / 10.0, pct(its, 0.5), pct(its, 0.9), pct(its, 0.99), ms * 1000 / 1000);
        }
    }
}

// 目标离基座的距离（占总长的比例）和收敛速度的关系：5 节等长骨头
static void boundary() {
    std::printf("== boundary: sweeps to 1e-4 vs target distance / reach (n = 5 equal bones, cap 20000, 300 targets each)\n");
    for (double f : {0.02, 0.1, 0.3, 0.5, 0.7, 0.9, 0.99, 0.999}) {
        std::printf("  d/L %-6.3f", f);
        for (int k = 0; k < 4; ++k) {
            Pcg32 r(11);
            std::vector<double> its;
            for (int trial = 0; trial < 300; ++trial) {
                Chain c = equalChain(5);
                randomPose(r, c, 1.5f);
                Vec2 target = fromAngle(r.nextFloat() * 6.2832f) * float(f);
                its.push_back(solve(c, target, kAll[k], 20000, 1e-4f).iterations);
            }
            std::printf("  %s median %5.0f p90 %5.0f", kName[k], pct(its, 0.5), pct(its, 0.9));
        }
        std::printf("\n");
    }
    // 两节骨头：长度比不同时，最难的目标要多少趟
    std::printf("  two bones, worst of 2000 targets (sweeps to 1e-4, cap 100000):\n");
    for (float ratio : {1.0f, 2.0f, 4.0f}) {
        for (int order = 0; order < 2; ++order) {
            float l1 = order == 0 ? 1 : ratio, l2 = order == 0 ? ratio : 1;
            double worst[2] = {}, med[2];
            for (int k = 0; k < 2; ++k) {
                Pcg32 r(12);
                std::vector<double> its;
                for (int trial = 0; trial < 2000; ++trial) {
                    Chain c = straightChain({0, 0}, {l1, l2}, {1, 0});
                    float lo = std::fabs(l1 - l2), hi = l1 + l2;
                    Vec2 target = fromAngle(r.nextFloat() * 6.2832f) * (lo + (hi - lo) * (0.01f + 0.98f * r.nextFloat()));
                    its.push_back(solve(c, target, kAll[k], 100000, 1e-4f).iterations);
                }
                worst[k] = pct(its, 0.9999), med[k] = pct(its, 0.5);
            }
            std::printf("    l1 %.0f l2 %.0f: CCD median %.0f worst %.0f | FABRIK median %.0f worst %.0f\n", l1, l2, med[0], worst[0], med[1], worst[1]);
        }
    }
}

// 弯曲集中在哪里：10 节等长骨头，从竖直伸直的姿势出发，目标在 0.5L 处
static void shape() {
    std::printf("== shape: mean |relative angle| (deg) by joint, n = 10, start straight up, 1000 targets at 0.5 L\n");
    for (int k = 0; k < 4; ++k) {
        Pcg32 r(13);
        std::vector<double> mean(10, 0);
        double maxBend = 0, totalBend = 0;
        for (int trial = 0; trial < 1000; ++trial) {
            Chain c = equalChain(10);
            Vec2 target = fromAngle(r.nextFloat() * 6.2832f) * 0.5f;
            solve(c, target, kAll[k], 2000, 1e-4f);
            auto a = relativeAngles(c);
            double tb = 0, mb = 0;
            for (int i = 1; i < 10; ++i) {  // 第 0 节是整体朝向，不算弯曲
                double d = std::fabs(a[size_t(i)]) * 180 / M_PI;
                mean[size_t(i)] += d / 1000, tb += d, mb = std::max(mb, d);
            }
            totalBend += tb / 1000, maxBend += mb / 1000;
        }
        std::printf("  %-12s", kName[k]);
        for (int i = 1; i < 10; ++i) std::printf(" %5.1f", mean[size_t(i)]);
        std::printf("  | total %.0f  max joint %.0f\n", totalBend, maxBend);
    }
}

// 跟随移动的目标：每帧只给 budget 趟，从上一帧的姿势继续
static void tracking() {
    std::printf("== tracking: target on a circle (r 0.35 L, centre 0.5 L away), 1080 frames = 3 laps; n = 8; drift = pose after lap 3 vs after lap 2\n");
    for (int budget : {1, 3, 10}) {
        for (int k = 0; k < 4; ++k) {
            Chain c = equalChain(8);
            double errSum = 0, errMax = 0, jumpMax = 0;
            int pops = 0;
            std::vector<float> lap1;
            for (int f = 0; f < 1080; ++f) {
                float a = float(f) / 360 * 6.2832f;
                Vec2 target = Vec2{0.5f, 0.3f} + Vec2{std::cos(a), std::sin(a)} * 0.35f;
                auto before = relativeAngles(c);
                for (int b = 0; b < budget; ++b) sweep(c, target, kAll[k], 0.1f);
                auto after = relativeAngles(c);
                double jump = 0;
                for (size_t i = 0; i < before.size(); ++i) {
                    double d = std::fabs(after[i] - before[i]);
                    d = std::min(d, 2 * M_PI - d);
                    jump = std::max(jump, d * 180 / M_PI);
                }
                if (f >= 30) jumpMax = std::max(jumpMax, jump), pops += jump > 20;  // 前 30 帧是从初始姿势调整过来
                double e = length(c.end() - target);
                if (f >= 30) errSum += e, errMax = std::max(errMax, e);
                if (f == 719) lap1 = after;
            }
            auto lap2 = relativeAngles(c);
            double drift = 0;
            for (size_t i = 0; i < lap2.size(); ++i) drift = std::max(drift, double(std::fabs(lap2[i] - lap1[i])) * 180 / M_PI);
            std::printf("  budget %-2d %-12s err mean %.4f max %.4f | max joint change/frame %5.1f deg, frames > 20 deg %3d | lap-to-lap pose drift %5.2f deg\n", budget, kName[k], errSum / 1050, errMax, jumpMax, pops, drift);
        }
    }
}

// 角度限制：目标由满足限制的随机姿势生成，所以一定有解
static void limits() {
    std::printf("== limits: success (1e-3 L in 1000 sweeps; in parentheses: 20000) on targets reachable under the limits; n = 5, 1000 targets\n");
    for (float limDeg : {30.0f, 60.0f, 90.0f, 150.0f}) {
        float lim = limDeg * float(M_PI) / 180;
        std::printf("  limit %3.0f deg", limDeg);
        for (int k = 0; k < 4; ++k) {
            Pcg32 r(14);
            int ok = 0, okLong = 0;
            for (int trial = 0; trial < 1000; ++trial) {
                Chain c = equalChain(5, lim), t = c;
                std::vector<float> a;
                for (int i = 0; i < 5; ++i) a.push_back((r.nextFloat() * 2 - 1) * lim);
                applyAngles(t, a);
                ok += solve(c, t.end(), kAll[k], 1000, 1e-3f).error <= 1e-3f;
                okLong += solve(c, t.end(), kAll[k], 19000, 1e-3f).error <= 1e-3f;  // 再给 19000 趟
            }
            std::printf("  %s %5.1f%% (%5.1f%%)", kName[k], ok / 10.0, okLong / 10.0);
        }
        std::printf("\n");
    }
}

int main(int argc, char** argv) {
    if (want(argc, argv, "converge")) converge();
    if (want(argc, argv, "boundary")) boundary();
    if (want(argc, argv, "shape")) shape();
    if (want(argc, argv, "tracking")) tracking();
    if (want(argc, argv, "limits")) limits();
}
