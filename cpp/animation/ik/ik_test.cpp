#include "ik.h"

#include <cmath>
#include <cstdio>
#include <vector>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;
using namespace ga::ik;

static const Method kAll[4] = {Method::CCD, Method::FABRIK, Method::Transpose, Method::DLS};

static Chain randomChain(Pcg32& r, int n, float limit = 0) {
    std::vector<float> len;
    for (int i = 0; i < n; ++i) len.push_back(0.3f + r.nextFloat());
    Chain c = straightChain({0, 0}, len, {0, 1});
    c.limit = limit;
    // 随机的初始姿势（满足角度限制）
    std::vector<float> a;
    float lim = limit > 0 ? limit : float(M_PI);
    for (int i = 0; i < n; ++i) a.push_back((r.nextFloat() * 2 - 1) * lim);
    applyAngles(c, a);
    return c;
}
// 由随机角度得到的末端位置：一定可达
static Vec2 reachableTarget(Pcg32& r, const Chain& c) {
    Chain t = c;
    std::vector<float> a;
    float lim = c.limit > 0 ? c.limit : float(M_PI);
    for (int i = 0; i < c.bones(); ++i) a.push_back((r.nextFloat() * 2 - 1) * lim);
    applyAngles(t, a);
    return t.end();
}
static bool lengthsKept(const Chain& c, Vec2 base) {
    if (length(c.p[0] - base) > 1e-5f) return false;
    for (int i = 0; i < c.bones(); ++i)
        if (std::fabs(length(c.p[size_t(i) + 1] - c.p[size_t(i)]) - c.len[size_t(i)]) > 1e-3f * c.len[size_t(i)]) return false;
    return true;
}
static bool limitsKept(const Chain& c) {
    for (float a : relativeAngles(c))
        if (std::fabs(a) > c.limit + 1e-3f) return false;
    return true;
}

// ---- 不变量 ----
static void testInvariants() {
    Pcg32 r(1);
    for (int trial = 0; trial < 200; ++trial) {
        int n = 1 + int(r.below(12));
        float limit = r.chance(0.5) ? 0.3f + r.nextFloat() : 0;
        Chain c0 = randomChain(r, n, limit);
        Vec2 target = {(r.nextFloat() * 2 - 1) * c0.total() * 1.3f, (r.nextFloat() * 2 - 1) * c0.total() * 1.3f};
        for (Method m : kAll) {
            Chain c = c0;
            for (int k = 0; k < 30; ++k) {
                sweep(c, target, m, 0.1f * c.total());
                CHECK(lengthsKept(c, c0.p[0]));  // 骨头长度不变、基座不动
                if (limit > 0) CHECK(limitsKept(c));
            }
        }
    }
}

static void testAnglesRoundTrip() {
    Pcg32 r(2);
    for (int trial = 0; trial < 200; ++trial) {
        Chain c = randomChain(r, 1 + int(r.below(10)));
        Chain d = c;
        applyAngles(d, relativeAngles(c));
        for (size_t i = 0; i < c.p.size(); ++i) CHECK(length(c.p[i] - d.p[i]) < 1e-4f);
    }
    CHECK(length(clampDirection({1, 0}, {0, 1}, 0.5f) - Vec2{std::cos(0.5f), std::sin(0.5f)}) < 1e-6f);
    CHECK(length(clampDirection({1, 0}, {0, -2}, 0.5f) - Vec2{std::cos(0.5f), -std::sin(0.5f)} * 2) < 1e-5f);
    CHECK(clampDirection({1, 0}, {1, 0.1f}, 0.5f) == (Vec2{1, 0.1f}));
}

// ---- 收敛 ----
static void testReachable() {
    Pcg32 r(3);
    int ok[4] = {}, total = 0;
    for (int trial = 0; trial < 300; ++trial) {
        int n = 2 + int(r.below(9));
        Chain c0 = randomChain(r, n);
        Vec2 target = reachableTarget(r, c0);
        ++total;
        for (int k = 0; k < 4; ++k) {
            Chain c = c0;
            SolveResult res = solve(c, target, kAll[k], 20000, 1e-3f * c.total());  // 目标靠近可达边界时收敛很慢（见 ik_bench boundary）
            ok[k] += res.error <= 1e-3f * c.total();
        }
    }
    CHECK(ok[0] == total);  // CCD
    CHECK(ok[1] == total);  // FABRIK
    CHECK(ok[2] >= total * 95 / 100);  // 雅可比转置
    CHECK(ok[3] >= total * 95 / 100);  // DLS 有阻尼，靠近奇异姿势时很慢
}

// 够不到：伸直指向目标，误差 = |目标距离 − 总长|
static void testUnreachableFar() {
    Pcg32 r(4);
    for (int trial = 0; trial < 100; ++trial) {
        Chain c0 = randomChain(r, 2 + int(r.below(8)));
        Vec2 dir = fromAngle(r.nextFloat() * 6.2832f);
        float d = c0.total() * (1.1f + r.nextFloat());
        for (Method m : {Method::CCD, Method::FABRIK}) {
            Chain c = c0;
            SolveResult res = solve(c, dir * d, m, 500, 0);
            CHECK(std::fabs(res.error - (d - c.total())) < 2e-3f * c.total());
        }
    }
}

// 暴力参照：三节骨头，网格穷举相对转角，求末端到目标的最小距离（含够不到的内圈）
static float gridBest(const Chain& c, Vec2 target, int steps) {
    float best = 1e30f;
    float lim = c.limit > 0 ? c.limit : float(M_PI);
    Chain t = c;
    std::vector<float> a(3);
    for (int i = 0; i <= steps; ++i)
        for (int j = 0; j <= steps; ++j)
            for (int k = 0; k <= steps; ++k) {
                a[0] = -lim + 2 * lim * i / steps, a[1] = -lim + 2 * lim * j / steps, a[2] = -lim + 2 * lim * k / steps;
                applyAngles(t, a);
                best = std::min(best, length(t.end() - target));
            }
    return best;
}
static void testAgainstBruteForce() {
    Pcg32 r(5);
    // 一节长、两节短：内圈半径 0.5 以内够不到
    Chain c0 = straightChain({0, 0}, {1.0f, 0.3f, 0.2f}, {1, 0});
    int ok[2] = {}, n = 0;
    for (int trial = 0; trial < 60; ++trial) {
        Vec2 target = fromAngle(r.nextFloat() * 6.2832f) * (r.nextFloat() * 1.8f);
        float best = gridBest(c0, target, 72);  // 5° 一格：末端位置的误差不超过 1.5 × 5° ≈ 0.13
        for (int k = 0; k < 2; ++k) {
            Chain c = c0;
            SolveResult res = solve(c, target, k == 0 ? Method::CCD : Method::FABRIK, 500, 1e-4f);
            ok[k] += res.error <= best + 1e-3f;
        }
        ++n;
    }
    CHECK(ok[0] == n && ok[1] == n);
}

// 有角度限制：解永远满足限制；和网格穷举的最优比，记录有多少次卡在局部最优（实际比例见 ik_bench limits）
static void testLimitsAgainstBruteForce() {
    Pcg32 r(6);
    int worse[2] = {}, n = 0;
    for (int trial = 0; trial < 60; ++trial) {
        Chain c0 = straightChain({0, 0}, {0.6f, 0.5f, 0.4f}, {1, 0});
        c0.limit = 1.0f;
        Vec2 target = fromAngle(r.nextFloat() * 6.2832f) * (r.nextFloat() * 1.6f);
        float best = gridBest(c0, target, 60);
        for (int k = 0; k < 2; ++k) {
            Chain c = c0;
            SolveResult res = solve(c, target, k == 0 ? Method::CCD : Method::FABRIK, 500, 1e-4f);
            CHECK(limitsKept(c));
            CHECK(res.error >= best - 0.05f);  // 不可能比全局最优好（留出网格误差）
            worse[k] += res.error > best + 0.02f;
        }
        ++n;
    }
    CHECK(worse[0] < n && worse[1] < n);
}

// 退化的情况：目标正好在某个关节上、伸直的链指向正前方的目标（奇异姿势）
static void testDegenerate() {
    for (int n : {2, 3, 6}) {
        Chain c0 = straightChain({0, 0}, std::vector<float>(size_t(n), 1.0f), {1, 0});
        for (int j = 0; j < n; ++j) {
            for (Method m : {Method::CCD, Method::FABRIK, Method::Transpose, Method::DLS}) {
                Chain c = c0;
                Vec2 target = c0.p[size_t(j)];
                for (int k = 0; k < 5; ++k) sweep(c, target, m, 0.1f * c.total());
                CHECK(lengthsKept(c, {0, 0}));
            }
        }
        // 伸直的链、目标在正前方更远处：雅可比矩阵退化（秩为 1），DLS 靠阻尼保持有限
        Chain c = c0;
        SolveResult res = solve(c, {float(n) + 1, 0}, Method::DLS, 50, 0);
        CHECK(std::isfinite(res.error) && std::fabs(res.error - 1) < 1e-3f);
    }
}

// ---- 两节骨头的解析解 ----
static void testTwoBone() {
    Pcg32 r(7);
    for (int trial = 0; trial < 500; ++trial) {
        float l1 = 0.3f + r.nextFloat(), l2 = 0.3f + r.nextFloat();
        Chain c = straightChain({0, 0}, {l1, l2}, {1, 0});
        float lo = std::fabs(l1 - l2), hi = l1 + l2;
        float d = lo + (hi - lo) * (0.02f + 0.96f * r.nextFloat());
        Vec2 target = fromAngle(r.nextFloat() * 6.2832f) * d;
        for (float s : {1.0f, -1.0f}) {
            twoBone(c, target, s);
            CHECK(length(c.end() - target) < 1e-4f);
            CHECK(lengthsKept(c, {0, 0}));
            float bend = relativeAngles(c)[1];
            CHECK(bend * s <= 1e-4f);  // bendSign 决定肘部弯向哪边：+1 时第二节相对第一节顺时针转
        }
        // CCD 和 FABRIK 也能到达
        for (Method m : {Method::CCD, Method::FABRIK}) {
            Chain cc = straightChain({0, 0}, {l1, l2}, {1, 0});
            CHECK(solve(cc, target, m, 100000, 1e-4f).error <= 1e-4f);
        }
    }
    // 够不到：伸直
    Chain c = straightChain({0, 0}, {1, 1}, {1, 0});
    twoBone(c, {0, 5});
    CHECK(length(c.end() - Vec2{0, 2}) < 1e-3f);
}

int main() {
    testInvariants();
    testAnglesRoundTrip();
    testReachable();
    testUnreachableFar();
    testAgainstBruteForce();
    testLimitsAgainstBruteForce();
    testDegenerate();
    testTwoBone();
    return report("ik_test");
}
