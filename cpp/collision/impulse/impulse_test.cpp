#include "impulse.h"

#include <cmath>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

static Vec2 momentum(const World& w) {
    Vec2 p{0, 0};
    for (const Body& b : w.bodies) p += b.vel * b.mass();
    return p;
}
// 绕原点的角动量：Σ (m·r × v + I·ω)
static float angularMomentum(const World& w) {
    float l = 0;
    for (const Body& b : w.bodies) l += b.mass() * cross(b.pos, b.vel) + b.inertia() * b.angVel;
    return l;
}
static float kinetic(const World& w) {
    float e = 0;
    for (const Body& b : w.bodies) e += 0.5f * b.mass() * dot(b.vel, b.vel) + 0.5f * b.inertia() * b.angVel * b.angVel;
    return e;
}

// 两个圆正面相撞，直到分开
static World headOn(float m1, float m2, float v1, float v2, float e) {
    World w;
    w.gravity = {0, 0};
    w.opt.baumgarte = 0;
    Body a = makeCircleBody({-1.0f, 0}, 0.5f, m1 / (kPi * 0.25f)), b = makeCircleBody({0.02f, 0}, 0.5f, m2 / (kPi * 0.25f));
    a.vel = {v1, 0}, b.vel = {v2, 0};
    a.restitution = b.restitution = e;
    w.bodies = {a, b};
    for (int i = 0; i < 120; ++i) w.step(1.0f / 120);
    return w;
}

static World groundWorld() {
    World w;
    w.bodies.push_back(makeBoxBody({0, -0.5f}, {50, 0.5f}, 0));  // 地面：顶面在 y = 0
    return w;
}

int main() {
    // ---- 一维碰撞：与教科书公式对照 ----
    {
        World w = headOn(1, 1, 3, 0, 1);  // 等质量完全弹性：交换速度
        CHECK(std::abs(w.bodies[0].vel.x) < 1e-4f && std::abs(w.bodies[1].vel.x - 3) < 1e-4f);
        w = headOn(1, 1, 3, 0, 0);  // 完全非弹性：一起以 1.5 运动
        CHECK(std::abs(w.bodies[0].vel.x - 1.5f) < 1e-4f && std::abs(w.bodies[1].vel.x - 1.5f) < 1e-4f);
        w = headOn(1, 3, 4, 0, 1);  // v1' = (m1−m2)/(m1+m2)·v1 = −2，v2' = 2m1/(m1+m2)·v1 = 2
        CHECK(std::abs(w.bodies[0].vel.x + 2) < 1e-3f && std::abs(w.bodies[1].vel.x - 2) < 1e-3f);
        w = headOn(2, 1, 3, -1, 0.5f);  // 恢复系数：分离速度 = 0.5 × 接近速度
        float sep = w.bodies[1].vel.x - w.bodies[0].vel.x;
        CHECK(std::abs(sep - 0.5f * 4) < 1e-3f);
        CHECK(std::abs(2 * w.bodies[0].vel.x + w.bodies[1].vel.x - (2 * 3 - 1)) < 1e-3f);  // 动量守恒
    }

    // ---- 随机碰撞：动量、角动量守恒，动能不增加 ----
    {
        Pcg32 rng(8);
        int collisions = 0;
        float maxGainElastic = 0;
        for (int trial = 0; trial < 300; ++trial) {
            World w;
            w.gravity = {0, 0};
            w.opt.baumgarte = 0;  // 位置修正会凭空注入能量，这里关掉
            for (int k = 0; k < 2; ++k) {
                Vec2 p{k == 0 ? -1.2f : 1.2f, rnd(rng, -0.6f, 0.6f)};
                Body b = trial % 3 == 0 ? makeCircleBody(p, rnd(rng, 0.3f, 0.8f), rnd(rng, 0.5f, 3))
                                        : makeBoxBody(p, {rnd(rng, 0.2f, 0.8f), rnd(rng, 0.2f, 0.8f)}, rnd(rng, 0.5f, 3), rnd(rng, 0, 6));
                b.vel = {k == 0 ? rnd(rng, 1, 4) : -rnd(rng, 1, 4), rnd(rng, -0.5f, 0.5f)};
                b.angVel = rnd(rng, -2, 2);
                b.friction = rnd(rng, 0, 1), b.restitution = rnd(rng, 0, 1);
                w.bodies.push_back(b);
            }
            Vec2 p0 = momentum(w);
            float l0 = angularMomentum(w), e0 = kinetic(w);
            bool touched = false;
            for (int i = 0; i < 240; ++i) {
                w.step(1.0f / 240);
                touched |= !w.arbiters().empty();
            }
            if (!touched) continue;
            ++collisions;
            float scale = std::max(1.0f, length(p0));
            CHECK(length(momentum(w) - p0) < 1e-3f * scale);
            CHECK(std::abs(angularMomentum(w) - l0) < 2e-3f * std::max(1.0f, std::abs(l0) + scale));
            // 恢复系数接近 1、又有两个接触点时，顺序冲量可能让能量略微增加（每个点各自按碰撞前的速度定目标）
            float e = std::max(w.bodies[0].restitution, w.bodies[1].restitution);
            float gain = kinetic(w) / e0 - 1;
            if (e <= 0.9f) CHECK(gain < 1e-3f);
            else maxGainElastic = std::max(maxGainElastic, gain);
        }
        CHECK(collisions > 150);
        std::printf("max kinetic energy gain with restitution > 0.9: %.2f%%\n", maxGainElastic * 100);
        CHECK(maxGainElastic < 0.02f);
    }

    // ---- 摩擦：在地面上滑行的盒子，停下来的距离 = v² / (2μg) ----
    {
        World w = groundWorld();
        Body box = makeBoxBody({0, 0.5f}, {0.5f, 0.5f}, 1);
        box.vel = {5, 0};
        w.bodies.push_back(box);
        for (Body& b : w.bodies) b.friction = 0.5f;
        for (int i = 0; i < 300; ++i) w.step(1.0f / 60);
        float expected = 25 / (2 * 0.5f * 10);
        std::printf("sliding box stopped after %.3f m (expected %.3f)\n", w.bodies[1].pos.x, expected);
        CHECK(std::abs(w.bodies[1].pos.x - expected) < 0.03f * expected);
        CHECK(std::abs(w.bodies[1].vel.x) < 1e-3f && std::abs(w.bodies[1].angle) < 1e-3f);
    }

    // ---- 斜面：tanθ < μ 时静止，tanθ > μ 时以 g(sinθ − μcosθ) 加速下滑 ----
    for (float deg : {20.0f, 35.0f}) {
        float th = deg * kPi / 180;
        World w;
        Vec2 down{-std::cos(th), -std::sin(th)}, up = perp(down) * -1.0f;  // 沿斜面向下；斜面法线
        if (up.y < 0) up = -up;
        w.bodies.push_back(makeBoxBody({0, 0}, {20, 0.5f}, 0, th));
        Vec2 start = up * 1.0f;  // 盒子中心：在斜面顶面上方半个盒子
        w.bodies.push_back(makeBoxBody(start, {0.5f, 0.5f}, 1, th));
        for (Body& b : w.bodies) b.friction = 0.5f;
        for (int i = 0; i < 30; ++i) w.step(1.0f / 60);  // 先落稳
        Vec2 p0 = w.bodies[1].pos;
        float v0 = dot(w.bodies[1].vel, down);
        for (int i = 0; i < 60; ++i) w.step(1.0f / 60);
        float travelled = dot(w.bodies[1].pos - p0, down);
        if (deg == 20) {
            std::printf("incline 20 deg: moved %.4f m in 1 s\n", travelled);
            CHECK(std::abs(travelled) < 0.01f);
        } else {
            float a = 10 * (std::sin(th) - 0.5f * std::cos(th));
            float expected = v0 * 1 + 0.5f * a * 1;
            std::printf("incline 35 deg: moved %.3f m in 1 s (expected %.3f)\n", travelled, expected);
            CHECK(std::abs(travelled - expected) < 0.05f * expected);
        }
    }

    // ---- 静止：放在地上的盒子 10 秒后仍在原处 ----
    {
        World w = groundWorld();
        w.bodies.push_back(makeBoxBody({0, 0.5f}, {0.5f, 0.5f}, 1));
        float maxDrift = 0;
        for (int i = 0; i < 600; ++i) {
            w.step(1.0f / 60);
            maxDrift = std::max(maxDrift, std::abs(w.bodies[1].pos.y - 0.5f) + std::abs(w.bodies[1].pos.x));
        }
        CHECK(maxDrift < w.opt.slop + 0.005f);
        CHECK(length(w.bodies[1].vel) < 0.01f);
    }

    // ---- 堆叠：10 个盒子，有热启动时 10 秒后仍然立着 ----
    auto stackDrift = [](bool warm, int iterations) {
        World w = groundWorld();
        w.opt.warmStart = warm, w.opt.iterations = iterations;
        for (int i = 0; i < 10; ++i) w.bodies.push_back(makeBoxBody({0, 0.5f + i * 1.0f}, {0.5f, 0.5f}, 1));
        for (int i = 0; i < 600; ++i) w.step(1.0f / 60);
        const Body& top = w.bodies.back();
        return std::abs(top.pos.x) + std::abs(top.pos.y - 9.5f);
    };
    float warm = stackDrift(true, 10), cold = stackDrift(false, 10);
    std::printf("10-box stack after 10 s: top displaced %.3f m (warm start) / %.3f m (no warm start)\n", warm, cold);
    CHECK(warm < 0.1f);
    CHECK(cold > warm);

    // ---- 牛顿摆：5 个互相接触的球。顺序冲量得不到理想结果，但能量不能增加 ----
    // （曾经的错误：边计算反弹速度边热启动，后面的接触把前面热启动造成的速度当成"碰撞"，能量变成 3.9 倍）
    {
        World w;
        w.gravity = {0, 0};
        w.opt.iterations = 100, w.opt.baumgarte = 0, w.opt.restitutionThreshold = 0;
        for (int i = 0; i < 5; ++i) {
            Body b = makeCircleBody({i * 0.999f, 0}, 0.5f, 1);
            b.restitution = 1;
            if (i == 0) b.vel = {1, 0};
            w.bodies.push_back(b);
        }
        float e0 = kinetic(w);
        Vec2 p0 = momentum(w);
        for (int s = 0; s < 60; ++s) w.step(1.0f / 60);
        CHECK(kinetic(w) <= e0 * 1.001f);
        CHECK(length(momentum(w) - p0) < 1e-4f);
        CHECK(std::abs(w.bodies[0].vel.x + 0.6f) < 0.01f);  // 同时求解的结果：(−0.6, 0.4, 0.4, 0.4, 0.4)
    }
    return report("impulse_test");
}
