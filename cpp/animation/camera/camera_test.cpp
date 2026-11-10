#include "camera.h"

#include <cmath>
#include <cstdio>
#include <vector>

#include "check.h"
#include "trace.h"

using namespace ga;
using namespace ga::camera;

// ---- 死区 ----
// 参照：在细网格上找"移动最少、又让目标落在框里"的相机位置
static void testDeadZone() {
    Pcg32 r(1);
    for (int trial = 0; trial < 2000; ++trial) {
        float cam = (r.nextFloat() * 2 - 1) * 100, target = (r.nextFloat() * 2 - 1) * 200, half = r.nextFloat() * 80;
        float best = cam, bestMove = 1e30f;
        for (int k = -40000; k <= 40000; ++k) {
            float c = k * 0.01f;
            if (std::fabs(target - c) <= half + 1e-4f && std::fabs(c - cam) < bestMove) bestMove = std::fabs(c - cam), best = c;
        }
        CHECK(std::fabs(deadZoneAxis(cam, target, half) - best) < 0.011f);
    }
}

// ---- 跟随 ----
static void testLockAndLag() {
    Rig lock({false, 0.25f});
    Autopilot ap;
    for (int f = 0; f < 600; ++f) {
        ap.step(1 / 60.0f);
        lock.update(ap.sample(), 1 / 60.0f);
        CHECK(lock.position() == ap.sample().pos);  // 不平滑、无死区：相机就是角色
    }
    // 匀速目标：smoothDamp 的稳态滞后 = 速度 × (smoothTime − dt/2)。多出的半帧来自目标每帧只在末尾采样一次
    for (float T : {0.1f, 0.25f, 0.5f})
        for (int fps : {30, 60, 240}) {
            Rig rig({true, T});
            float v = 300, x = 0, dt = 1.0f / fps;
            rig.reset({0, 0});
            for (int f = 0; f < fps * 6; ++f) {
                x += v * dt;
                rig.update({{x, 0}, {v, 0}, true}, dt);
            }
            CHECK(std::fabs((x - rig.position().x) - v * (T - dt / 2)) < 0.01f * v * T);
        }
    // 阶跃：临界阻尼，不过冲、单调接近
    Rig rig({true, 0.3f});
    rig.reset({0, 0});
    float prev = 0;
    for (int f = 0; f < 600; ++f) {
        rig.update({{100, 0}, {0, 0}, true}, 1 / 60.0f);
        CHECK(rig.position().x <= 100.001f && rig.position().x >= prev - 1e-4f);
        prev = rig.position().x;
    }
    CHECK(std::fabs(prev - 100) < 0.01f);
}

// 没有平滑时，角色永远在死区框内
static void testDeadZoneRig() {
    RigOptions o;
    o.smooth = false, o.deadZone = {60, 40};
    Rig rig(o);
    Autopilot ap;
    int moved = 0;
    Vec2 last{};
    for (int f = 0; f < 6000; ++f) {
        ap.step(1 / 60.0f);
        rig.update(ap.sample(), 1 / 60.0f);
        Vec2 d = ap.sample().pos - rig.position();
        CHECK(std::fabs(d.x) <= 60.001f && std::fabs(d.y) <= 40.001f);
        moved += !(rig.position() == last);
        last = rig.position();
    }
    CHECK(moved < 6000);
}

// 死区作用在期望位置上，而不是相机当前的位置：相机还在追的时候，角色在框里走动不会改变期望位置
static void testDeadZoneOnGoal() {
    RigOptions o;
    o.deadZone = {60, 40}, o.smoothTime = 0.5f;
    Rig rig(o);
    rig.reset({0, 0});
    rig.update({{200, 0}, {0, 0}, true}, 1 / 60.0f);
    CHECK(std::fabs(rig.goal().x - 140) < 1e-3f);
    CHECK(rig.position().x < 20);  // 相机才刚开始追
    rig.update({{130, 0}, {0, 0}, true}, 1 / 60.0f);
    CHECK(std::fabs(rig.goal().x - 140) < 1e-3f);
}

// 平台吸附：在空中（且没超出余量）时期望高度不变，落地后等于落地高度
static void testPlatformSnap() {
    RigOptions o;
    o.smooth = false, o.platformSnap = true, o.snapMargin = 1e9f;
    Rig rig(o);
    Autopilot ap;
    float lastGround = 0;
    int airborne = 0;
    for (int f = 0; f < 6000; ++f) {
        ap.step(1 / 60.0f);
        PlayerSample s = ap.sample();
        rig.update(s, 1 / 60.0f);
        if (s.grounded) lastGround = s.pos.y;
        else ++airborne;
        CHECK(std::fabs(rig.position().y - lastGround) < 1e-3f);
    }
    CHECK(airborne > 500);
    // 余量：角色离开吸附高度超过 margin 时，相机保持在 margin 处跟随
    o.snapMargin = 50;
    Rig r2(o);
    r2.reset({0, 0});
    r2.update({{0, 0}, {0, 0}, true}, 1 / 60.0f);
    r2.update({{0, 200}, {0, 500}, false}, 1 / 60.0f);
    CHECK(std::fabs(r2.position().y - 150) < 1e-3f);
    r2.update({{0, -30}, {0, 0}, false}, 1 / 60.0f);
    CHECK(std::fabs(r2.position().y - 0) < 1e-3f);
}

// 前视的滞回：往回走不到 turnHysteresis 不换边
static void testLookAheadHysteresis() {
    RigOptions o;
    o.smooth = false, o.lookAhead = 80, o.turnHysteresis = 60;
    Rig rig(o);
    rig.reset({0, 0});
    float x = 0;
    auto walk = [&](float v, float seconds) {
        for (int f = 0; f < int(seconds * 60); ++f) {
            x += v / 60;
            rig.update({{x, 0}, {v, 0}, true}, 1 / 60.0f);
        }
    };
    walk(200, 1);
    CHECK(rig.facing() == 1);
    walk(-200, 0.25f);  // 往回 50 px
    CHECK(rig.facing() == 1);
    walk(200, 0.5f);
    walk(-200, 0.4f);  // 往回 80 px
    CHECK(rig.facing() == -1);
    walk(0, 3);
    CHECK(std::fabs(rig.lookOffset() + 80) < 0.5f);
    // 没有滞回：一往回走就换边
    o.turnHysteresis = 0;
    Rig r2(o);
    r2.reset({0, 0});
    r2.update({{1, 0}, {200, 0}, true}, 1 / 60.0f);
    r2.update({{0, 0}, {-200, 0}, true}, 1 / 60.0f);
    CHECK(r2.facing() == -1);
}

// 帧率无关：同一条连续轨迹，30 fps 和 240 fps 下相机在同一时刻的位置只差半帧的滞后（约 v·Δdt/2 ≈ 3 px）
static void testFrameRateIndependence() {
    RigOptions o;
    o.deadZone = {40, 30}, o.lookAhead = 60, o.platformSnap = true;
    auto path = [](float t) { return Vec2{200 * t + 60 * std::sin(3 * t), 40 * std::sin(1.7f * t)}; };
    auto run = [&](int fps) {
        Rig rig(o);
        float dt = 1.0f / fps;
        for (int f = 1; f <= fps * 4; ++f) {
            float t = f * dt;
            Vec2 v = (path(t) - path(t - dt)) / dt;
            rig.update({path(t), v, true}, dt);
        }
        return rig.position();
    };
    CHECK(length(run(30) - run(240)) < 5.0f);
    CHECK(length(run(120) - run(240)) < 1.0f);
}

// ---- 震屏 ----
static void testShake() {
    for (ShakeSource src : {ShakeSource::Random, ShakeSource::Noise}) {
        ShakeOptions o;
        o.source = src;
        Shake s(o);
        s.update(1 / 60.0f);
        CHECK(s.offset() == (Vec2{0, 0}) && s.angle() == 0);  // 没有 trauma 就不抖
        s.add(0.7f);
        s.add(0.7f);
        CHECK(s.trauma() == 1);  // 上限 1
        for (int f = 0; f < 120; ++f) {
            s.update(1 / 60.0f);
            float amp = std::pow(s.trauma(), o.exponent);
            CHECK(std::fabs(s.offset().x) <= o.maxOffset * amp + 1e-4f && std::fabs(s.offset().y) <= o.maxOffset * amp + 1e-4f);
            CHECK(std::fabs(s.angle()) <= o.maxAngle * amp + 1e-6f);
        }
        CHECK(s.trauma() == 0 && s.offset() == (Vec2{0, 0}));  // 1 / decay 秒后完全停止
    }
    // 噪声：连续、按时间采样，所以 30 fps 和 120 fps 在同一时刻给出同样的偏移；随机源则不同
    for (ShakeSource src : {ShakeSource::Noise, ShakeSource::Random}) {
        ShakeOptions o;
        o.source = src;
        Shake a(o), b(o);
        a.add(1), b.add(1);
        for (int f = 0; f < 15; ++f) a.update(1 / 30.0f);
        for (int f = 0; f < 60; ++f) b.update(1 / 120.0f);
        if (src == ShakeSource::Noise) CHECK(length(a.offset() - b.offset()) < 0.05f);
        else CHECK(length(a.offset() - b.offset()) > 0.05f);
    }
    // 通道错开：噪声在整数格点上为 0；错开后 x 归零的时刻 y 不为 0
    for (bool st : {true, false}) {
        ShakeOptions o;
        o.decay = 0, o.staggered = st, o.frequency = 1;
        Shake sh(o);
        sh.add(1);
        int yZero = 0;
        for (int k = 0; k < 40; ++k) {
            sh.update(0.5f), sh.update(0.5f);  // 时间恰好是整数
            CHECK(std::fabs(sh.offset().x) < 1e-3f);
            yZero += std::fabs(sh.offset().y) < 1e-3f;
        }
        if (st) CHECK(yZero < 4);
        else CHECK(yZero == 40);
    }
    // x、y 两个通道互不相关
    {
        ShakeOptions o;
        o.decay = 0;
        Shake sh(o);
        sh.add(1);
        double sxy = 0, sxx = 0, syy = 0;
        for (int f = 0; f < 200000; ++f) {
            sh.update(1 / 600.0f);
            Vec2 v = sh.offset();
            sxy += double(v.x) * v.y, sxx += double(v.x) * v.x, syy += double(v.y) * v.y;
        }
        CHECK(std::fabs(sxy / std::sqrt(sxx * syy)) < 0.1);
    }
    // 一维梯度噪声：整数处为 0、连续、有界
    float maxAbs = 0;
    for (int i = 0; i < 100000; ++i) {
        float t = i * 0.001f;
        float n = noise1(7, t);
        maxAbs = std::max(maxAbs, std::fabs(n));
        CHECK(std::fabs(noise1(7, t + 1e-4f) - n) < 0.01f);
    }
    for (int k = -5; k < 50; ++k) CHECK(std::fabs(noise1(7, float(k))) < 1e-6f);
    CHECK(maxAbs <= 1.0f && maxAbs > 0.5f);
}

int main() {
    testDeadZone();
    testLockAndLag();
    testDeadZoneRig();
    testDeadZoneOnGoal();
    testPlatformSnap();
    testLookAheadHysteresis();
    testFrameRateIndependence();
    testShake();
    return report("camera_test");
}
