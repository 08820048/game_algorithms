// 相机：各种组合在平台跳跃轨迹上的表现、震屏与帧率、噪声通道错开、trauma 指数、物理与渲染频率不同时的抖动（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/animation/camera/camera_bench.cpp -o camera_bench && ./camera_bench [section]
//   section：rigs shake stagger exponent judder（缺省全部）
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "camera.h"
#include "trace.h"

using namespace ga;
using namespace ga::camera;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }

static void rigs() {
    std::printf("== rigs: 5 autopilot runs x 120 s at 60 Hz; screen 800 x 480\n");
    struct V {
        const char* name;
        RigOptions o;
    };
    std::vector<V> vs;
    RigOptions o;
    o.smooth = false;
    vs.push_back({"lock", o});
    o.smooth = true;
    vs.push_back({"smoothDamp 0.25 s", o});
    o.smooth = false, o.deadZone = {60, 40};
    vs.push_back({"dead zone 120x80", o});
    o.smooth = true;
    vs.push_back({"dead zone + smooth", o});
    o.platformSnap = true;
    vs.push_back({"+ platform snap", o});
    o.lookAhead = 100;
    vs.push_back({"+ look-ahead 100", o});
    o.turnHysteresis = 60;
    vs.push_back({"+ turn hysteresis 60", o});
    const float dt = 1 / 60.0f;
    for (const V& v : vs) {
        double travelX = 0, travelY = 0, acc2X = 0, acc2Y = 0, maxDx = 0, maxDy = 0, ahead = 0;
        long long frames = 0, aheadFrames = 0, reversals = 0;
        for (uint64_t seed = 1; seed <= 5; ++seed) {
            TraceOptions to;
            to.seed = seed;
            Autopilot ap(to);
            Rig rig(v.o);
            Vec2 prev{}, prevVel{};
            int lastSign = 0;
            for (int f = 0; f < 120 * 60; ++f) {
                ap.step(dt);
                PlayerSample s = ap.sample();
                rig.update(s, dt);
                Vec2 c = rig.position();
                if (f > 0) {
                    Vec2 vel = (c - prev) / dt;
                    travelX += std::fabs(c.x - prev.x), travelY += std::fabs(c.y - prev.y);
                    if (f > 1) {
                        Vec2 a = (vel - prevVel) / dt;
                        acc2X += double(a.x) * a.x, acc2Y += double(a.y) * a.y;
                    }
                    int sign = vel.x > 1 ? 1 : vel.x < -1 ? -1 : 0;  // 相机水平方向来回摆动的次数
                    if (sign != 0) {
                        if (lastSign != 0 && sign != lastSign) ++reversals;
                        lastSign = sign;
                    }
                    prevVel = vel;
                }
                prev = c;
                maxDx = std::max(maxDx, double(std::fabs(s.pos.x - c.x))), maxDy = std::max(maxDy, double(std::fabs(s.pos.y - c.y)));
                if (std::fabs(s.vel.x) > 1) {  // 移动时，前方还能看到多远
                    float dir = s.vel.x > 0 ? 1.0f : -1.0f;
                    ahead += (c.x + dir * 400 - s.pos.x) * dir, ++aheadFrames;
                }
                ++frames;
            }
        }
        double secs = frames * dt;
        std::printf("  %-22s travel x %5.0f y %4.0f px/s | rms accel x %6.0f y %6.0f px/s2 | max offset x %4.0f y %4.0f | sway %5.1f /min | ahead %4.0f px\n", v.name, travelX / secs, travelY / secs,
                    std::sqrt(acc2X / frames), std::sqrt(acc2Y / frames), maxDx, maxDy, reversals / (secs / 60), ahead / double(aheadFrames));
    }
}

// 震屏：同样的 trauma（保持为 1），不同帧率下偏移的大小和变化速度
static void shake() {
    std::printf("== shake: trauma held at 1, 10 s; rms offset, mean speed of the offset, correlation between frames 1/60 s apart\n");
    for (ShakeSource src : {ShakeSource::Random, ShakeSource::Noise}) {
        for (int fps : {30, 60, 144, 240}) {
            ShakeOptions o;
            o.source = src, o.decay = 0;
            Shake s(o);
            s.add(1);
            float dt = 1.0f / fps;
            std::vector<float> xs;
            double sum2 = 0, path = 0;
            for (int f = 0; f < fps * 10; ++f) {
                s.update(dt);
                xs.push_back(s.offset().x);
                sum2 += double(s.offset().x) * s.offset().x;
                if (f) path += std::fabs(xs[size_t(f)] - xs[size_t(f) - 1]);
            }
            // 相隔约 1/60 s 的两个样本的相关系数
            int lag = std::max(1, int(std::lround(fps / 60.0)));
            double num = 0, d = 0;
            for (size_t i = 0; i + size_t(lag) < xs.size(); ++i) num += double(xs[i]) * xs[i + size_t(lag)], d += double(xs[i]) * xs[i];
            std::printf("  %-6s %3d fps  rms %5.1f px  speed %6.0f px/s  corr(1/60 s) %+.2f\n", src == ShakeSource::Random ? "random" : "noise", fps, std::sqrt(sum2 / xs.size()), path / 10, num / d);
        }
    }
}

// 梯度噪声在整数格点上为 0：三个通道不错开时，会同时归零
static void stagger() {
    std::printf("== stagger: trauma 1, fraction of time |offset| < 10%% / 25%% of max (x and y together)\n");
    for (bool st : {false, true}) {
        ShakeOptions o;
        o.decay = 0, o.staggered = st;
        Shake s(o);
        s.add(1);
        int n = 0, below10 = 0, below25 = 0;
        for (int f = 0; f < 600000; ++f) {
            s.update(1 / 6000.0f);
            float m = length(s.offset()) / o.maxOffset;
            below10 += m < 0.1f, below25 += m < 0.25f, ++n;
        }
        std::printf("  staggered %-5s  <10%% %5.1f%%  <25%% %5.1f%%\n", st ? "yes" : "no", 100.0 * below10 / n, 100.0 * below25 / n);
    }
}

// trauma 的指数：一次满 trauma 的冲击，幅度超过最大值 50% / 10% 的时间
static void exponent() {
    std::printf("== exponent: full hit, decay 1.2/s; seconds with amplitude above 50%% / 10%% / 2%% of max\n");
    for (float e : {1.0f, 2.0f, 3.0f}) {
        double t50 = 0, t10 = 0, t2 = 0;
        float trauma = 1, dt = 1e-4f;
        while (trauma > 0) {
            float a = std::pow(trauma, e);
            t50 += (a > 0.5f) * dt, t10 += (a > 0.1f) * dt, t2 += (a > 0.02f) * dt;
            trauma -= 1.2f * dt;
        }
        std::printf("  exponent %.0f: %.2f / %.2f / %.2f s\n", e, t50, t10, t2);
    }
}

// 物理 50 Hz、渲染 60 / 144 Hz：角色匀速 300 px/s。每个渲染帧里角色在屏幕上、背景在屏幕上的位移的标准差（理想是 0）
static void judder() {
    std::printf("== judder: physics 50 Hz, player 300 px/s; std of per-frame screen motion (px) of player / background\n");
    for (int fps : {60, 144}) {
        for (int interp = 0; interp < 2; ++interp) {
            for (int smooth = 0; smooth < 2; ++smooth) {
                const double phys = 1.0 / 50, render = 1.0 / fps, v = 300;
                double acc = 0, t = 0, x = 0, prevX = 0;
                Rig rig({smooth == 1, 0.2f});
                std::vector<double> dp, db;
                double lastP = 0, lastB = 0;
                for (int f = 0; f < fps * 20; ++f) {
                    acc += render, t += render;
                    while (acc >= phys) prevX = x, x += v * phys, acc -= phys;
                    double alpha = acc / phys;
                    double shown = interp ? prevX + (x - prevX) * alpha : x;  // 渲染时角色的位置
                    rig.update({{float(shown), 0}, {float(v), 0}, true}, float(render));
                    double cam = rig.position().x;
                    double p = shown - cam, b = -cam;
                    if (f > fps) dp.push_back(p - lastP), db.push_back(b - lastB);
                    lastP = p, lastB = b;
                }
                auto sd = [](const std::vector<double>& d) {
                    double m = 0, m2 = 0;
                    for (double x : d) m += x, m2 += x * x;
                    m /= d.size();
                    return std::sqrt(std::max(0.0, m2 / d.size() - m * m));
                };
                std::printf("  render %3d Hz  %-14s camera %-10s player %5.2f  background %5.2f\n", fps, interp ? "interpolated" : "raw physics", smooth ? "smoothDamp" : "locked", sd(dp), sd(db));
            }
        }
    }
}

int main(int argc, char** argv) {
    if (want(argc, argv, "rigs")) rigs();
    if (want(argc, argv, "shake")) shake();
    if (want(argc, argv, "stagger")) stagger();
    if (want(argc, argv, "exponent")) exponent();
    if (want(argc, argv, "judder")) judder();
}
