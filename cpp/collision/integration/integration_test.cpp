#include "integration.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <cstdio>

#include "check.h"

using namespace ga;
using S = StateT<Vec2d>;

static double err(Vec2d a, Vec2d b) { return length(a - b); }

// 弹簧 ω = 1，从 (1, 0) 以速度 (0, 1) 出发：精确解是单位圆 x = (cos t, sin t)
static Vec2d exactCircle(double t) { return {std::cos(t), std::sin(t)}; }

enum Method { Euler, SemiImplicit, Verlet, PosVerlet, Rk4 };

// 用某种方法把弹簧推进到时刻 T（步长 dt = T / n），返回位置
template <class A>
static Vec2d advance(Method m, S s, double T, int n, const A& acc) {
    double dt = T / n;
    if (m == Verlet) {
        auto vs = makeVerlet(s, acc);
        for (int i = 0; i < n; ++i) vs = velocityVerlet(vs, dt, acc);
        return vs.x;
    }
    if (m == PosVerlet) {
        auto ps = makePositionVerlet(s, dt, acc);
        for (int i = 0; i < n; ++i) ps = positionVerlet(ps, acc);
        return ps.x;
    }
    for (int i = 0; i < n; ++i)
        s = m == Euler ? explicitEuler(s, dt, acc) : m == SemiImplicit ? semiImplicitEuler(s, dt, acc) : rk4(s, dt, acc);
    return s.x;
}

int main() {
    // ---- 恒定重力：各方法的误差有精确的公式 ----
    {
        Constant<Vec2d> g{{0, -10}};
        S s{{0, 0}, {3, 20}};
        for (double dt : {0.25, 1.0 / 30, 1.0 / 60}) {
            int n = static_cast<int>(std::round(2.0 / dt));
            double t = n * dt;
            double exact = 20 * t - 5 * t * t;
            S e = s, si = s, r = s;
            auto vs = makeVerlet(s, g);
            for (int i = 0; i < n; ++i) e = explicitEuler(e, dt, g), si = semiImplicitEuler(si, dt, g), r = rk4(r, dt, g), vs = velocityVerlet(vs, dt, g);
            CHECK(std::abs(e.x.y - (20 * t - 5 * t * (t - dt))) < 1e-9);   // 显式欧拉：½·g·t·(t − dt)
            CHECK(std::abs(si.x.y - (20 * t - 5 * t * (t + dt))) < 1e-9);  // 半隐式欧拉：½·g·t·(t + dt)
            CHECK(std::abs(vs.x.y - exact) < 1e-9);                         // Verlet 与 RK4：恰好精确
            CHECK(std::abs(r.x.y - exact) < 1e-9);
            CHECK(std::abs(e.v.y - (20 - 10 * t)) < 1e-9 && std::abs(si.v.y - (20 - 10 * t)) < 1e-9);  // 速度都是精确的
        }
        // float 版本同样可用
        State f{{0, 0}, {0, 10}};
        Constant<Vec2> gf{{0, -10}};
        for (int i = 0; i < 60; ++i) f = rk4(f, 1.0 / 60, gf);
        CHECK(std::abs(f.x.y - 5.0f) < 1e-4f);
    }

    // ---- 收敛阶：步长减半，误差变为 1/2^p ----
    {
        Spring sp{1};
        S s{{1, 0}, {0, 1}};
        const double T = 4;
        const double expected[] = {1, 1, 2, 2, 4};
        for (int m = Euler; m <= Rk4; ++m) {
            int n0 = m == Rk4 ? 10 : 100;
            double e1 = err(advance(Method(m), s, T, n0, sp), exactCircle(T));
            double e2 = err(advance(Method(m), s, T, 2 * n0, sp), exactCircle(T));
            double e3 = err(advance(Method(m), s, T, 4 * n0, sp), exactCircle(T));
            double p1 = std::log2(e1 / e2), p2 = std::log2(e2 / e3);
            std::printf("method %d: order %.2f, %.2f\n", m, p1, p2);
            CHECK(std::abs(p2 - expected[m]) < 0.15);
        }
        // 位置 Verlet 与速度 Verlet 的轨迹相同
        CHECK(err(advance(Verlet, s, T, 137, sp), advance(PosVerlet, s, T, 137, sp)) < 1e-9);
    }

    // ---- 能量：长时间模拟弹簧（1000 个周期，ω·dt = 0.1）----
    {
        Spring sp{1};
        S s0{{1, 0}, {0, 0}};
        double E0 = sp.energy(s0), dt = 0.1;
        int n = static_cast<int>(1000 * 2 * kPi / dt);
        S e = s0, si = s0, r = s0;
        auto vs = makeVerlet(s0, sp);
        double maxSi = 0, maxV = 0, prevR = E0;
        bool rkMonotone = true;
        for (int i = 0; i < n; ++i) {
            e = explicitEuler(e, dt, sp), si = semiImplicitEuler(si, dt, sp), r = rk4(r, dt, sp), vs = velocityVerlet(vs, dt, sp);
            maxSi = std::max(maxSi, std::abs(sp.energy(si) / E0 - 1));
            maxV = std::max(maxV, std::abs(sp.energy(S{vs.x, vs.v}) / E0 - 1));
            rkMonotone &= sp.energy(r) <= prevR + 1e-15;
            prevR = sp.energy(r);
        }
        std::printf("spring, 1000 periods: Euler E x%.3g, semi-implicit max dev %.4f, Verlet max dev %.5f, RK4 E x%.6f\n",
                    sp.energy(e) / E0, maxSi, maxV, sp.energy(r) / E0);
        CHECK(sp.energy(e) / E0 > 1e10);   // 显式欧拉：每步能量乘以 (1 + ω²dt²)，指数增长
        CHECK(maxSi < 0.06);              // 半隐式欧拉：能量上下波动，但不漂移
        CHECK(maxV < 0.003);              // Verlet：波动更小
        CHECK(rkMonotone);                // RK4：能量单调减少
        CHECK(sp.energy(r) / E0 < 1 && sp.energy(r) / E0 > 0.99);
    }

    // ---- 稳定性：步长超过某个界限，弹簧就会"爆炸" ----
    {
        Spring sp{1};
        S s0{{1, 0}, {0, 0}};
        auto blowsUp = [&](Method m, double dt) {
            Vec2d x = advance(m, s0, dt * 2000, 2000, sp);
            return !(length(x) < 100);
        };
        CHECK(!blowsUp(SemiImplicit, 1.9) && blowsUp(SemiImplicit, 2.1));  // 界限 ω·dt = 2
        CHECK(!blowsUp(Verlet, 1.9) && blowsUp(Verlet, 2.1));
        CHECK(!blowsUp(Rk4, 2.7) && blowsUp(Rk4, 2.9));  // 界限 2√2 ≈ 2.83
        CHECK(blowsUp(Euler, 0.2));                       // 显式欧拉没有界限：任何步长最终都会发散
    }

    // ---- 轨道：100 圈 ----
    {
        Gravity gr{1};
        S s0{{1, 0}, {0, 1.2}};  // 偏心率约 0.44 的椭圆
        double E0 = gr.energy(s0), dt = 0.01;
        double a = -1 / (2 * E0), period = 2 * kPi * std::pow(a, 1.5);
        int n = static_cast<int>(100 * period / dt);
        S e = s0, si = s0, r = s0;
        double maxSi = 0;
        for (int i = 0; i < n; ++i) {
            e = explicitEuler(e, dt, gr), si = semiImplicitEuler(si, dt, gr), r = rk4(r, dt, gr);
            maxSi = std::max(maxSi, std::abs(gr.energy(si) / E0 - 1));
        }
        std::printf("orbit, 100 periods: Euler E %.3f (start %.3f), semi-implicit max dev %.4f, RK4 E dev %.2e\n", gr.energy(e), E0,
                    maxSi, gr.energy(r) / E0 - 1);
        CHECK(gr.energy(e) > E0 + 0.1);  // 显式欧拉：能量增加，轨道越飞越远
        CHECK(maxSi < 0.02);
        CHECK(std::abs(gr.energy(r) / E0 - 1) < 1e-6);
    }
    return report("integration_test");
}
