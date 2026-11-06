// 数值积分的精度与开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/collision/integration/integration_bench.cpp -o int_bench && ./int_bench
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "integration.h"

using namespace ga;
using Clock = std::chrono::steady_clock;
using S = StateT<Vec2d>;

enum Method { Euler, SemiImplicit, Verlet, PosVerlet, Rk4 };
static const char* kNames[] = {"explicit Euler", "semi-implicit Euler", "velocity Verlet", "position Verlet", "RK4"};
static const int kEvals[] = {1, 1, 1, 1, 4};  // 每步计算几次加速度

// 推进 n 步，每步之后调用 visit(state)
template <class A, class F>
static S simulate(Method m, S s, double dt, int n, const A& acc, F&& visit) {
    if (m == Verlet) {
        auto vs = makeVerlet(s, acc);
        for (int i = 0; i < n; ++i) vs = velocityVerlet(vs, dt, acc), visit(S{vs.x, vs.v});
        return {vs.x, vs.v};
    }
    if (m == PosVerlet) {
        auto ps = makePositionVerlet(s, dt, acc);
        for (int i = 0; i < n; ++i) {
            auto next = positionVerlet(ps, acc);
            visit(S{next.x, (next.x - ps.x) / dt});
            ps = next;
        }
        return {ps.x, (ps.x - ps.prev) / dt};
    }
    for (int i = 0; i < n; ++i) {
        s = m == Euler ? explicitEuler(s, dt, acc) : m == SemiImplicit ? semiImplicitEuler(s, dt, acc) : rk4(s, dt, acc);
        visit(s);
    }
    return s;
}

int main() {
    std::printf("1) jump apex (g = 1400 px/s^2, launch 480 px/s, exact apex %.2f px) at different frame rates\n", 480.0 * 480 / 2800);
    std::printf("  %-6s", "fps");
    for (int m : {Euler, SemiImplicit, Verlet, Rk4}) std::printf(" %20s", kNames[m]);
    std::printf("\n");
    for (int fps : {20, 30, 60, 144}) {
        std::printf("  %-6d", fps);
        Constant<Vec2d> g{{0, -1400}};
        for (int m : {Euler, SemiImplicit, Verlet, Rk4}) {
            double apex = 0;
            simulate(Method(m), S{{0, 0}, {0, 480}}, 1.0 / fps, fps, g, [&](const S& s) { apex = std::max(apex, s.x.y); });
            std::printf(" %18.2fpx", apex);
        }
        std::printf("\n");
    }

    std::printf("\n2) cost to reach a given accuracy: eccentric orbit (e = 0.44), 10 periods, position error < tol\n");
    {
        Gravity gr{1};
        S s0{{1, 0}, {0, 1.2}};
        double E0 = gr.energy(s0), a = -1 / (2 * E0), period = 2 * kPi * std::pow(a, 1.5), T = 10 * period;
        // 参考解：RK4 用极小的步长
        S ref = simulate(Rk4, s0, T / 2000000, 2000000, gr, [](const S&) {});
        std::printf("  %-20s %14s %14s %14s\n", "method", "tol 1e-1", "tol 1e-3", "tol 1e-6");
        for (int m : {Euler, SemiImplicit, Verlet, Rk4}) {
            std::printf("  %-20s", kNames[m]);
            for (double tol : {1e-1, 1e-3, 1e-6}) {
                // 步数每次乘 1.25，直到误差达标（最多 2 亿次加速度计算）
                long found = -1;
                for (double n = 50; n * kEvals[m] < 2e8; n *= 1.25) {
                    S end = simulate(Method(m), s0, T / std::floor(n), static_cast<int>(n), gr, [](const S&) {});
                    if (length(end.x - ref.x) < tol) {
                        found = static_cast<long>(n) * kEvals[m];
                        break;
                    }
                }
                if (found < 0) std::printf(" %14s", "> 2e8");
                else std::printf(" %14ld", found / 10);  // 每个周期的加速度计算次数
            }
            std::printf("\n");
        }
        std::printf("  (numbers are acceleration evaluations per orbit)\n");
    }

    std::printf("\n3) float vs double: RK4 on a spring, error after 4 periods\n");
    {
        Spring sp{1};
        for (int n : {16, 32, 64, 128, 256, 512, 1024, 4096}) {
            double T = 8 * std::acos(-1.0);  // 用双精度的 π：float 的 kPi 本身就有 1e-7 的误差
            S d{{1, 0}, {0, 1}};
            State f{{1, 0}, {0, 1}};
            for (int i = 0; i < n; ++i) d = rk4(d, T / n, sp), f = rk4(f, T / n, sp);
            double ed = length(d.x - Vec2d{1, 0}), ef = std::hypot(f.x.x - 1.0, double(f.x.y));
            std::printf("  steps %5d: double %.2e  float %.2e\n", n, ed, ef);
        }
    }

    std::printf("\n4) velocity-dependent force: damped spring a = -x - 0.5 v, convergence order\n");
    {
        auto damped = [](const Vec2d& x, const Vec2d& v) { return -x - v * 0.5; };
        S s0{{1, 0}, {0, 0}};
        double T = 6;
        S ref = simulate(Rk4, s0, T / 100000, 100000, damped, [](const S&) {});
        for (int m : {SemiImplicit, Verlet, PosVerlet, Rk4}) {
            double e[3];
            for (int k = 0; k < 3; ++k) {
                int n = (m == Rk4 ? 25 : 200) << k;
                e[k] = length(simulate(Method(m), s0, T / n, n, damped, [](const S&) {}).x - ref.x);
            }
            std::printf("  %-20s order %.2f, %.2f   (error %.2e at the finest step)\n", kNames[m], std::log2(e[0] / e[1]),
                        std::log2(e[1] / e[2]), e[2]);
        }
    }

    std::printf("\n5) throughput: 1,000,000 particles (float Vec2), spring force, ns per particle step\n");
    {
        const int n = 1000000;
        std::vector<State> ps(n);
        for (int i = 0; i < n; ++i) ps[i] = {{float(i % 1000), float(i / 1000)}, {1, 0}};
        std::vector<VerletStateT<Vec2>> vs(n);
        Spring sp{2};
        auto acc = [&](const Vec2& x, const Vec2&) { return x * -4.0f; };
        for (int i = 0; i < n; ++i) vs[i] = makeVerlet(ps[i], acc);
        auto bench = [&](const char* name, auto&& stepAll) {
            double best = 1e30;
            for (int r = 0; r < 7; ++r) {
                auto t0 = Clock::now();
                stepAll();
                best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / n);
            }
            std::printf("  %-20s %.2f ns\n", name, best);
        };
        const double dt = 1.0 / 60;
        bench("explicit Euler", [&] { for (auto& p : ps) p = explicitEuler(p, dt, acc); });
        bench("semi-implicit Euler", [&] { for (auto& p : ps) p = semiImplicitEuler(p, dt, acc); });
        bench("velocity Verlet", [&] { for (auto& p : vs) p = velocityVerlet(p, dt, acc); });
        bench("RK4", [&] { for (auto& p : ps) p = rk4(p, dt, acc); });
        double sink = 0;
        for (auto& p : ps) sink += p.x.x;
        for (auto& p : vs) sink += p.x.x;
        std::printf("  (checksum %g)\n", sink);
        (void)sp;
    }
}
