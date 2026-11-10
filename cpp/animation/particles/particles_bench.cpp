// 粒子系统：发射数与帧率、拖尾的均匀程度、积分与帧率、删除策略、内存布局（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/animation/particles/particles_bench.cpp -o particles_bench && ./particles_bench [section]
//   section：emit trail integrate death layout（缺省全部）
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "particles.h"

using namespace ga;
using namespace ga::particles;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
static double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
static const char* modeName(EmitMode m) {
    switch (m) {
        case EmitMode::Truncate: return "truncate";
        case EmitMode::Round: return "round";
        case EmitMode::Accumulate: return "accumulate";
        default: return "subframe";
    }
}

static void emit() {
    std::printf("== emit: particles per second actually emitted (10 s)\n");
    for (float rate : {30.0f, 100.0f, 1000.0f}) {
        for (EmitMode m : {EmitMode::Truncate, EmitMode::Round, EmitMode::Accumulate}) {
            std::printf("  rate %5.0f %-10s", rate, modeName(m));
            for (int fps : {30, 60, 144, 240}) {
                Emitter e;
                e.rate = rate, e.mode = m;
                int total = 0;
                for (int f = 0; f < fps * 10; ++f) total += e.emit(1.0f / fps, {0, 0}, {0, 0}, [](Vec2, float) {});
                std::printf("  %3d fps %6.1f", fps, total / 10.0);
            }
            std::printf("\n");
        }
    }
}

// 发射器沿半径 150 的圆每秒转 2 圈（约 1,900 px/s），每秒发射 300 个，粒子以 60 px/s 向外飘。
// 帧末时相邻两个粒子的距离：均匀时应当都差不多（变异系数接近 0）
static void trail() {
    std::printf("== trail: emitter on a circle at ~1900 px/s, 300 particles/s; spacing between consecutive particles at frame end\n");
    for (EmitMode m : {EmitMode::Accumulate, EmitMode::Subframe}) {
        for (int fps : {20, 30, 60, 144}) {
            Emitter e;
            e.rate = 300, e.mode = m;
            float dt = 1.0f / fps, t = 0;
            auto at = [](float t) { return Vec2{150 * std::cos(t * 4 * kPi), 150 * std::sin(t * 4 * kPi)}; };
            std::vector<Vec2> pos, vel;
            for (int f = 0; f < fps * 3; ++f) {
                Vec2 from = at(t);
                t += dt;
                Vec2 to = at(t);
                for (size_t i = 0; i < pos.size(); ++i) pos[i] += vel[i] * dt;
                e.emit(dt, from, to, [&](Vec2 p, float age) {
                    Vec2 v = normalized(p) * 60;
                    pos.push_back(p + v * age), vel.push_back(v);  // 补走已经存在的时间
                });
            }
            // 最后 300 个粒子（最近一秒）的相邻间距
            std::vector<double> d;
            for (size_t i = pos.size() - 300; i + 1 < pos.size(); ++i) d.push_back(length(pos[i + 1] - pos[i]));
            double mean = 0, m2 = 0, mx = 0;
            int zero = 0;
            for (double x : d) mean += x, m2 += x * x, mx = std::max(mx, x), zero += x < 0.5;
            mean /= d.size();
            std::printf("  %-10s %3d fps  spacing mean %5.2f  cv %.2f  max %5.1f  coincident pairs %3d / %zu\n", modeName(m), fps, mean, std::sqrt(m2 / d.size() - mean * mean) / mean, mx, zero, d.size());
        }
    }
}

// 喷泉：v0 = (150, −600)，g = 980，阻力 k；1 秒后的位置和最高点，与精确解的差
static void integrate() {
    std::printf("== integrate: fountain v0 (150,-600), g 980; error vs exact after 1 s, and apex height error (px)\n");
    for (float k : {0.0f, 1.5f}) {
        Vec2 g{0, 980}, v0{150, -600};
        Vec2 exact = positionAt({0, 0}, v0, g, k, 1);
        // 精确的最高点：竖直速度为零的时刻（解析）
        double tApex = k > 0 ? std::log((g.y / k - v0.y) / (g.y / k)) / k : -v0.y / g.y;
        float apexExact = positionAt({0, 0}, v0, g, k, float(tApex)).y;
        for (Integrator m : {Integrator::Explicit, Integrator::SemiImplicit, Integrator::Exact}) {
            std::printf("  k %.1f %-13s", k, m == Integrator::Explicit ? "explicit" : m == Integrator::SemiImplicit ? "semi-implicit" : "exact");
            for (int fps : {30, 60, 144, 240}) {
                Vec2 p{0, 0}, v = v0;
                float apex = 0;
                for (int f = 0; f < fps; ++f) integrate(p, v, g, k, 1.0f / fps, m), apex = std::min(apex, p.y);
                std::printf("  %3d fps: pos %6.2f apex %+6.2f", fps, length(p - exact), apex - apexExact);
            }
            std::printf("\n");
        }
    }
}

// 删除策略：稳态约 N 个粒子，寿命 U(0.5, 1.5) 秒，60 fps
struct Fat {  // 一个典型的 AoS 粒子：位置、速度之外还有颜色、大小、旋转等冷数据
    Vec2 pos, vel;
    float age, life;
    uint32_t id;
    float size, rot, spin, color[4], pad[3];
};
static void death() {
    std::printf("== death: steady state ~N particles, life U(0.5,1.5) s, 60 fps; ms per frame (best of 3 runs of 240 frames) and age order kept?\n");
    for (int N : {10000, 100000}) {
        const float dt = 1 / 60.0f, rate = float(N);  // 平均寿命 1 秒 → 稳态约 N 个
        auto lifeOf = [](Pcg32& r) { return 0.5f + r.nextFloat(); };
        auto run = [&](const char* name, auto&& frame) {
            double best = 1e30;
            bool ordered = true;
            for (int rep = 0; rep < 3; ++rep) {
                auto t0 = Clock::now();
                ordered = frame();
                best = std::min(best, msSince(t0) / 240);
            }
            std::printf("  N %6d %-26s %8.3f ms/frame  age order %s\n", N, name, best, ordered ? "kept" : "lost");
        };
        // 预热到稳态的帮助函数：每个策略自己维护容器
        run("AoS erase each dead", [&] {
            std::vector<Fat> v;
            Pcg32 r(1);
            uint32_t next = 0;
            Emitter e;
            e.rate = rate, e.mode = EmitMode::Accumulate;
            for (int f = 0; f < 60 + 240; ++f) {
                for (size_t i = 0; i < v.size();) {
                    v[i].age += dt;
                    if (v[i].age >= v[i].life) v.erase(v.begin() + long(i));
                    else v[i].pos += v[i].vel * dt, ++i;
                }
                e.emit(dt, {0, 0}, {0, 0}, [&](Vec2, float) { v.push_back({{0, 0}, {1, 1}, 0, lifeOf(r), next++, 1, 0, 0, {1, 1, 1, 1}, {}}); });
            }
            return std::is_sorted(v.begin(), v.end(), [](const Fat& a, const Fat& b) { return a.id < b.id; });
        });
        run("AoS remove_if per frame", [&] {
            std::vector<Fat> v;
            Pcg32 r(1);
            uint32_t next = 0;
            Emitter e;
            e.rate = rate, e.mode = EmitMode::Accumulate;
            for (int f = 0; f < 60 + 240; ++f) {
                for (Fat& p : v) p.age += dt, p.pos += p.vel * dt;
                v.erase(std::remove_if(v.begin(), v.end(), [](const Fat& p) { return p.age >= p.life; }), v.end());
                e.emit(dt, {0, 0}, {0, 0}, [&](Vec2, float) { v.push_back({{0, 0}, {1, 1}, 0, lifeOf(r), next++, 1, 0, 0, {1, 1, 1, 1}, {}}); });
            }
            return std::is_sorted(v.begin(), v.end(), [](const Fat& a, const Fat& b) { return a.id < b.id; });
        });
        run("AoS swap-remove", [&] {
            std::vector<Fat> v;
            Pcg32 r(1);
            uint32_t next = 0;
            Emitter e;
            e.rate = rate, e.mode = EmitMode::Accumulate;
            for (int f = 0; f < 60 + 240; ++f) {
                for (size_t i = v.size(); i-- > 0;) {
                    v[i].age += dt;
                    if (v[i].age >= v[i].life) v[i] = v.back(), v.pop_back();
                    else v[i].pos += v[i].vel * dt;
                }
                e.emit(dt, {0, 0}, {0, 0}, [&](Vec2, float) { v.push_back({{0, 0}, {1, 1}, 0, lifeOf(r), next++, 1, 0, 0, {1, 1, 1, 1}, {}}); });
            }
            return std::is_sorted(v.begin(), v.end(), [](const Fat& a, const Fat& b) { return a.id < b.id; });
        });
        run("SoA swap-remove (Pool)", [&] {
            Pool pool;
            Pcg32 r(1);
            uint32_t next = 0;
            Emitter e;
            e.rate = rate, e.mode = EmitMode::Accumulate;
            for (int f = 0; f < 60 + 240; ++f) {
                pool.update(dt, {0, 0}, 0, Integrator::SemiImplicit);
                e.emit(dt, {0, 0}, {0, 0}, [&](Vec2, float) { pool.add({0, 0}, {1, 1}, lifeOf(r), next++); });
            }
            return std::is_sorted(pool.id.begin(), pool.id.end());
        });
        run("dead flag + free list", [&] {
            std::vector<Fat> v;
            std::vector<uint32_t> freeList;
            Pcg32 r(1);
            uint32_t next = 0;
            Emitter e;
            e.rate = rate, e.mode = EmitMode::Accumulate;
            for (int f = 0; f < 60 + 240; ++f) {
                for (size_t i = 0; i < v.size(); ++i) {
                    if (v[i].life < 0) continue;  // 空洞
                    v[i].age += dt;
                    if (v[i].age >= v[i].life) v[i].life = -1, freeList.push_back(uint32_t(i));
                    else v[i].pos += v[i].vel * dt;
                }
                e.emit(dt, {0, 0}, {0, 0}, [&](Vec2, float) {
                    Fat p{{0, 0}, {1, 1}, 0, lifeOf(r), next++, 1, 0, 0, {1, 1, 1, 1}, {}};
                    if (!freeList.empty()) v[freeList.back()] = p, freeList.pop_back();
                    else v.push_back(p);
                });
            }
            std::vector<uint32_t> ids;
            for (const Fat& p : v)
                if (p.life >= 0) ids.push_back(p.id);
            return std::is_sorted(ids.begin(), ids.end());
        });
        run("ring buffer (life fixed 1 s)", [&] {
            Ring<Fat> ring(size_t(N) * 2);
            uint32_t next = 0;
            Emitter e;
            e.rate = rate, e.mode = EmitMode::Accumulate;
            for (int f = 0; f < 60 + 240; ++f) {
                for (size_t i = 0; i < ring.size(); ++i) ring[i].age += dt, ring[i].pos += ring[i].vel * dt;
                while (ring.size() && ring[0].age >= ring[0].life) ring.popOldest();
                e.emit(dt, {0, 0}, {0, 0}, [&](Vec2, float) { ring.push({{0, 0}, {1, 1}, 0, 1.0f, next++, 1, 0, 0, {1, 1, 1, 1}, {}}); });
            }
            bool ok = true;
            for (size_t i = 1; i < ring.size(); ++i) ok &= ring[i - 1].id < ring[i].id;
            return ok;
        });
    }
}

// 内存布局：只做积分（不删），1M 个粒子
struct Hot {
    Vec2 pos, vel;
};
static void layout() {
    std::printf("== layout: integrate 1M particles (pos += vel*dt; vel += g*dt), ms per frame, best of 20\n");
    const size_t N = 1000000;
    const float dt = 1 / 60.0f;
    const Vec2 g{0, 980};
    std::vector<Hot> hot(N, {{0, 0}, {1, 1}});
    std::vector<Fat> fat(N, Fat{{0, 0}, {1, 1}, 0, 1, 0, 1, 0, 0, {1, 1, 1, 1}, {}});
    std::vector<Vec2> pos(N, {0, 0}), vel(N, {1, 1});
    std::vector<float> px(N, 0), py(N, 0), vx(N, 1), vy(N, 1);
    auto bench = [&](const char* name, size_t bytes, auto&& f) {
        double best = 1e30;
        for (int r = 0; r < 20; ++r) {
            auto t0 = Clock::now();
            f();
            best = std::min(best, msSince(t0));
        }
        std::printf("  %-34s %3zu B/particle  %6.3f ms\n", name, bytes, best);
    };
    bench("AoS {pos, vel}", sizeof(Hot), [&] {
        for (Hot& p : hot) p.vel += g * dt, p.pos += p.vel * dt;
    });
    bench("AoS fat (64 B: + color, size, ...)", sizeof(Fat), [&] {
        for (Fat& p : fat) p.vel += g * dt, p.pos += p.vel * dt;
    });
    bench("SoA pos[], vel[] (Vec2)", 16, [&] {
        for (size_t i = 0; i < N; ++i) vel[i] += g * dt, pos[i] += vel[i] * dt;
    });
    bench("SoA x[], y[], vx[], vy[]", 16, [&] {
        for (size_t i = 0; i < N; ++i) vx[i] += g.x * dt, vy[i] += g.y * dt, px[i] += vx[i] * dt, py[i] += vy[i] * dt;
    });
    bench("SoA x[], y[], ... in two loops", 16, [&] {  // 每个循环只读写 4 条内存流
        for (size_t i = 0; i < N; ++i) vx[i] += g.x * dt, px[i] += vx[i] * dt;
        for (size_t i = 0; i < N; ++i) vy[i] += g.y * dt, py[i] += vy[i] * dt;
    });
    volatile float sink = hot[7].pos.y + fat[7].pos.y + pos[7].y + py[7];
    (void)sink;
}

int main(int argc, char** argv) {
    if (want(argc, argv, "emit")) emit();
    if (want(argc, argv, "trail")) trail();
    if (want(argc, argv, "integrate")) integrate();
    if (want(argc, argv, "death")) death();
    if (want(argc, argv, "layout")) layout();
}
