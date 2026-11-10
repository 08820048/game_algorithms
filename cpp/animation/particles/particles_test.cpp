#include "particles.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::particles;

// ---- 积分 ----
// 参照：半隐式欧拉，步长 1e-5 秒（双精度）
static void reference(double& px, double& py, double& vx, double& vy, double gx, double gy, double k, double T) {
    const double h = 1e-5;
    int n = int(T / h + 0.5);
    for (int i = 0; i < n; ++i) {
        vx += (gx - k * vx) * h, vy += (gy - k * vy) * h;
        px += vx * h, py += vy * h;
    }
}
static void testExactIntegrator() {
    Pcg32 r(1);
    for (int trial = 0; trial < 40; ++trial) {
        Vec2 p0{0, 0}, v0{(r.nextFloat() * 2 - 1) * 200, -r.nextFloat() * 400};
        Vec2 g{0, 300 + r.nextFloat() * 600};
        float k = trial % 4 == 0 ? 0 : r.nextFloat() * 4;
        double px = 0, py = 0, vx = v0.x, vy = v0.y;
        reference(px, py, vx, vy, g.x, g.y, k, 1.5);
        Vec2 p = positionAt(p0, v0, g, k, 1.5f);
        CHECK(std::fabs(p.x - px) < 0.02 && std::fabs(p.y - py) < 0.02);
        // 精确解可以分段走：走 90 小步和一步走完结果相同（与帧率无关）
        Vec2 q = p0, v = v0;
        for (int f = 0; f < 90; ++f) integrate(q, v, g, k, 1.5f / 90, Integrator::Exact);
        CHECK(length(q - p) < 1e-4f * length(p) + 0.01f);  // 只差 float 的舍入
        CHECK(std::fabs(v.x - vx) < 0.01 && std::fabs(v.y - vy) < 0.01);
    }
}
// 没有阻力时欧拉法的误差有解析式：显式 p = p0 + v0 t + g t (t − dt)/2；半隐式 p = p0 + v0 t + g t (t + dt)/2
static void testEulerErrorFormula() {
    for (int fps : {30, 60, 144}) {
        float dt = 1.0f / fps;
        int n = fps;  // 1 秒
        Vec2 g{0, 980}, v0{100, -500};
        Vec2 pe{0, 0}, ve = v0, ps{0, 0}, vs = v0;
        for (int i = 0; i < n; ++i) integrate(pe, ve, g, 0, dt, Integrator::Explicit), integrate(ps, vs, g, 0, dt, Integrator::SemiImplicit);
        float t = n * dt;
        CHECK(std::fabs(pe.y - (v0.y * t + g.y * t * (t - dt) / 2)) < 0.05f);
        CHECK(std::fabs(ps.y - (v0.y * t + g.y * t * (t + dt) / 2)) < 0.05f);
        CHECK(std::fabs(pe.x - 100 * t) < 1e-3f);
    }
}

// ---- 粒子池 ----
static void testPoolSwapKill() {
    Pcg32 r(2);
    Pool pool;
    std::set<uint32_t> ref;
    uint32_t next = 0;
    for (int step = 0; step < 20000; ++step) {
        if (pool.size() == 0 || r.chance(0.55)) {
            pool.add({0, 0}, {0, 0}, 1, next), ref.insert(next), ++next;
        } else {
            size_t i = r.below(uint32_t(pool.size()));
            ref.erase(pool.id[i]);
            pool.kill(i);
        }
        if (step % 500 == 0) {
            std::set<uint32_t> got(pool.id.begin(), pool.id.end());
            CHECK(got == ref && got.size() == pool.size());
        }
    }
    // update：寿命到了的粒子被删除，其余推进
    pool.clear();
    for (uint32_t i = 0; i < 1000; ++i) pool.add({0, 0}, {10, 0}, 0.1f + 0.001f * float(i), i);
    pool.update(0.5f, {0, 0}, 0, Integrator::Exact);
    std::set<uint32_t> alive(pool.id.begin(), pool.id.end());
    for (uint32_t i = 0; i < 1000; ++i) CHECK(alive.count(i) == (0.1f + 0.001f * float(i) > 0.5f));
    for (size_t i = 0; i < pool.size(); ++i) CHECK(std::fabs(pool.pos[i].x - 5) < 1e-4f);
}

static void testRing() {
    Ring<int> ring(5);
    for (int i = 0; i < 3; ++i) ring.push(i);
    CHECK(ring.size() == 3 && ring[0] == 0 && ring[2] == 2);
    for (int i = 3; i < 8; ++i) ring.push(i);  // 满了就覆盖最老的
    CHECK(ring.size() == 5 && ring[0] == 3 && ring[4] == 7);
    ring.popOldest();
    CHECK(ring.size() == 4 && ring[0] == 4);
    ring.push(8);
    for (size_t i = 0; i < ring.size(); ++i) CHECK(ring[i] == int(4 + i));  // 遍历顺序 = 年龄顺序
}

// ---- 发射器 ----
static void testEmitterCounts() {
    for (EmitMode m : {EmitMode::Truncate, EmitMode::Round, EmitMode::Accumulate, EmitMode::Subframe}) {
        for (int fps : {30, 60, 144, 240}) {
            for (float rate : {50.0f, 100.0f, 1000.0f}) {
                Emitter e;
                e.rate = rate, e.mode = m;
                int total = 0;
                for (int f = 0; f < fps * 10; ++f) total += e.emit(1.0f / fps, {0, 0}, {0, 0}, [](Vec2, float) {});
                int expect = int(rate * 10);
                if (m == EmitMode::Accumulate || m == EmitMode::Subframe) CHECK(std::abs(total - expect) <= 1);
                if (m == EmitMode::Truncate && rate / fps < 1) CHECK(total == 0);  // 每帧不到一个：一个都发不出
                if (m == EmitMode::Round) CHECK(total == int(std::lround(rate / fps)) * fps * 10);  // 每帧四舍五入：60 fps 时 100/s 变成 120/s
            }
        }
    }
}
// 帧内出生时刻：粒子出生的时间间隔恰好是 1/rate，位置沿发射器的路径均匀分布
static void testSubframe() {
    for (int fps : {20, 60, 144}) {
        Emitter e;
        e.rate = 250, e.mode = EmitMode::Subframe;
        float dt = 1.0f / fps, t = 0;
        std::vector<float> births;
        std::vector<Vec2> where;
        for (int f = 0; f < fps * 2; ++f) {
            Vec2 from{t * 300, 0};
            t += dt;
            Vec2 to{t * 300, 0};
            e.emit(dt, from, to, [&](Vec2 p, float ageAtEnd) {
                births.push_back(t - ageAtEnd);
                where.push_back(p);
                CHECK(ageAtEnd >= -1e-6f && ageAtEnd < dt + 1e-6f);
            });
        }
        for (size_t i = 1; i < births.size(); ++i) {
            CHECK(std::fabs((births[i] - births[i - 1]) - 1 / 250.0f) < 2e-5f);
            CHECK(std::fabs(where[i].x - births[i] * 300) < 0.02f);  // 发射器在出生时刻的位置
        }
    }
}

// ---- 发射形状 ----
// 圆盘均匀：落在半径 [r_i, r_{i+1}] 圆环里的比例 = 面积比例；朴素写法挤在圆心
static void testDisk() {
    Pcg32 r(3);
    const int N = 200000, rings = 10;
    int good[rings] = {}, naive[rings] = {};
    for (int i = 0; i < N; ++i) {
        Vec2 a = uniformDisk(r, 1), b = naiveDisk(r, 1);
        CHECK(length(a) <= 1.0001f && length(b) <= 1.0001f);
        ++good[std::min(rings - 1, int(length(a) * rings))];
        ++naive[std::min(rings - 1, int(length(b) * rings))];
    }
    double chiGood = 0, chiNaive = 0;
    for (int k = 0; k < rings; ++k) {
        double expect = N * (double((k + 1) * (k + 1)) - double(k * k)) / (rings * rings);
        chiGood += (good[k] - expect) * (good[k] - expect) / expect;
        chiNaive += (naive[k] - expect) * (naive[k] - expect) / expect;
    }
    CHECK(chiGood < 30);       // 9 个自由度，p ≈ 0.0005
    CHECK(chiNaive > 10000);
    // 锥形：方向在范围内且两侧对称，速度在范围内
    int left = 0;
    for (int i = 0; i < 10000; ++i) {
        Vec2 v = cone(r, {0, -1}, 0.4f, 100, 200);
        float a = signedAngle({0, -1}, v);
        CHECK(std::fabs(a) <= 0.4001f);
        CHECK(length(v) >= 99.99f && length(v) <= 200.01f);
        left += a < 0;
    }
    CHECK(std::abs(left - 5000) < 250);
}

static void testGradient() {
    std::vector<Key> keys = {{0, 0}, {0.2f, 1}, {1, 0.5f}};
    CHECK(evaluate(keys, -1) == 0 && evaluate(keys, 2) == 0.5f);
    CHECK(std::fabs(evaluate(keys, 0.1f) - 0.5f) < 1e-6f);
    CHECK(std::fabs(evaluate(keys, 0.6f) - 0.75f) < 1e-6f);
    CHECK(evaluate(keys, 0.2f) == 1);
}

int main() {
    testExactIntegrator();
    testEulerErrorFormula();
    testPoolSwapKill();
    testRing();
    testEmitterCounts();
    testSubframe();
    testDisk();
    testGradient();
    return report("particles_test");
}
