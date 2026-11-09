#include "terrain.h"

#include <cstdio>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::noise;
using namespace ga::terrain;

static Heightmap fbmMap(int w, int h, uint32_t seed, float cells = 4) {
    HashLattice lat{seed};
    FbmParams p;
    p.octaves = 5;
    Heightmap hm(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            hm.at(x, y) = 0.5f + 0.5f * fbm([&](float u, float v) { return perlinNoise(lat, u, v); }, x * cells / w, y * cells / h, p);
    return hm;
}

// ---- 热侵蚀的参照写法："拉"而不是"推"：每个格子查看每个邻居会分给自己多少，用 double 计算 ----
static std::vector<double> thermalPull(const Heightmap& hm, float talus, Neighborhood nb, float rate) {
    const int nn = neighborCount(nb);
    auto outflow = [&](int x, int y, int k) -> double {  // 格子 (x, y) 分给第 k 个邻居的量
        double ex[8], total = 0, emax = 0;
        for (int j = 0; j < nn; ++j) {
            int qx = x + kDx[j], qy = y + kDy[j];
            ex[j] = 0;
            if (!hm.inside(qx, qy)) continue;
            double e = double(hm.at(x, y)) - hm.at(qx, qy) - double(talus) * neighborDist(nb, j);
            if (e > 0) ex[j] = e, total += e, emax = std::max(emax, e);
        }
        return total > 0 ? rate * emax * ex[k] / total : 0.0;
    };
    std::vector<double> out(hm.v.size());
    for (int y = 0; y < hm.h; ++y)
        for (int x = 0; x < hm.w; ++x) {
            double v = hm.at(x, y);
            for (int k = 0; k < nn; ++k) {
                int qx = x + kDx[k], qy = y + kDy[k];
                if (!hm.inside(qx, qy)) continue;
                v -= outflow(x, y, k);
                // 邻居 q 往回指向自己的方向：kDx/kDy 中与 k 相反的那个
                for (int j = 0; j < nn; ++j)
                    if (kDx[j] == -kDx[k] && kDy[j] == -kDy[k]) v += outflow(qx, qy, j);
            }
            out[size_t(y) * hm.w + x] = v;
        }
    return out;
}

static void testThermal() {
    const Neighborhood all[3] = {Neighborhood::Four, Neighborhood::Eight, Neighborhood::EightScaled};
    for (Neighborhood nb : all) {
        Heightmap hm = fbmMap(48, 40, 3);
        for (float& v : hm.v) v *= 30;  // 放大到坡度明显超过休止角
        std::vector<float> delta;
        // 与参照写法逐格比较（多轮，每轮都以当前结果为起点）
        double worst = 0;
        for (int it = 0; it < 20; ++it) {
            std::vector<double> ref = thermalPull(hm, 0.6f, nb, 0.5f);
            thermalStep(hm, delta, 0.6f, nb);
            for (size_t i = 0; i < ref.size(); ++i) worst = std::max(worst, std::abs(ref[i] - hm.v[i]));
        }
        CHECK(worst < 1e-4);  // 高度在 0..30，float 的舍入

        // 质量守恒
        Heightmap g = fbmMap(64, 64, 5);
        for (float& v : g.v) v *= 40;
        double before = g.sum();
        for (int it = 0; it < 300; ++it) thermalStep(g, delta, 0.5f, nb);
        CHECK(std::abs(g.sum() - before) / before < 1e-5);
        // 收敛：足够多轮后，几乎没有超过休止角的坡
        for (int it = 0; it < 3000; ++it) thermalStep(g, delta, 0.5f, nb);
        float maxExcess = 0;
        for (int y = 0; y < g.h; ++y)
            for (int x = 0; x < g.w; ++x)
                for (int k = 0; k < neighborCount(nb); ++k) {
                    int qx = x + kDx[k], qy = y + kDy[k];
                    if (g.inside(qx, qy)) maxExcess = std::max(maxExcess, g.at(x, y) - g.at(qx, qy) - 0.5f * neighborDist(nb, k));
                }
        CHECK(maxExcess < 0.01f);
        // 已经稳定的地形（处处坡度 0.15 / 格）不再变化
        Heightmap stable(20, 20);
        for (int y = 0; y < 20; ++y)
            for (int x = 0; x < 20; ++x) stable.at(x, y) = 0.15f * (x + y);
        Heightmap s2 = stable;
        CHECK(thermalStep(s2, delta, 0.5f, nb) == 0);
        CHECK(s2.v == stable.v);
        // 转置对称：结果与遍历顺序无关，所以转置输入得到转置输出
        Heightmap a = fbmMap(32, 32, 7), t(32, 32);
        for (float& v : a.v) v *= 20;
        for (int y = 0; y < 32; ++y)
            for (int x = 0; x < 32; ++x) t.at(y, x) = a.at(x, y);
        for (int it = 0; it < 50; ++it) thermalStep(a, delta, 0.5f, nb), thermalStep(t, delta, 0.5f, nb);
        float asym = 0;
        for (int y = 0; y < 32; ++y)
            for (int x = 0; x < 32; ++x) asym = std::max(asym, std::abs(t.at(y, x) - a.at(x, y)));
        CHECK(asym < 1e-4f);
    }
    // 一根柱子塌成的沙堆：4 邻域是菱形（对角方向短），不加权的 8 邻域是正方形（对角方向长）
    auto pile = [](Neighborhood nb, float& axis, float& diag) {
        const int n = 101, c = 50;
        Heightmap hm(n, n);
        hm.at(c, c) = 10000;
        std::vector<float> delta;
        for (int it = 0; it < 20000 && thermalStep(hm, delta, 1.0f, nb) > 1e-3; ++it) {}
        axis = diag = 0;
        for (int r = 1; r < c; ++r) {
            if (hm.at(c + r, c) > 0.5f) axis = float(r);
            if (hm.at(c + r, c + r) > 0.5f) diag = r * 1.41421356f;
        }
    };
    float a4, d4, a8, d8, as, ds;
    pile(Neighborhood::Four, a4, d4);
    pile(Neighborhood::Eight, a8, d8);
    pile(Neighborhood::EightScaled, as, ds);
    CHECK(d4 < a4 * 0.8f);   // 实测 0.74（菱形是 0.71）
    CHECK(d8 > a8 * 1.25f);  // 实测 1.34（正方形是 1.41）
    CHECK(std::abs(ds / as - 1) < std::abs(d4 / a4 - 1));
    CHECK(std::abs(ds / as - 1) < std::abs(d8 / a8 - 1));
}

static void testSample() {
    Heightmap hm = fbmMap(20, 20, 9);
    for (int y = 0; y < 19; ++y)
        for (int x = 0; x < 19; ++x) CHECK(heightAndGradient(hm, float(x), float(y)).value == hm.at(x, y));
    Pcg32 rng(2);
    double worst = 0;
    for (int k = 0; k < 2000; ++k) {
        float x = 0.01f + float(rng.nextDouble()) * 18.98f, y = 0.01f + float(rng.nextDouble()) * 18.98f;
        // 在同一个格子内做差分（双线性插值跨格子时梯度不连续）；不取格子中心，那里 u = v = 0.5，权重写反也看不出来
        float cx = std::clamp(x, std::floor(x) + 0.01f, std::floor(x) + 0.99f);
        float cy = std::clamp(y, std::floor(y) + 0.01f, std::floor(y) + 0.99f), e = 1e-3f;
        Sample<float> s = heightAndGradient(hm, cx, cy);
        double gx = (heightAndGradient(hm, cx + e, cy).value - heightAndGradient(hm, cx - e, cy).value) / (2 * e);
        double gy = (heightAndGradient(hm, cx, cy + e).value - heightAndGradient(hm, cx, cy - e).value) / (2 * e);
        worst = std::max({worst, std::abs(gx - s.dx), std::abs(gy - s.dy)});
    }
    CHECK(worst < 2e-3);
}

static void testDroplet() {
    DropletParams p;
    // 平地：什么都不发生
    Heightmap flat(32, 32, 0.25f);
    DropletStats st;
    Pcg32 rng(3);
    for (int k = 0; k < 200; ++k) simulateDroplet(flat, float(rng.nextDouble()) * 30, float(rng.nextDouble()) * 30, p, st);
    for (float v : flat.v) CHECK(v == 0.25f);
    CHECK(st.eroded == 0 && st.deposited == 0);

    // 斜面：沿下坡方向走，从下游边界流出
    Heightmap ramp(64, 16);
    for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 64; ++x) ramp.at(x, y) = 1 - x / 64.0f;
    DropletStats rs;
    simulateDroplet(ramp, 2.5f, 7.5f, p, rs);
    CHECK(rs.steps > 55 && rs.steps < 64);  // 每步走 1 格，约 61 步后从 x = 63 流出
    CHECK(rs.eroded > 0);

    // 质量守恒：地面的变化 = 放下的 - 取走的；取走的 - 放下的 = 水滴带走的
    Heightmap hm = fbmMap(96, 96, 11);
    double before = hm.sum();
    DropletStats ds;
    for (int k = 0; k < 20000; ++k) simulateDroplet(hm, float(rng.nextDouble()) * 95, float(rng.nextDouble()) * 95, p, ds);
    CHECK(std::abs(hm.sum() - (before - ds.eroded + ds.deposited)) < 1e-6 * before + 1e-2);
    CHECK(std::abs(ds.eroded - ds.deposited - ds.carriedOff) < 1e-6 * ds.eroded + 1e-3);
    CHECK(ds.eroded > 1 && ds.deposited > 0.5);
    // 冲刷让地面整体变低（有泥沙被带出地图）但不会出现 NaN
    for (float v : hm.v) CHECK(std::isfinite(v));
    // 最低点永远不会更低：冲刷不低于下一步的高度（它不低于当前的最低点），沉积只会抬高。
    // 关掉这条限制，同样的水滴数会把地图边缘挖出无底的沟
    for (bool limit : {true, false}) {
        Heightmap g = fbmMap(64, 64, 17);
        float lo = *std::min_element(g.v.begin(), g.v.end());
        DropletParams q;
        q.floorAtNext = limit;
        DropletStats gs;
        for (int k = 0; k < 64 * 64 * 16; ++k) simulateDroplet(g, float(rng.nextDouble()) * 62.999f, float(rng.nextDouble()) * 62.999f, q, gs);
        float lo2 = *std::min_element(g.v.begin(), g.v.end());
        if (limit) CHECK(lo2 >= lo);
        else CHECK(lo2 < lo - 1);
    }
    // 同样的种子得到同样的结果
    Heightmap h1 = fbmMap(40, 40, 4), h2 = h1;
    Pcg32 r1(9), r2(9);
    DropletStats s1, s2;
    for (int k = 0; k < 500; ++k) {
        simulateDroplet(h1, float(r1.nextDouble()) * 39, float(r1.nextDouble()) * 39, p, s1);
        simulateDroplet(h2, float(r2.nextDouble()) * 39, float(r2.nextDouble()) * 39, p, s2);
    }
    CHECK(h1.v == h2.v);
}

static void testFlow() {
    // 参照：从每个格子沿 down 走到底，途经的格子面积各加 1
    Heightmap hm = fbmMap(50, 40, 13);
    FlowResult r = flow(hm);
    std::vector<float> area(hm.v.size(), 0);
    for (size_t i = 0; i < hm.v.size(); ++i)
        for (int c = int(i); c >= 0; c = r.down[c]) area[c] += 1;
    CHECK(area == r.area);
    // 下游一定更低（严格），所以不会成环
    for (size_t i = 0; i < hm.v.size(); ++i)
        if (r.down[i] >= 0) CHECK(hm.v[r.down[i]] < hm.v[i]);
    // 洼地：参照写法逐格判断"8 个邻居都不比它低"
    int pits = 0;
    for (int y = 1; y < hm.h - 1; ++y)
        for (int x = 1; x < hm.w - 1; ++x) {
            bool low = true;
            for (int k = 0; k < 8; ++k) low &= hm.at(x + kDx[k], y + kDy[k]) >= hm.at(x, y);
            pits += low;
        }
    CHECK(pits == r.pits);
    // 完全平坦：没有下坡邻居，内部格子都是洼地（等高的邻居不算下坡，否则会成环）
    Heightmap flatMap(12, 10, 0.5f);
    FlowResult rf = flow(flatMap);
    CHECK(rf.pits == 10 * 8);
    for (int d : rf.down) CHECK(d == -1);
    // 对角方向的坡度要除以 √2：轴向下降 1.0（坡度 1）比对角下降 1.3（坡度 0.92）更陡
    Heightmap diag(3, 3, 5.0f);
    diag.at(1, 1) = 2, diag.at(2, 1) = 1.0f, diag.at(2, 2) = 0.7f;
    CHECK(flow(diag).down[4] == 1 * 3 + 2);
    // 向一侧倾斜的平面：全部流出，没有洼地
    Heightmap ramp(30, 20);
    for (int y = 0; y < 20; ++y)
        for (int x = 0; x < 30; ++x) ramp.at(x, y) = float(x) + 0.01f * std::abs(y - 10);
    FlowResult rr = flow(ramp);
    CHECK(rr.pits == 0);
    CHECK(std::abs(rr.drainedOut - 1) < 1e-12);
    // 中间挖一个坑：坑周围的水流进坑里
    ramp.at(15, 10) = -5;
    FlowResult rp = flow(ramp);
    CHECK(rp.pits == 1);
    CHECK(rp.drainedOut < 1);
    CHECK(rp.area[10 * 30 + 15] > 1);
}

static void testShapes() {
    HashLattice lat{21};
    FbmParams p;
    auto base = [&](double x, double y) { return perlinNoise(lat, x, y); };
    auto baseD = [&](double x, double y) { return perlinNoiseD(lat, x, y); };
    Pcg32 rng(5);
    double worst = 0, worstK0 = 0;
    for (int k = 0; k < 3000; ++k) {
        double x = rng.nextDouble() * 40, y = rng.nextDouble() * 40;
        // k = 0：就是普通 fBm
        worstK0 = std::max(worstK0, std::abs(dampedFbm(baseD, x, y, p, 0.0) - fbm(base, x, y, p)));
        // 参照：梯度用对部分和的中心差分求
        double sum = 0, norm = 0, amp = 1, e = 1e-6;
        auto partial = [&](double u, double v, int upto) {
            double s = 0, a = 1, f = 1;
            for (int o = 0; o <= upto; ++o) {
                double ox, oy;
                octaveOffset(o, ox, oy);
                s += a * base(u * f + ox, v * f + oy);
                a *= p.gain, f *= p.lacunarity;
            }
            return s;
        };
        double freq = 1;
        for (int o = 0; o < p.octaves; ++o) {
            double ox, oy;
            octaveOffset(o, ox, oy);
            double gx = (partial(x + e, y, o) - partial(x - e, y, o)) / (2 * e);
            double gy = (partial(x, y + e, o) - partial(x, y - e, o)) / (2 * e);
            sum += amp * base(x * freq + ox, y * freq + oy) / (1 + 1.0 * (gx * gx + gy * gy));
            norm += amp, amp *= p.gain, freq *= p.lacunarity;
        }
        worst = std::max(worst, std::abs(sum / norm - dampedFbm(baseD, x, y, p, 1.0)));
    }
    CHECK(worstK0 < 1e-12);
    CHECK(worst < 1e-6);

    // 域扭曲：与手写的两步一致；强度 0 就是原函数
    for (int k = 0; k < 100; ++k) {
        double x = rng.nextDouble() * 40, y = rng.nextDouble() * 40, s = 1.7;
        double qx = base(x, y), qy = base(x + 5.2, y + 1.3);
        CHECK(std::abs(warped(base, x, y, s) - base(x + s * qx, y + s * qy)) < 1e-12);
    }
    for (int k = 0; k < 100; ++k) {
        double x = rng.nextDouble() * 40, y = rng.nextDouble() * 40;
        CHECK(warped(base, x, y, 0.0) == base(x, y));
    }
    // 重映射：端点不变，单调
    CHECK(redistribute(0, 2.3f) == 0 && redistribute(1, 2.3f) == 1);
    for (int i = 0; i < 100; ++i) CHECK(redistribute(i / 100.0f, 2.3f) < redistribute((i + 1) / 100.0f, 2.3f));
    // 岛屿：mix = 0 不变；mix = 1 时边缘为 0、中心为 1
    CHECK(island(0.37f, 0.5f, 0.2f, 0) == 0.37f);
    CHECK(island(0.8f, 1, 0, 1) == 0 && island(0.8f, 0.8f, 0.8f, 1) == 0 && island(0.1f, 0, 0, 1) == 1);
}

int main() {
    testThermal();
    testSample();
    testDroplet();
    testFlow();
    testShapes();
    return report("terrain_test");
}
