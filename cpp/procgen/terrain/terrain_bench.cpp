// 用噪声生成地形：高度分布、细节与坡度/高度的相关、热侵蚀的方向性与收敛、水力侵蚀与河网、开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/terrain/terrain_bench.cpp -o terrain_bench && ./terrain_bench [section]
//   section：dist detail thermal hydro speed（缺省全部）
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include "terrain.h"

using namespace ga;
using namespace ga::noise;
using namespace ga::terrain;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
static const HashLattice kLat{2026};
static const char* kNames[] = {"fbm", "ridged", "damped", "warped"};

// 高度在 [0, 1] 左右；地图宽度对应 cells 个第 0 层格子
static Heightmap generate(int type, int n, float cells, int octaves = 8) {
    FbmParams p;
    p.octaves = octaves;
    auto base = [](float x, float y) { return perlinNoise(kLat, x, y); };
    auto baseD = [](float x, float y) { return perlinNoiseD(kLat, x, y); };
    auto f = [&](float x, float y) { return fbm(base, x, y, p); };
    Heightmap hm(n, n);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            float u = x * cells / n, v = y * cells / n, e = 0;
            if (type == 0) e = 0.5f + f(u, v);
            if (type == 1) e = ridged(base, u, v, p);
            if (type == 2) e = 0.5f + dampedFbm(baseD, u, v, p, 1.0f);
            if (type == 3) e = 0.5f + warped(f, u, v, 1.5f);
            hm.at(x, y) = e;
        }
    return hm;
}

static std::vector<float> boxBlur(const std::vector<float>& a, int n, int r) {
    std::vector<float> t(a.size()), o(a.size());
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            double s = 0;
            int c = 0;
            for (int k = std::max(0, x - r); k <= std::min(n - 1, x + r); ++k) s += a[y * n + k], ++c;
            t[y * n + x] = float(s / c);
        }
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            double s = 0;
            int c = 0;
            for (int k = std::max(0, y - r); k <= std::min(n - 1, y + r); ++k) s += t[k * n + x], ++c;
            o[y * n + x] = float(s / c);
        }
    return o;
}

static double pearson(const std::vector<float>& a, const std::vector<float>& b, const std::vector<char>& use) {
    double ma = 0, mb = 0, n = 0;
    for (size_t i = 0; i < a.size(); ++i)
        if (use[i]) ma += a[i], mb += b[i], n += 1;
    ma /= n, mb /= n;
    double sab = 0, saa = 0, sbb = 0;
    for (size_t i = 0; i < a.size(); ++i)
        if (use[i]) sab += (a[i] - ma) * (b[i] - mb), saa += (a[i] - ma) * (a[i] - ma), sbb += (b[i] - mb) * (b[i] - mb);
    return sab / std::sqrt(saa * sbb);
}

struct Moments {
    double mean, sd, skew, p05, p50, p95;
};
static Moments moments(std::vector<float> v) {
    double m = 0, s2 = 0, s3 = 0;
    for (float x : v) m += x;
    m /= v.size();
    for (float x : v) s2 += (x - m) * (x - m), s3 += (x - m) * (x - m) * (x - m);
    s2 /= v.size(), s3 /= v.size();
    std::sort(v.begin(), v.end());
    return {m, std::sqrt(s2), s3 / std::pow(s2, 1.5), v[v.size() / 20], v[v.size() / 2], v[v.size() * 19 / 20]};
}

static double meanSlope(const Heightmap& hm) {
    double s = 0;
    int c = 0;
    for (int y = 1; y < hm.h - 1; ++y)
        for (int x = 1; x < hm.w - 1; ++x)
            s += std::hypot(hm.at(x + 1, y) - hm.at(x - 1, y), hm.at(x, y + 1) - hm.at(x, y - 1)) * 0.5, ++c;
    return s / c;
}

int main(int argc, char** argv) {
    const int n = 512;

    // ---- 高度分布：均值、标准差、偏度、分位数（地图 512²，宽度 = 6 个格子，8 层） ----
    if (want(argc, argv, "dist")) {
        std::printf("== dist: height distribution (512^2, 6 cells, 8 octaves) ==\n");
        std::printf("%-8s %7s %7s %7s %7s %7s %7s\n", "type", "mean", "sd", "skew", "p05", "p50", "p95");
        for (int t = 0; t < 4; ++t) {
            Moments m = moments(generate(t, n, 6).v);
            std::printf("%-8s %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n", kNames[t], m.mean, m.sd, m.skew, m.p05, m.p50, m.p95);
        }
        // 重映射：先把 fbm 线性拉伸到 [0, 1]，再取幂次；看"低于 0.3 的面积"
        Heightmap f = generate(0, n, 6);
        float lo = *std::min_element(f.v.begin(), f.v.end()), hi = *std::max_element(f.v.begin(), f.v.end());
        std::printf("redistribution of fbm stretched to [0,1]: share below 0.3 / above 0.7\n");
        for (float ex : {0.5f, 1.0f, 1.5f, 2.0f, 3.0f}) {
            int below = 0, above = 0;
            for (float x : f.v) {
                float e = redistribute((x - lo) / (hi - lo), ex);
                below += e < 0.3f, above += e > 0.7f;
            }
            std::printf("  exponent %.1f: %5.1f%% / %5.1f%%\n", ex, 100.0 * below / f.v.size(), 100.0 * above / f.v.size());
        }
    }

    // ---- 细节与坡度、高度的相关 ----
    // 按层拆分：大尺度 L = 只取前 3 层，细节 D = 8 层 - 前 3 层（都换算成未归一化的和）。
    // 粗糙度 = 半径 8 的窗口内 D 的均方根；坡度 = |∇L|。统计时去掉边缘 32 像素
    if (want(argc, argv, "detail")) {
        std::printf("\n== detail: local roughness of octaves 3..7 vs slope / height of octaves 0..2 ==\n");
        std::printf("%-12s %12s %12s %16s %16s %10s\n", "type", "corr(slope)", "corr(height)", "rough q1/q5 slope", "rough q1/q5 hgt", "roughness");
        auto measure = [&](const char* name, auto&& gen) {
            std::vector<float> L = gen(3), H = gen(8), D2(L.size()), slope(L.size(), 0);
            for (size_t i = 0; i < D2.size(); ++i) D2[i] = (H[i] - L[i]) * (H[i] - L[i]);
            std::vector<float> rough = boxBlur(D2, n, 8);
            std::vector<char> use(L.size(), 0);
            double rm = 0;
            int c = 0;
            for (int y = 32; y < n - 32; ++y)
                for (int x = 32; x < n - 32; ++x) {
                    int i = y * n + x;
                    use[i] = 1, rough[i] = std::sqrt(rough[i]), rm += rough[i], ++c;
                    slope[i] = std::hypot(L[i + 1] - L[i - 1], L[i + n] - L[i - n]) * 0.5f;
                }
            // 按坡度（或高度）分成 5 档，比较最低档与最高档的平均粗糙度
            auto quintRatio = [&](const std::vector<float>& key) {
                std::vector<std::pair<float, float>> kv;
                for (size_t i = 0; i < key.size(); ++i)
                    if (use[i]) kv.push_back({key[i], rough[i]});
                std::sort(kv.begin(), kv.end());
                size_t q = kv.size() / 5;
                double a = 0, b = 0;
                for (size_t i = 0; i < q; ++i) a += kv[i].second, b += kv[kv.size() - 1 - i].second;
                return a / b;
            };
            std::printf("%-12s %12.3f %12.3f %16.2f %16.2f %10.4f\n", name, pearson(slope, rough, use), pearson(L, rough, use), quintRatio(slope),
                        quintRatio(L), rm / c);
        };
        auto mapOf = [&](auto&& f) {  // f(x, y, octaves) 返回未归一化的和
            return [&, f](int oct) {
                std::vector<float> v(size_t(n) * n);
                for (int y = 0; y < n; ++y)
                    for (int x = 0; x < n; ++x) v[size_t(y) * n + x] = f(x * 6.0f / n, y * 6.0f / n, oct);
                return v;
            };
        };
        auto norm = [](int oct) { return float(2 - std::pow(0.5, oct - 1)); };
        auto base = [](float x, float y) { return perlinNoise(kLat, x, y); };
        auto baseD = [](float x, float y) { return perlinNoiseD(kLat, x, y); };
        auto params = [](int oct) {
            FbmParams p;
            p.octaves = oct;
            return p;
        };
        measure("fbm", mapOf([&](float x, float y, int o) { return fbm(base, x, y, params(o)) * norm(o); }));
        measure("ridged", mapOf([&](float x, float y, int o) { return ridged(base, x, y, params(o)) * norm(o); }));
        for (float k : {0.25f, 1.0f, 4.0f}) {
            char nm[32];
            std::snprintf(nm, sizeof nm, "damped k=%g", k);
            measure(nm, mapOf([&, k](float x, float y, int o) { return dampedFbm(baseD, x, y, params(o), k) * norm(o); }));
        }
        // 域扭曲：扭曲场固定用 8 层 fBm，只拆分被采样的那个
        measure("warped", mapOf([&](float x, float y, int o) {
                    auto f8 = [&](float u, float v) { return fbm(base, u, v, params(8)); };
                    float qx = f8(x, y), qy = f8(x + 5.2f, y + 1.3f);
                    return fbm(base, x + 1.5f * qx, y + 1.5f * qy, params(o)) * norm(o);
                }));
    }

    // ---- 热侵蚀 ----
    if (want(argc, argv, "thermal")) {
        std::printf("\n== thermal: sand pile shape (talus 1, 10000 units dropped on one cell) ==\n");
        const char* nbName[] = {"4", "8", "8 scaled"};
        for (int nb = 0; nb < 3; ++nb) {
            const int m = 101, c = 50;
            Heightmap hm(m, m);
            hm.at(c, c) = 10000;
            std::vector<float> delta;
            int it = 0;
            while (it < 20000 && thermalStep(hm, delta, 1.0f, Neighborhood(nb)) > 1e-3) ++it;
            float axis = 0, diag = 0;
            for (int r = 1; r < c; ++r) {
                if (hm.at(c + r, c) > 0.5f) axis = float(r);
                if (hm.at(c + r, c + r) > 0.5f) diag = r * 1.41421356f;
            }
            // 每个方向（每 5°）上沙堆的半径：沿射线找高度首次低于 0.5 的位置
            double rmin = 1e9, rmax = 0;
            for (int a = 0; a < 72; ++a) {
                double th = a * 5 * 3.14159265358979 / 180, r = 0;
                for (; r < c - 1; r += 0.05) {
                    float s = heightAndGradient(hm, float(c + r * std::cos(th)), float(c + r * std::sin(th))).value;
                    if (s < 0.5f) break;
                }
                rmin = std::min(rmin, r), rmax = std::max(rmax, r);
            }
            std::printf("  %-9s iterations %5d  peak %6.1f  radius axis %4.1f diag %4.1f  diag/axis %.3f  max/min over angles %.3f\n", nbName[nb],
                        it, hm.at(c, c), axis, diag, diag / axis, rmax / rmin);
        }
        std::printf("thermal on fbm (256^2, 6 octaves, heights x 64 so 1 unit = 1 cell, talus 0.6): slope distribution\n");
        for (int nb = 0; nb < 3; ++nb) {
            Heightmap hm = generate(0, 256, 3, 6);
            for (float& v : hm.v) v *= 64;
            double before = hm.sum();
            std::vector<float> delta;
            auto t0 = Clock::now();
            int it = 0;
            for (; it < 500; ++it) thermalStep(hm, delta, 0.6f, Neighborhood(nb));
            double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / it;
            // 坡度按方向统计：沿轴（x 方向差分）与沿对角
            double axisMax = 0, diagMax = 0;
            for (int y = 1; y < 255; ++y)
                for (int x = 1; x < 255; ++x) {
                    axisMax = std::max(axisMax, double(std::abs(hm.at(x + 1, y) - hm.at(x, y))));
                    diagMax = std::max(diagMax, std::abs(hm.at(x + 1, y + 1) - hm.at(x, y)) / 1.41421356);
                }
            std::printf("  %-9s %.3f ms/iter  max slope axis %.3f  diagonal %.3f  mass drift %.2e\n", nbName[nb], ms, axisMax, diagMax,
                        (hm.sum() - before) / before);
        }
    }

    // ---- 水力侵蚀与河网 ----
    if (want(argc, argv, "hydro")) {
        std::printf("\n== hydro: droplet erosion on 256^2 fbm (3 cells, 6 octaves, heights 0..1) ==\n");
        std::printf("%-22s %7s %8s %9s %9s %9s %8s %8s %8s\n", "", "pits", "drained", "max area", "slope", "eroded", "deposit", "lost", "us/drop");
        auto row = [&](const char* name, const Heightmap& hm, const DropletStats* st, double us) {
            FlowResult fr = flow(hm);
            float amax = *std::max_element(fr.area.begin(), fr.area.end());
            std::printf("%-22s %7d %7.1f%% %9.0f %9.5f", name, fr.pits, fr.drainedOut * 100, amax, meanSlope(hm));
            if (st) std::printf(" %9.2f %8.2f %8.2f %8.2f", st->eroded, st->deposited, st->carriedOff, us);
            std::printf("\n");
        };
        auto runDrops = [&](Heightmap& hm, int drops, const DropletParams& p, DropletStats& st) {
            Pcg32 rng(7);
            auto t0 = Clock::now();
            for (int k = 0; k < drops; ++k) simulateDroplet(hm, float(rng.nextDouble()) * 254.999f, float(rng.nextDouble()) * 254.999f, p, st);
            return std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / drops;
        };
        for (int t : {0, 2}) {
            Heightmap base = generate(t, 256, 3, 6);
            char nm[64];
            std::snprintf(nm, sizeof nm, "%s raw", kNames[t]);
            row(nm, base, nullptr, 0);
            for (bool limit : {true, false})
                for (double perCell : {0.25, 1.0, 4.0, 16.0}) {
                    if (!limit && (t != 0 || perCell > 4)) continue;
                    Heightmap hm = base;
                    DropletParams p;
                    p.floorAtNext = limit;
                    DropletStats st;
                    double us = runDrops(hm, int(perCell * 256 * 256), p, st);
                    std::snprintf(nm, sizeof nm, "  %5.2f drops/cell%s", perCell, limit ? "" : " *");
                    row(nm, hm, &st, us);
                    float lo = *std::min_element(hm.v.begin(), hm.v.end()), hi = *std::max_element(hm.v.begin(), hm.v.end());
                    std::printf("      height range %.3f .. %.3f, avg steps/drop %.1f\n", lo, hi, double(st.steps) / (perCell * 65536));
                }
        }
        std::printf("  (* = without the floor at the next position)\n");
        // 冲刷半径
        std::printf("radius (fbm, 1 drop/cell):\n");
        for (int r : {1, 2, 3, 5}) {
            Heightmap hm = generate(0, 256, 3, 6);
            DropletParams p;
            p.radius = r;
            DropletStats st;
            Pcg32 rng(7);
            auto t0 = Clock::now();
            for (int k = 0; k < 65536; ++k) simulateDroplet(hm, float(rng.nextDouble()) * 254.999f, float(rng.nextDouble()) * 254.999f, p, st);
            double us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / 65536;
            char nm[32];
            std::snprintf(nm, sizeof nm, "  radius %d", r);
            row(nm, hm, &st, us);
        }
        // 惯性
        std::printf("inertia (fbm, 1 drop/cell):\n");
        for (float in : {0.0f, 0.05f, 0.3f, 0.7f}) {
            Heightmap hm = generate(0, 256, 3, 6);
            DropletParams p;
            p.inertia = in;
            DropletStats st;
            Pcg32 rng(7);
            for (int k = 0; k < 65536; ++k) simulateDroplet(hm, float(rng.nextDouble()) * 254.999f, float(rng.nextDouble()) * 254.999f, p, st);
            char nm[32];
            std::snprintf(nm, sizeof nm, "  inertia %.2f", in);
            row(nm, hm, &st, 0);
        }
    }

    // ---- 开销 ----
    if (want(argc, argv, "speed")) {
        std::printf("\n== speed: generating 256^2, 8 octaves (min of 5 rounds) ==\n");
        for (int t = 0; t < 4; ++t) {
            double best = 1e9;
            for (int r = 0; r < 5; ++r) {
                auto t0 = Clock::now();
                Heightmap hm = generate(t, 256, 3, 6);
                double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                best = std::min(best, ms);
                if (hm.v[123] == 42) std::printf("!");
            }
            std::printf("  %-8s %.2f ms  (%.0f ns/sample)\n", kNames[t], best, best * 1e6 / 65536);
        }
        Heightmap hm = generate(0, 256, 3, 6);
        double best = 1e9;
        for (int r = 0; r < 5; ++r) {
            auto t0 = Clock::now();
            FlowResult fr = flow(hm);
            best = std::min(best, std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
            if (fr.pits == -1) std::printf("!");
        }
        std::printf("  flow (D8 + sort) %.2f ms\n", best);
    }
}
