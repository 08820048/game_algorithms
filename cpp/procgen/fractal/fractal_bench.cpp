// Simplex、Worley 与 fBm：分布、方向性、影响半径、fBm 频谱、整数点上的零、3×3 搜索的错误率、开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/fractal/fractal_bench.cpp -o fractal_bench && ./fractal_bench [section]
//   section：stats iso kernel spectrum ridge zeros worley speed（缺省全部）
#include <chrono>
#include <complex>
#include <cstdio>
#include <cstring>
#include <vector>

#include "fractal.h"

using namespace ga;
using namespace ga::noise;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
static const HashLattice kLat{2026};
static const double kPi = 3.14159265358979323846;

static void fft(std::vector<std::complex<double>>& a) {
    size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        std::complex<double> w = std::polar(1.0, -2 * kPi / double(len));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> cur = 1;
            for (size_t k = 0; k < len / 2; ++k, cur *= w) {
                auto u = a[i + k], v = a[i + k + len / 2] * cur;
                a[i + k] = u + v, a[i + k + len / 2] = u - v;
            }
        }
    }
}

// 二维径向功率谱密度：N×N 块，Hann 窗，返回按整数半径 r（单位 1/N 周期每像素）的平均功率
template <class F>
static std::vector<double> radialPsd(F&& f, int N, int patches, Pcg32& rng) {
    std::vector<double> psd(N / 2, 0), cnt(N / 2, 0);
    for (int p = 0; p < patches; ++p) {
        double x0 = rng.nextDouble() * 1e4, y0 = rng.nextDouble() * 1e4;
        std::vector<std::vector<std::complex<double>>> a(N, std::vector<std::complex<double>>(N));
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                double w = (0.5 - 0.5 * std::cos(2 * kPi * i / N)) * (0.5 - 0.5 * std::cos(2 * kPi * j / N));
                a[j][i] = f(x0 + i, y0 + j) * w;
            }
        for (auto& row : a) fft(row);
        for (int i = 0; i < N; ++i) {
            std::vector<std::complex<double>> col(N);
            for (int j = 0; j < N; ++j) col[j] = a[j][i];
            fft(col);
            for (int j = 0; j < N; ++j) a[j][i] = col[j];
        }
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                int fi = i <= N / 2 ? i : i - N, fj = j <= N / 2 ? j : j - N;
                int r = int(std::lround(std::hypot(fi, fj)));
                if (r < N / 2) psd[r] += std::norm(a[j][i]), cnt[r] += 1;
            }
    }
    for (int r = 0; r < N / 2; ++r) psd[r] /= std::max(1.0, cnt[r]);
    return psd;
}

int main(int argc, char** argv) {
    Pcg32 rng(1);
    auto perlin = [](double x, double y) { return perlinNoise(kLat, x, y); };
    auto simplex = [](double x, double y) { return simplexNoise(kLat, x, y); };
    auto value = [](double x, double y) { return valueNoise(kLat, x, y); };

    if (want(argc, argv, "stats")) {
        std::printf("== distribution over 2M random points ==\n%-14s %8s %8s %8s %8s %8s %8s\n", "noise", "mean", "std", "p1", "p99", "min", "max");
        struct K {
            const char* name;
            double (*fn)(double, double);
        } ks[] = {{"perlin", [](double x, double y) { return perlinNoise(kLat, x, y); }},
                  {"simplex", [](double x, double y) { return simplexNoise(kLat, x, y); }},
                  {"worley F1", [](double x, double y) { return worley(kLat, x, y).f1; }},
                  {"worley F2", [](double x, double y) { return worley(kLat, x, y).f2; }},
                  {"worley F2-F1", [](double x, double y) {
                       auto c = worley(kLat, x, y);
                       return c.f2 - c.f1;
                   }}};
        for (const K& k : ks) {
            std::vector<double> v(2000000);
            double s = 0, s2 = 0;
            for (double& x : v) x = k.fn(rng.nextDouble() * 1e4, rng.nextDouble() * 1e4), s += x, s2 += x * x;
            std::sort(v.begin(), v.end());
            double n = double(v.size()), m = s / n;
            std::printf("%-14s %8.4f %8.4f %8.3f %8.3f %8.3f %8.3f\n", k.name, m, std::sqrt(s2 / n - m * m), v[v.size() / 100], v[v.size() * 99 / 100], v.front(),
                        v.back());
        }
    }

    if (want(argc, argv, "iso")) {
        // 梯度方向偏差：E[|g| cos kθ] / E[|g|]。正方形网格的偏差体现在 cos 4θ，三角形网格体现在 cos 6θ
        std::printf("\n== gradient direction bias (0 = isotropic) ==\n%-10s %10s %10s\n", "noise", "cos 4θ", "cos 6θ");
        for (int which = 0; which < 3; ++which) {
            double c4 = 0, c6 = 0, den = 0;
            for (int i = 0; i < 2000000; ++i) {
                double x = rng.nextDouble() * 1e4, y = rng.nextDouble() * 1e4;
                Sample<double> s = which == 0 ? valueNoiseD(kLat, x, y) : which == 1 ? perlinNoiseD(kLat, x, y) : simplexNoiseD(kLat, x, y);
                double m = std::hypot(s.dx, s.dy), th = std::atan2(s.dy, s.dx);
                c4 += m * std::cos(4 * th), c6 += m * std::cos(6 * th), den += m;
            }
            std::printf("%-10s %+10.3f %+10.3f\n", which == 0 ? "value" : which == 1 ? "perlin" : "simplex", c4 / den, c6 / den);
        }
        // 相关系数：沿轴 vs 沿对角，以及沿 0° vs 30°（三角形网格的两个特殊方向）
        std::printf("\ncorrelation at distance r=1 along different directions:\n%-10s %8s %8s %8s %8s\n", "noise", "0deg", "30deg", "45deg", "max diff");
        for (int which = 1; which < 3; ++which) {
            double c[3] = {}, s0 = 0;
            const double ang[3] = {0, kPi / 6, kPi / 4};
            for (int i = 0; i < 2000000; ++i) {
                double x = rng.nextDouble() * 1e4, y = rng.nextDouble() * 1e4;
                auto f = [&](double u, double v) { return which == 1 ? perlinNoise(kLat, u, v) : simplexNoise(kLat, u, v); };
                double v0 = f(x, y);
                s0 += v0 * v0;
                for (int a = 0; a < 3; ++a) c[a] += v0 * (f(x + std::cos(ang[a]), y + std::sin(ang[a])) + f(x + std::sin(ang[a]), y + std::cos(ang[a]))) / 2;
            }
            double mx = std::max({c[0], c[1], c[2]}) / s0, mn = std::min({c[0], c[1], c[2]}) / s0;
            std::printf("%-10s %8.4f %8.4f %8.4f %8.4f\n", which == 1 ? "perlin" : "simplex", c[0] / s0, c[1] / s0, c[2] / s0, mx - mn);
        }
    }

    if (want(argc, argv, "kernel")) {
        // 影响半径的平方取 0.6（网上一些二维实现的取值）：只算 3 个角时会漏掉三角形外的顶点，跨过三角形的边时值会跳变
        std::printf("\n== simplex kernel radius: jump across triangle edges (relative to the noise std) ==\n");
        for (double r2 : {0.5, 0.55, 0.6}) {
            double s2 = 0, maxJump = 0, sumJump = 0;
            int big = 0;
            const int n = 200000;
            for (int k = 0; k < n; ++k) {
                // 在斜切坐标里取对角边 u = v 上的点，比较两侧
                double base = rng.below(5000), w = rng.nextDouble();
                double u = base + w, v = rng.below(5000) + w;
                double e = 1e-9;
                double ta = (u + e + v) * kUnskew, tb = (u + v + e) * kUnskew;
                double a = simplexNoiseD(kLat, u + e - ta, v - ta, r2).value, b = simplexNoiseD(kLat, u - tb, v + e - tb, r2).value;
                double j = std::abs(a - b);
                maxJump = std::max(maxJump, j), sumJump += j, big += j > 1e-6;
                s2 += a * a;
            }
            double sd = std::sqrt(s2 / n);
            // 梯度的跳变，相对平均梯度大小
            double gsum = 0, gmax = 0, gmean = 0;
            for (int k = 0; k < 50000; ++k) {
                double w = rng.nextDouble(), u = rng.below(5000) + w, v = rng.below(5000) + w, e = 1e-9;
                double ta = (u + e + v) * kUnskew, tb = (u + v + e) * kUnskew;
                auto a = simplexNoiseD(kLat, u + e - ta, v - ta, r2), b = simplexNoiseD(kLat, u - tb, v + e - tb, r2);
                double j = std::hypot(a.dx - b.dx, a.dy - b.dy);
                gsum += j, gmax = std::max(gmax, j), gmean += std::hypot(a.dx, a.dy);
            }
            std::printf("            gradient: mean jump %.3f, max jump %.3f (relative to mean |gradient|)\n", gsum / gmean, gmax / (gmean / 50000));
            std::printf("r2 = %.2f   std %.4f   edges with a jump %5.1f%%   mean jump %.3f std   max jump %.3f std\n", r2, sd, 100.0 * big / n, sumJump / n / sd,
                        maxJump / sd);
        }
    }

    if (want(argc, argv, "spectrum")) {
        // fBm 的功率谱应当是一条直线：log PSD = -β log f + c，β = 2 - 2 log2(gain)（频率倍率为 2 时）
        std::printf("\n== fBm radial power spectrum: fitted slope vs theory, and how bumpy it is (RMS of log10 residual) ==\n");
        std::printf("%-8s %6s %10s %10s %14s\n", "base", "gain", "slope", "theory", "bumpiness");
        const int N = 256;
        for (double gain : {0.35, 0.5, 0.7})
            for (int which = 0; which < 3; ++which) {
                FbmParams p;
                p.octaves = 6, p.gain = gain;
                const double scale = 1.0 / 64;  // 第 0 层每 64 像素一个格子，第 5 层每 2 像素一个格子
                auto f = [&](double x, double y) {
                    if (which == 0) return fbm(value, x * scale, y * scale, p);
                    if (which == 1) return fbm(perlin, x * scale, y * scale, p);
                    return fbm(simplex, x * scale, y * scale, p);
                };
                auto psd = radialPsd(f, N, 12, rng);
                // 拟合范围：r = 4 .. 64（每像素 1/64 .. 1/4 个周期，对应第 0~4 层）
                std::vector<double> lx, ly;
                for (int r = 4; r <= 64; ++r) lx.push_back(std::log10(r)), ly.push_back(std::log10(psd[r]));
                double mx = 0, my = 0;
                for (size_t i = 0; i < lx.size(); ++i) mx += lx[i], my += ly[i];
                mx /= lx.size(), my /= ly.size();
                double sxy = 0, sxx = 0;
                for (size_t i = 0; i < lx.size(); ++i) sxy += (lx[i] - mx) * (ly[i] - my), sxx += (lx[i] - mx) * (lx[i] - mx);
                double slope = sxy / sxx, res = 0;
                for (size_t i = 0; i < lx.size(); ++i) {
                    double d = ly[i] - (my + slope * (lx[i] - mx));
                    res += d * d;
                }
                std::printf("%-8s %6.2f %10.2f %10.2f %14.3f\n", which == 0 ? "value" : which == 1 ? "perlin" : "simplex", gain, slope, -(2 - 2 * std::log2(gain)),
                            std::sqrt(res / lx.size()));
            }
    }

    if (want(argc, argv, "ridge")) {
        // 单层 ridged：Perlin 在每个格点上为 0，1 - |n| 在每个格点上都是 1，山脊网络穿过全部格点，形成周期成分。
        // 采样间距 1/16，256 点正好 16 个格子；看沿坐标轴、格点频率（每单位 1 个周期，r = 16）处的功率与相邻频率之比
        std::printf("\n== single-octave spectrum along the axes: power at the lattice frequency / neighbours ==\n");
        const int N = 256;
        for (int mode = 0; mode < 4; ++mode) {
            std::vector<double> ps(N, 0);
            for (int patch = 0; patch < 20; ++patch) {
                double x0 = rng.below(1000), y0 = rng.below(1000), mean = 0;
                std::vector<std::vector<std::complex<double>>> a(N, std::vector<std::complex<double>>(N));
                for (int j = 0; j < N; ++j)
                    for (int i = 0; i < N; ++i) {
                        double x = x0 + i / 16.0, y = y0 + j / 16.0;
                        double n = mode == 0 || mode == 1 ? perlin(x, y) : value(x, y);
                        if (mode == 1 || mode == 3) n = (1 - std::abs(n)) * (1 - std::abs(n));
                        a[j][i] = n, mean += n;
                    }
                mean /= N * N;
                for (auto& row : a)
                    for (auto& v : row) v -= mean;
                for (auto& row : a) fft(row);
                for (int i = 0; i < N; ++i) {
                    std::vector<std::complex<double>> col(N);
                    for (int j = 0; j < N; ++j) col[j] = a[j][i];
                    fft(col);
                    for (int j = 0; j < N; ++j) a[j][i] = col[j];
                }
                for (int r = 1; r < N / 2; ++r) ps[r] += std::norm(a[0][r]) + std::norm(a[r][0]);
            }
            const char* names[] = {"perlin", "ridged perlin", "value", "ridged value"};
            std::printf("%-14s", names[mode]);
            for (int r : {16, 32}) std::printf("  f=%d/unit: %6.1fx", r / 16, ps[r] / ((ps[r - 2] + ps[r - 1] + ps[r + 1] + ps[r + 2]) / 4));
            std::printf("\n");
        }
    }

    if (want(argc, argv, "zeros")) {
        // 不加偏移时，各层的格点在整数点重合
        std::printf("\n== fBm (6 octaves, lacunarity 2) at integer points vs random points: std ==\n");
        std::printf("%-8s %10s %16s %16s\n", "base", "random", "integer, aligned", "integer, offset");
        for (int which = 0; which < 3; ++which) {
            FbmParams al, of;
            al.offsets = false;
            auto f = [&](double x, double y, const FbmParams& p) {
                if (which == 0) return fbm(value, x, y, p);
                if (which == 1) return fbm(perlin, x, y, p);
                return fbm(simplex, x, y, p);
            };
            double sr = 0, sa = 0, so = 0;
            const int n = 200000;
            for (int k = 0; k < n; ++k) {
                double x = rng.nextDouble() * 1e4, y = rng.nextDouble() * 1e4;
                double ix = rng.below(10000), iy = rng.below(10000);
                // simplex 的"整数点"是三角形网格的顶点：斜切坐标 (i, j) 对应原平面 (i - t, j - t)
                if (which == 2) {
                    double t = (ix + iy) * kUnskew;
                    ix -= t, iy -= t;
                }
                double a = f(x, y, al), b = f(ix, iy, al), c = f(ix, iy, of);
                sr += a * a, sa += b * b, so += c * c;
            }
            std::printf("%-8s %10.4f %16.4f %16.4f\n", which == 0 ? "value" : which == 1 ? "perlin" : "simplex", std::sqrt(sr / n), std::sqrt(sa / n), std::sqrt(so / n));
        }
    }

    if (want(argc, argv, "worley")) {
        std::printf("\n== worley: how often does the 3x3 search get it wrong? (4M random points) ==\n");
        long long w1 = 0, w2 = 0, wid = 0, searched = 0, outside = 0;
        double worst1 = 0, worst2 = 0;
        const int n = 4000000;
        for (int k = 0; k < n; ++k) {
            double x = rng.nextDouble() * 1e5, y = rng.nextDouble() * 1e5;
            auto e = worley(kLat, x, y), c = worley(kLat, x, y, 1);
            searched += e.searched;
            w1 += c.f1 != e.f1, w2 += c.f2 != e.f2, wid += c.cellX != e.cellX || c.cellY != e.cellY;
            worst1 = std::max(worst1, c.f1 - e.f1), worst2 = std::max(worst2, c.f2 - e.f2);
            outside += e.searched > 25;
        }
        std::printf("F1 wrong: %lld (%.5f%%), worst error %.4f\n", w1, 100.0 * w1 / n, worst1);
        std::printf("F2 wrong: %lld (%.4f%%), worst error %.4f\n", w2, 100.0 * w2 / n, worst2);
        std::printf("nearest cell wrong: %lld\n", wid);
        std::printf("exact search: %.2f cells on average; %.3f%% of points needed more than 5x5\n", double(searched) / n, 100.0 * outside / n);
    }

    if (want(argc, argv, "speed")) {
        std::printf("\n== nanoseconds per sample (float, 512x512 grid, spacing 1/32) ==\n");
        PermLattice perm(2026);
        auto bench = [&](const char* name, auto fn) {
            double best = 1e30, sink = 0;
            for (int round = 0; round < 5; ++round) {
                auto t0 = Clock::now();
                for (int y = 0; y < 512; ++y)
                    for (int x = 0; x < 512; ++x) sink += fn(x * (1.0f / 32) + 0.3f, y * (1.0f / 32) + 0.7f);
                best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / (512 * 512));
            }
            std::printf("%-34s %7.2f ns   (checksum %.2f)\n", name, best, sink);
        };
        bench("perlin   hash", [&](float x, float y) { return perlinNoise(kLat, x, y); });
        bench("simplex  hash", [&](float x, float y) { return simplexNoise(kLat, x, y); });
        bench("perlin   perm table", [&](float x, float y) { return perlinNoise(perm, x, y); });
        bench("simplex  perm table", [&](float x, float y) { return simplexNoise(perm, x, y); });
        bench("simplex  hash + gradient", [&](float x, float y) {
            auto s = simplexNoiseD(kLat, x, y);
            return s.value + s.dx + s.dy;
        });
        bench("worley   3x3", [&](float x, float y) { return worley(kLat, x, y, 1).f1; });
        bench("worley   exact", [&](float x, float y) { return worley(kLat, x, y).f1; });
        FbmParams p;
        bench("fbm 6 octaves of perlin (hash)", [&](float x, float y) { return fbm([&](float u, float v) { return perlinNoise(kLat, u, v); }, x, y, p); });
        bench("fbm 6 octaves of simplex (hash)", [&](float x, float y) { return fbm([&](float u, float v) { return simplexNoise(kLat, u, v); }, x, y, p); });
    }
}
