// 噪声：分布、格点结构、频谱、各向异性、导数跳变、开销、大坐标下的 float 精度（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/noise/noise_bench.cpp -o noise_bench && ./noise_bench [section]
//   section：stats grid spectrum aniso deriv speed precision（缺省全部）
#include <chrono>
#include <complex>
#include <cstdio>
#include <cstring>
#include <set>
#include <vector>

#include "noise.h"

using namespace ga;
using namespace ga::noise;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }

static const HashLattice kLat{2026};
static double valueAt(double x, double y, FadeCurve f = FadeCurve::Quintic) { return valueNoise(kLat, x, y, f); }
static double perlinAt(double x, double y, FadeCurve f = FadeCurve::Quintic) { return perlinNoise(kLat, x, y, f); }
using Fn = double (*)(double, double, FadeCurve);
struct Kind {
    const char* name;
    Fn fn;
};
static const Kind kKinds[] = {{"value", valueAt}, {"perlin", perlinAt}};

static void fft(std::vector<std::complex<double>>& a) {
    size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        std::complex<double> w = std::polar(1.0, -2 * 3.14159265358979323846 / double(len));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> cur = 1;
            for (size_t k = 0; k < len / 2; ++k, cur *= w) {
                auto u = a[i + k], v = a[i + k + len / 2] * cur;
                a[i + k] = u + v, a[i + k + len / 2] = u - v;
            }
        }
    }
}

int main(int argc, char** argv) {
    Pcg32 rng(1);

    if (want(argc, argv, "stats")) {
        std::printf("== distribution over 4M random points (quintic fade) ==\n");
        std::printf("%-8s %8s %8s %8s %8s %8s %8s %10s %10s\n", "noise", "mean", "std", "p1", "p50", "p99", "max|n|", "|n|>0.5", "|n|>0.6");
        for (const Kind& k : kKinds) {
            std::vector<double> v(4000000);
            double s = 0, s2 = 0, mx = 0;
            for (double& x : v) {
                x = k.fn((rng.nextDouble() - 0.5) * 2e4, (rng.nextDouble() - 0.5) * 2e4, FadeCurve::Quintic);
                s += x, s2 += x * x, mx = std::max(mx, std::abs(x));
            }
            long long over5 = 0, over6 = 0;
            for (double x : v) over5 += std::abs(x) > 0.5, over6 += std::abs(x) > 0.6;
            std::sort(v.begin(), v.end());
            double n = double(v.size()), mean = s / n;
            std::printf("%-8s %8.4f %8.4f %8.3f %8.3f %8.3f %8.4f %9.3f%% %9.4f%%\n", k.name, mean, std::sqrt(s2 / n - mean * mean), v[v.size() / 100],
                        v[v.size() / 2], v[v.size() * 99 / 100], mx, 100.0 * over5 / n, 100.0 * over6 / n);
        }
        std::printf("(perlin bound sqrt(2)/2 = %.4f)\n", std::sqrt(0.5));
        std::printf("\nstd by fade (value / perlin):");
        for (FadeCurve f : {FadeCurve::Linear, FadeCurve::Smooth, FadeCurve::Quintic}) {
            double sv = 0, sp = 0;
            const int n = 1000000;
            for (int i = 0; i < n; ++i) {
                double x = rng.nextDouble() * 1e4, y = rng.nextDouble() * 1e4;
                sv += valueAt(x, y, f) * valueAt(x, y, f), sp += perlinAt(x, y, f) * perlinAt(x, y, f);
            }
            std::printf("  %.4f / %.4f", std::sqrt(sv / n), std::sqrt(sp / n));
        }
        std::printf("   (linear, smooth, quintic)\n");
        std::printf("\nhistogram, 16 bins on [-0.8, 0.8], share per bin:\n");
        for (const Kind& k : kKinds) {
            long long bins[16] = {};
            const int n = 2000000;
            for (int i = 0; i < n; ++i) {
                double x = k.fn(rng.nextDouble() * 1e4, rng.nextDouble() * 1e4, FadeCurve::Quintic);
                int b = int(std::floor((x + 0.8) / 0.1));
                if (b >= 0 && b < 16) ++bins[b];
            }
            std::printf("%-8s", k.name);
            for (long long b : bins) std::printf(" %5.1f", 100.0 * b / n);
            std::printf("\n");
        }
    }

    if (want(argc, argv, "grid")) {
        // 噪声的统计性质随"在格子里的位置"变化：这就是能看出网格的原因
        std::printf("\n== std of noise at a fixed position inside the cell (over 1M cells) ==\n");
        struct P {
            const char* name;
            double fx, fy;
        } pos[] = {{"corner (0,0)", 0, 0}, {"edge mid (.5,0)", 0.5, 0}, {"center (.5,.5)", 0.5, 0.5}, {"(.25,.25)", 0.25, 0.25}};
        std::printf("%-18s %10s %10s %10s %10s\n", "position", "value", "perlin", "value lin", "perlin lin");
        for (const P& p : pos) {
            double s[4] = {};
            const int n = 1000000;
            for (int i = 0; i < n; ++i) {
                double cx = rng.below(100000), cy = rng.below(100000);
                double a = valueAt(cx + p.fx, cy + p.fy), b = perlinAt(cx + p.fx, cy + p.fy);
                double c = valueAt(cx + p.fx, cy + p.fy, FadeCurve::Linear), d = perlinAt(cx + p.fx, cy + p.fy, FadeCurve::Linear);
                s[0] += a * a, s[1] += b * b, s[2] += c * c, s[3] += d * d;
            }
            std::printf("%-18s %10.4f %10.4f %10.4f %10.4f\n", p.name, std::sqrt(s[0] / n), std::sqrt(s[1] / n), std::sqrt(s[2] / n), std::sqrt(s[3] / n));
        }
        // 梯度大小随位置的变化
        std::printf("\nmean |gradient| at those positions (quintic):\n%-18s %10s %10s\n", "position", "value", "perlin");
        for (const P& p : pos) {
            double gv = 0, gp = 0;
            const int n = 1000000;
            for (int i = 0; i < n; ++i) {
                double cx = rng.below(100000), cy = rng.below(100000);
                auto a = valueNoiseD(kLat, cx + p.fx, cy + p.fy), b = perlinNoiseD(kLat, cx + p.fx, cy + p.fy);
                gv += std::hypot(a.dx, a.dy), gp += std::hypot(b.dx, b.dy);
            }
            std::printf("%-18s %10.4f %10.4f\n", p.name, gv / n, gp / n);
        }
    }

    if (want(argc, argv, "spectrum")) {
        // 二维功率谱：256×256 的采样块（间距 1/8，覆盖 32×32 个格子），二维 FFT 后按 |频率| 分段求和，平均 40 块。
        // 注意不能只沿一条直线做一维 FFT：直线切片会把另一个方向的能量全部折叠到低频
        std::printf("\n== radial power spectrum (share of total power per band of |f|, cycles per lattice unit) ==\n");
        const int N = 256;
        const double dx = 0.125;
        const double edges[] = {0, 0.125, 0.25, 0.5, 0.75, 1.0, 1.5, 4.0001};
        const int nb = 7;
        std::printf("%-14s", "noise");
        for (int b = 0; b < nb; ++b) std::printf(" %5.3g-%-4.3g", edges[b], edges[b + 1]);
        std::printf("   peak |f|\n");
        for (FadeCurve f : {FadeCurve::Linear, FadeCurve::Quintic})
            for (const Kind& k : kKinds) {
                std::vector<double> radial(N, 0);  // 按 |f| 的细分桶，宽度 1/(N·dx)
                double band[nb] = {}, total = 0;
                for (int patch = 0; patch < 40; ++patch) {
                    double x0 = rng.nextDouble() * 1e4, y0 = rng.nextDouble() * 1e4;
                    std::vector<std::vector<std::complex<double>>> a(N, std::vector<std::complex<double>>(N));
                    for (int j = 0; j < N; ++j)
                        for (int i = 0; i < N; ++i) {
                            double w = (0.5 - 0.5 * std::cos(2 * 3.14159265358979 * i / N)) * (0.5 - 0.5 * std::cos(2 * 3.14159265358979 * j / N));
                            a[j][i] = k.fn(x0 + i * dx, y0 + j * dx, f) * w;
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
                            double fr = std::hypot(fi, fj) / (N * dx), pw = std::norm(a[j][i]);
                            if (fr >= 4) continue;  // 只统计 |f| < 4（奈奎斯特频率）
                            total += pw;
                            for (int b = 0; b < nb; ++b)
                                if (fr >= edges[b] && fr < edges[b + 1]) band[b] += pw;
                            int r = int(std::lround(std::hypot(fi, fj)));
                            if (r < N) radial[r] += pw / std::max(1.0, 2 * 3.14159265 * r);  // 按圆周长归一化：功率谱密度
                        }
                }
                int peak = 1;
                for (int r = 1; r < N / 2; ++r)
                    if (radial[r] > radial[peak]) peak = r;
                char label[32];
                std::snprintf(label, sizeof label, "%s %s", k.name, f == FadeCurve::Linear ? "lin" : "quin");
                std::printf("%-14s", label);
                for (double b : band) std::printf(" %9.1f%%", 100 * b / total);
                std::printf("   %.3f\n", peak / (N * dx));
                if (f == FadeCurve::Quintic) {  // 密度曲线（相对峰值）
                    std::printf("   density vs |f|:");
                    for (double fq : {0.0, 0.03125, 0.0625, 0.125, 0.25, 0.375, 0.5, 0.75, 1.0, 1.5})
                        std::printf(" %.3g:%.2f", fq, radial[int(std::lround(fq * N * dx))] / radial[peak]);
                    std::printf("\n");
                }
            }
    }

    if (want(argc, argv, "aniso")) {
        // 相距 r 的两点的相关系数：沿坐标轴 vs 沿对角线。各向同性的噪声两者应当相同
        std::printf("\n== correlation between points at distance r: along an axis vs along a diagonal (quintic) ==\n");
        std::printf("%-8s %5s %10s %10s %10s\n", "noise", "r", "axis", "diagonal", "diff");
        for (const Kind& k : kKinds)
            for (double r : {0.5, 1.0, 1.5, 2.0}) {
                double sa = 0, sd = 0, s0 = 0;
                const int n = 2000000;
                for (int i = 0; i < n; ++i) {
                    double x = rng.nextDouble() * 1e4, y = rng.nextDouble() * 1e4, v = k.fn(x, y, FadeCurve::Quintic);
                    double h = r * std::sqrt(0.5);
                    // 轴向取 +x 与 +y 的平均，对角取两条对角线的平均
                    sa += v * (k.fn(x + r, y, FadeCurve::Quintic) + k.fn(x, y + r, FadeCurve::Quintic)) / 2;
                    sd += v * (k.fn(x + h, y + h, FadeCurve::Quintic) + k.fn(x + h, y - h, FadeCurve::Quintic)) / 2;
                    s0 += v * v;
                }
                std::printf("%-8s %5.1f %10.4f %10.4f %+10.4f\n", k.name, r, sa / s0, sd / s0, (sa - sd) / s0);
            }
    }

    if (want(argc, argv, "aniso")) {
        // 梯度方向的分布：E[cos 4θ]（θ 是梯度方向，按梯度大小加权）。各向同性为 0；> 0 偏向坐标轴，< 0 偏向对角线
        std::printf("\n== gradient direction bias E[|g| cos 4theta] / E[|g|]  (0 = isotropic, +1 = all along axes, -1 = all diagonal) ==\n");
        for (FadeCurve f : {FadeCurve::Linear, FadeCurve::Smooth, FadeCurve::Quintic}) {
            double num[2] = {}, den[2] = {};
            for (int i = 0; i < 2000000; ++i) {
                double x = rng.nextDouble() * 1e4, y = rng.nextDouble() * 1e4;
                Sample<double> smp[2] = {valueNoiseD(kLat, x, y, f), perlinNoiseD(kLat, x, y, f)};
                for (int k = 0; k < 2; ++k) {
                    double m = std::hypot(smp[k].dx, smp[k].dy), th = std::atan2(smp[k].dy, smp[k].dx);
                    num[k] += m * std::cos(4 * th), den[k] += m;
                }
            }
            std::printf("%-8s value %+.3f   perlin %+.3f\n", f == FadeCurve::Linear ? "linear" : f == FadeCurve::Smooth ? "smooth" : "quintic", num[0] / den[0],
                        num[1] / den[1]);
        }
    }

    if (want(argc, argv, "deriv")) {
        // 跨过竖直格线时，一阶、二阶导数的跳变（沿 2000 条格线取样的最大值与平均值）
        std::printf("\n== jumps across lattice lines (value noise / perlin) ==\n");
        std::printf("%-8s %22s %22s\n", "fade", "|f' jump| max / mean", "|f'' jump| max / mean");
        for (FadeCurve f : {FadeCurve::Linear, FadeCurve::Smooth, FadeCurve::Quintic}) {
            std::printf("%-8s", f == FadeCurve::Linear ? "linear" : f == FadeCurve::Smooth ? "smooth" : "quintic");
            double m1[2] = {}, a1[2] = {}, m2[2] = {}, a2[2] = {};
            const int n = 2000;
            for (int i = 0; i < n; ++i) {
                double x = double(rng.below(10000)), y = rng.nextDouble() * 1e4;
                const double e = 1e-9, h = 1e-4;
                for (int k = 0; k < 2; ++k) {
                    auto d1 = [&](double xx) { return k == 0 ? valueNoiseD(kLat, xx, y, f).dx : perlinNoiseD(kLat, xx, y, f).dx; };
                    double j1 = std::abs(d1(x + e) - d1(x - e));
                    double j2 = std::abs((d1(x + 2 * h) - d1(x + h)) / h - (d1(x - h) - d1(x - 2 * h)) / h);
                    m1[k] = std::max(m1[k], j1), a1[k] += j1 / n, m2[k] = std::max(m2[k], j2), a2[k] += j2 / n;
                }
            }
            std::printf("   %5.2f/%-5.2f  %5.2f/%-5.2f   %6.2f/%-5.2f  %6.2f/%-5.2f\n", m1[0], a1[0], m1[1], a1[1], m2[0], a2[0], m2[1], a2[1]);
        }
        std::printf("(columns: value max/mean, perlin max/mean for f'; then the same for f'')\n");
    }

    if (want(argc, argv, "speed")) {
        std::printf("\n== nanoseconds per sample, 512x512 grid with spacing 1/32 ==\n");
        PermLattice perm(2026);
        auto bench = [&](const char* name, auto fn) {
            double best = 1e30, sink = 0;
            for (int round = 0; round < 7; ++round) {
                auto t0 = Clock::now();
                for (int y = 0; y < 512; ++y)
                    for (int x = 0; x < 512; ++x) sink += fn(x * (1.0f / 32) + 0.3f, y * (1.0f / 32) + 0.7f);
                best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / (512 * 512));
            }
            std::printf("%-30s %6.2f ns   (checksum %.3f)\n", name, best, sink);
        };
        bench("value  float  hash", [&](float x, float y) { return valueNoise(kLat, x, y); });
        bench("value  float  perm table", [&](float x, float y) { return valueNoise(perm, x, y); });
        bench("perlin float  hash", [&](float x, float y) { return perlinNoise(kLat, x, y); });
        bench("perlin float  perm table", [&](float x, float y) { return perlinNoise(perm, x, y); });
        bench("perlin float  hash, smooth", [&](float x, float y) { return perlinNoise(kLat, x, y, FadeCurve::Smooth); });
        bench("perlin double hash", [&](float x, float y) { return perlinNoise(kLat, double(x), double(y)); });
        bench("perlin float  hash + gradient", [&](float x, float y) {
            auto s = perlinNoiseD(kLat, x, y);
            return s.value + s.dx + s.dy;
        });
        bench("value  float  hash + gradient", [&](float x, float y) {
            auto s = valueNoiseD(kLat, x, y);
            return s.value + s.dx + s.dy;
        });
    }

    if (want(argc, argv, "precision")) {
        // 离原点很远时，float 坐标的小数部分精度不够。每个偏移量取 200 个格子，每个格子横向采样 256 点，与 double 结果比较
        std::printf("\n== float coordinates far from the origin (perlin, 200 cells x 256 samples per offset) ==\n");
        std::printf("%12s %14s %20s %16s %16s\n", "offset", "float spacing", "distinct x per cell", "mean |error|", "max |error|");
        for (double off : {0.0, 1e3, 1e4, 1e5, 1e6, 4e6, 1.6e7}) {
            double worst = 0, sum = 0, distinctSum = 0;
            for (int cell = 0; cell < 200; ++cell) {
                double cx = off + cell * 3, y = 0.1 + rng.nextDouble() * 50;
                std::set<float> xs;
                for (int i = 0; i < 256; ++i) {
                    double xd = cx + i / 256.0;
                    float xf = float(xd);
                    xs.insert(xf);
                    double e = std::abs(double(perlinNoise(kLat, xf, float(y))) - perlinNoise(kLat, xd, double(float(y))));
                    worst = std::max(worst, e), sum += e;
                }
                distinctSum += double(xs.size());
            }
            float o = float(std::max(off, 1.0));  // 偏移为 0 时显示 x ≈ 1 处的间距
            std::printf("%12.0f %14.3g %20.1f %16.5f %16.4f\n", off, double(std::nextafter(o, 1e30f) - o), distinctSum / 200, sum / (200 * 256), worst);
        }
    }
}
