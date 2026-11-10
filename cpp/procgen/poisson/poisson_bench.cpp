// 泊松圆盘采样：投飞镖的饱和、Bridson 的密度与 k、环带取样方式、活跃点的选法、最近邻距离、功率谱、开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/poisson/poisson_bench.cpp -o poisson_bench && ./poisson_bench [section]
//   section：dart bridson pick nn spectrum speed（缺省全部）
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "poisson.h"

using namespace ga;
using namespace ga::poisson;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }

// 内部密度：只数离边界至少 2r 的窗口里的点，避免边界附近密度偏高的影响
static double bulkDensity(const std::vector<Vec2>& p, float w, float h, float r) {
    int n = 0;
    for (const Vec2& a : p) n += a.x >= 2 * r && a.y >= 2 * r && a.x < w - 2 * r && a.y < h - 2 * r;
    return packingDensity(n, w - 4 * r, h - 4 * r, r);
}
static double edgeDensity(const std::vector<Vec2>& p, float w, float h, float r) {  // 离边界 r 以内的条带
    int n = 0;
    for (const Vec2& a : p) n += a.x < r || a.y < r || a.x >= w - r || a.y >= h - r;
    return packingDensity(n, 1, w * h - (w - 2 * r) * (h - 2 * r), r);
}

static const char* annName(Annulus a) { return a == Annulus::Area ? "area" : a == Annulus::Radius ? "radius" : "circle"; }

int main(int argc, char** argv) {
    const float W = 40, H = 40, R = 1;
    if (want(argc, argv, "dart")) {
        std::printf("== dart: 40x40, r=1, 20 seeds; stop after N consecutive failures ==\n");
        std::printf("  N        points  bulk density  edge strip  uncovered  attempts/point\n");
        for (long long N : {10LL, 100LL, 1000LL, 10000LL, 100000LL, 1000000LL}) {
            double n = 0, bd = 0, ed = 0, un = 0, at = 0;
            for (int s = 0; s < 20; ++s) {
                Pcg32 rng{uint64_t(s + 1)};
                long long tries = 0;
                std::vector<Vec2> p = dartThrowing(W, H, R, rng, N, true, &tries);
                n += p.size(), bd += bulkDensity(p, W, H, R), ed += edgeDensity(p, W, H, R);
                un += uncoveredFraction(p, W, H, R, 800), at += double(tries) / p.size();
            }
            std::printf("  %-8lld %6.0f  %12.4f  %10.4f  %8.4f%%  %14.0f\n", N, n / 20, bd / 20, ed / 20, 100 * un / 20, at / 20);
        }
    }

    if (want(argc, argv, "bridson")) {
        std::printf("== bridson: 40x40, r=1, 20 seeds, random pick ==\n");
        std::printf("  annulus  k     points  bulk density  uncovered  attempts/point\n");
        for (Annulus an : {Annulus::Area, Annulus::Radius, Annulus::Circle})
            for (int k : {1, 2, 5, 10, 30, 100, 300}) {
                double n = 0, bd = 0, un = 0, at = 0;
                for (int s = 0; s < 20; ++s) {
                    Pcg32 rng{uint64_t(s + 1)};
                    BridsonParams prm;
                    prm.k = k, prm.annulus = an;
                    long long tries = 0;
                    std::vector<Vec2> p = bridson(W, H, R, rng, prm, &tries);
                    n += p.size(), bd += bulkDensity(p, W, H, R), un += uncoveredFraction(p, W, H, R, 800), at += double(tries) / p.size();
                }
                std::printf("  %-7s %4d  %6.0f  %12.4f  %8.4f%%  %14.1f\n", annName(an), k, n / 20, bd / 20, 100 * un / 20, at / 20);
            }
    }

    if (want(argc, argv, "pick")) {
        std::printf("== pick: 40x40, r=1, k=30, 20 seeds ==\n");
        const char* names[] = {"random", "oldest", "newest"};
        for (Annulus an : {Annulus::Area, Annulus::Circle})
            for (Pick pk : {Pick::Random, Pick::Oldest, Pick::Newest}) {
                double bd = 0, un = 0, nn = 0;
                for (int s = 0; s < 20; ++s) {
                    Pcg32 rng{uint64_t(s + 1)};
                    BridsonParams prm;
                    prm.annulus = an, prm.pick = pk;
                    std::vector<Vec2> p = bridson(W, H, R, rng, prm);
                    bd += bulkDensity(p, W, H, R), un += uncoveredFraction(p, W, H, R, 800);
                    std::vector<float> d = nearestNeighbor(p, W, H, 1);
                    double m = 0;
                    for (float x : d) m += x / d.size();
                    nn += m;
                }
                std::printf("  %-7s %-7s bulk density %.4f  uncovered %.4f%%  mean NN %.3f r\n", annName(an), names[int(pk)], bd / 20, 100 * un / 20, nn / 20);
            }
    }

    if (want(argc, argv, "nn")) {
        std::printf("== nn: nearest-neighbour distance / r, 40x40, 20 seeds (white noise and jitter use the Bridson point count) ==\n");
        std::printf("  method              points  min     mean   in [r,1.1r)  >1.5r   neighbours in [r,1.05r)  with >=4 of them\n");
        for (int m = 0; m < 5; ++m) {
            double n = 0, mn = 0, mean = 0, near = 0, far = 0, nbs = 0, hex = 0;
            for (int s = 0; s < 20; ++s) {
                Pcg32 rng{uint64_t(s + 1)};
                BridsonParams prm;
                std::vector<Vec2> ref = bridson(W, H, R, rng, prm);
                std::vector<Vec2> p;
                if (m == 0) p = whiteNoise(W, H, int(ref.size()), rng);
                else if (m == 1) p = jitteredGrid(W, H, std::sqrt(W * H / ref.size()), rng);
                else if (m == 2) p = dartThrowing(W, H, R, rng, 100000);
                else if (m == 3) p = ref;
                else {
                    prm.annulus = Annulus::Circle;
                    p = bridson(W, H, R, rng, prm);
                }
                std::vector<float> d = nearestNeighbor(p, W, H, 1);
                float lo = INFINITY;
                double sum = 0, a = 0, b = 0;
                for (float x : d) lo = std::min(lo, x), sum += x, a += x >= R && x < 1.1f * R, b += x > 1.5f * R;
                n += p.size(), mn += lo, mean += sum / d.size(), near += a / d.size(), far += b / d.size();
                // 局部的晶格化：距离在 [r, 1.05r) 的邻居数（六角密排是 6）
                double nb = 0, many = 0;
                for (size_t i = 0; i < p.size(); ++i) {
                    int c = 0;
                    for (size_t j = 0; j < p.size(); ++j)
                        if (j != i) {
                            float d = std::hypot(p[i].x - p[j].x, p[i].y - p[j].y);
                            c += d >= 0.999f * R && d < 1.05f * R;
                        }
                    nb += c, many += c >= 4;
                }
                nbs += nb / p.size(), hex += many / p.size();
            }
            const char* names[] = {"white noise", "jittered grid", "dart (1e5 fails)", "bridson area k30", "bridson circle k30"};
            std::printf("  %-19s %6.0f  %.3f  %.3f  %10.1f%%  %5.1f%%  %23.2f  %15.1f%%\n", names[m], n / 20, mn / 20, mean / 20, 100 * near / 20, 100 * far / 20, nbs / 20, 100 * hex / 20);
        }
    }

    if (want(argc, argv, "spectrum")) {
        const float S = 32;
        const int M = 64;  // 频率到 M / S = 2 个周期 / r
        std::printf("== spectrum: 32x32, r=1, radially averaged periodogram, 10 seeds; f in cycles per r ==\n");
        std::printf("  method              points  f<0.25   0.25-0.5  0.5-0.75  peak (f, P)      1.5-2.0\n");
        for (int m = 0; m < 5; ++m) {
            std::vector<double> acc(M + 1, 0);
            double n = 0;
            for (int s = 0; s < 10; ++s) {
                Pcg32 rng{uint64_t(s + 1)};
                BridsonParams prm;
                std::vector<Vec2> ref = bridson(S, S, R, rng, prm);
                std::vector<Vec2> p;
                if (m == 0) p = whiteNoise(S, S, int(ref.size()), rng);
                else if (m == 1) p = jitteredGrid(S, S, std::sqrt(S * S / ref.size()), rng);
                else if (m == 2) p = dartThrowing(S, S, R, rng, 100000);
                else if (m == 3) p = ref;
                else {
                    prm.annulus = Annulus::Circle;
                    p = bridson(S, S, R, rng, prm);
                }
                n += p.size();
                std::vector<double> rad = radialAverage(periodogram(p, S, S, M), M);
                for (int b = 0; b <= M; ++b) acc[b] += rad[b] / 10;
            }
            auto band = [&](double f0, double f1) {
                double s = 0, c = 0;
                for (int b = 1; b <= M; ++b) {
                    double f = b / S;
                    if (f >= f0 && f < f1) s += acc[b], c += 1;
                }
                return s / c;
            };
            int pk = 1;
            for (int b = 1; b <= M; ++b)
                if (acc[b] > acc[pk]) pk = b;
            const char* names[] = {"white noise", "jittered grid", "dart (1e5 fails)", "bridson area k30", "bridson circle k30"};
            std::printf("  %-19s %6.0f  %6.3f  %8.3f  %8.3f  (%.2f, %.2f)     %6.3f\n", names[m], n / 10, band(0, 0.25), band(0.25, 0.5), band(0.5, 0.75),
                        pk / S, acc[pk], band(1.5, 2.01));
        }
    }

    if (want(argc, argv, "speed")) {
        std::printf("== speed: 100x100, r=1 ==\n");
        auto ms = [](auto&& f) {
            auto t0 = Clock::now();
            f();
            return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        };
        size_t n = 0;
        long long tries = 0;
        double naive = ms([&] {
            Pcg32 rng{1};
            n = dartThrowing(100, 100, 1, rng, 1000, false, &tries).size();
        });
        std::printf("  dart, linear scan, 1000 fails: %.1f ms  %zu points  %lld attempts\n", naive, n, tries);
        double grid = ms([&] {
            Pcg32 rng{1};
            n = dartThrowing(100, 100, 1, rng, 1000, true, &tries).size();
        });
        std::printf("  dart, grid,        1000 fails: %.1f ms  %zu points  %lld attempts\n", grid, n, tries);
        double grid5 = ms([&] {
            Pcg32 rng{1};
            n = dartThrowing(100, 100, 1, rng, 100000, true, &tries).size();
        });
        std::printf("  dart, grid,      100000 fails: %.1f ms  %zu points  %lld attempts\n", grid5, n, tries);
        for (Annulus an : {Annulus::Area, Annulus::Circle}) {
            double t = 0;
            for (int rep = 0; rep < 10; ++rep)
                t += ms([&] {
                    Pcg32 rng{uint64_t(rep + 1)};
                    BridsonParams prm;
                    prm.annulus = an;
                    n = bridson(100, 100, 1, rng, prm, &tries).size();
                });
            std::printf("  bridson %-6s k=30: %.2f ms  %zu points  %lld attempts\n", annName(an), t / 10, n, tries);
        }
    }
}
