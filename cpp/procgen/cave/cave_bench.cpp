// 元胞自动机洞穴：填充率与相变、收敛与周期、原地更新的方向偏差、柱子规则、连通处理、规模、开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/cave/cave_bench.cpp -o cave_bench && ./cave_bench [section]
//   section：fill converge exhaust r2cycle inplace pillars connect scale speed（缺省全部；exhaust 约半分钟）
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>

#include "cave.h"

using namespace ga;
using namespace ga::cave;
using Clock = std::chrono::steady_clock;

// 边长为 2r+1 的方框里墙的数量，用求和面积表（地图外算墙）在 O(1) 内得到
// 求和面积表版本，仅用于速度对比：O(1) 得到任意方框里的墙数，但建表本身比直接数 9 个格子还贵
class WallCounter {
public:
    explicit WallCounter(const Map& m) : w_(m.w), h_(m.h), s_(size_t(m.w + 2 * kPad + 1) * (m.h + 2 * kPad + 1), 0) {
        int sw = w_ + 2 * kPad + 1;
        for (int y = 0; y < h_ + 2 * kPad; ++y)
            for (int x = 0; x < w_ + 2 * kPad; ++x) {
                int mx = x - kPad, my = y - kPad;
                int wall = !m.inside(mx, my) || m.at(mx, my) == Wall;
                s_[size_t(y + 1) * sw + x + 1] = wall + s_[size_t(y) * sw + x + 1] + s_[size_t(y + 1) * sw + x] - s_[size_t(y) * sw + x];
            }
    }
    int count(int x, int y, int r) const {
        int sw = w_ + 2 * kPad + 1;
        int x0 = x + kPad - r, y0 = y + kPad - r, x1 = x + kPad + r + 1, y1 = y + kPad + r + 1;
        return s_[size_t(y1) * sw + x1] - s_[size_t(y0) * sw + x1] - s_[size_t(y1) * sw + x0] + s_[size_t(y0) * sw + x0];
    }
    static constexpr int kPad = 2;

private:
    int w_, h_;
    std::vector<int> s_;
};


static int stepSat(Map& m, const Rule& rule) {
    WallCounter wc(m);
    int changed = 0;
    for (int y = 1; y < m.h - 1; ++y)
        for (int x = 1; x < m.w - 1; ++x) {
            uint8_t v = ruleSaysWall(rule, wc.count(x, y, 1), rule.wall2 >= 0 ? wc.count(x, y, 2) : 99) ? Wall : Room;
            changed += v != m.at(x, y);
            m.at(x, y) = v;
        }
    return changed;
}

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
constexpr int W = 80, H = 50, kSeeds = 200;

static Map make(int w, int h, uint64_t seed, float p, int iters, Rule r = {}) {
    Map m(w, h);
    Pcg32 rng{seed};
    randomFill(m, rng, p);
    for (int i = 0; i < iters; ++i) step(m, r);
    return m;
}
static double interior(const Map& m) { return double(m.w - 2) * (m.h - 2); }
static int pillars(const Map& m) {  // 不接触外圈的墙块数
    std::vector<char> seen(m.t.size(), 0);
    std::vector<int> q;
    int n = 0;
    for (int i = 0; i < int(m.t.size()); ++i) {
        if (m.t[i] != Wall || seen[i]) continue;
        q.assign(1, i), seen[i] = 1;
        bool border = false;
        for (size_t h = 0; h < q.size(); ++h) {
            int x = q[h] % m.w, y = q[h] / m.w;
            border |= x == 0 || y == 0 || x == m.w - 1 || y == m.h - 1;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dungeon::kDx4[k], ny = y + dungeon::kDy4[k];
                if (m.inside(nx, ny) && m.at(nx, ny) == Wall && !seen[ny * m.w + nx]) seen[ny * m.w + nx] = 1, q.push_back(ny * m.w + nx);
            }
        }
        n += !border;
    }
    return n;
}

int main(int argc, char** argv) {
    if (want(argc, argv, "fill")) {
        std::printf("== fill: 80x50, %d seeds, 4-5 rule ==\n", kSeeds);
        std::printf("   p   | 5 iters: floor  regions  largest/floor  pillars | settled: iters  floor  regions  largest/floor\n");
        for (float p : {0.35f, 0.40f, 0.42f, 0.44f, 0.45f, 0.46f, 0.48f, 0.50f, 0.55f}) {
            double f5 = 0, r5 = 0, l5 = 0, pl = 0, it = 0, fs = 0, rs = 0, ls = 0;
            for (int s = 0; s < kSeeds; ++s) {
                Map m = make(W, H, s + 1, p, 5);
                Regions r = labelRegions(m);
                f5 += m.floorCount() / interior(m), r5 += r.size.size(), pl += pillars(m);
                l5 += r.size.empty() ? 0 : double(r.size[r.largest()]) / m.floorCount();
                Settle st = settle(m, Rule{}, 500);
                r = labelRegions(m);
                it += st.iterations, fs += m.floorCount() / interior(m), rs += r.size.size();
                ls += r.size.empty() ? 0 : double(r.size[r.largest()]) / m.floorCount();
            }
            double n = kSeeds;
            std::printf("  %.2f |   %5.1f%%  %6.1f  %8.1f%%  %8.1f | %10.1f  %5.1f%%  %6.1f  %8.1f%%\n", p, 100 * f5 / n, r5 / n,
                        100 * l5 / n, pl / n, it / n, 100 * fs / n, rs / n, 100 * ls / n);
        }
    }

    if (want(argc, argv, "converge")) {
        std::printf("== converge: 80x50 p=0.45, %d seeds ==\n", kSeeds);
        double ch[12] = {};
        for (int s = 0; s < kSeeds; ++s) {
            Map m = make(W, H, s + 1, 0.45f, 0);
            for (int i = 0; i < 12; ++i) ch[i] += step(m, Rule{});
        }
        std::printf("  changed cells per iteration:");
        for (int i = 0; i < 12; ++i) std::printf(" %.1f", ch[i] / kSeeds);
        std::printf("\n");
        for (Rule r : {Rule{5, -1}, Rule{5, 2}, Rule{6, -1}, Rule{4, -1}}) {
            int hist[3] = {}, maxIt = 0;
            double sumIt = 0;
            for (int s = 0; s < 1000; ++s) {
                Map m = make(W, H, s + 1, 0.45f, 0);
                Settle st = settle(m, r, 2000);
                ++hist[st.period], sumIt += st.iterations, maxIt = std::max(maxIt, st.iterations);
            }
            std::printf("  rule wall1=%d wall2=%2d: 1000 maps  fixed %d  2-cycle %d  none %d  iters mean %.1f max %d\n", r.wall1, r.wall2, hist[1],
                        hist[2], hist[0], sumIt / 1000, maxIt);
        }
        // R2 规则一直用下去：300 轮之后，接下来 1000 轮里变过的格子
        {
            double moving = 0, maps = 0;
            for (int s = 0; s < 100; ++s) {
                Map m = make(W, H, s + 1, 0.45f, 300, Rule{5, 2});
                std::vector<char> mv(m.t.size(), 0);
                for (int i = 0; i < 1000; ++i) {
                    std::vector<uint8_t> before = m.t;
                    step(m, Rule{5, 2});
                    for (size_t c = 0; c < m.t.size(); ++c) mv[c] |= before[c] != m.t[c];
                }
                int k = 0;
                for (char c : mv) k += c;
                moving += k, maps += k > 0;
            }
            std::printf("  R2 rule after 300 iterations: %.1f cells still changing per map (%.1f%% of interior), %d/100 maps never settle\n", moving / 100,
                        100 * moving / 100 / ((W - 2) * (H - 2)), int(maps));
        }
        // 大地图上找周期 2
        int c2 = 0;
        for (int s = 0; s < 100; ++s) {
            Map m = make(256, 256, s + 1, 0.45f, 0);
            c2 += settle(m, Rule{}, 2000).period == 2;
        }
        std::printf("  256x256: %d/100 ended in a 2-cycle\n", c2);
    }

    if (want(argc, argv, "exhaust")) {
        std::printf("== exhaust: every interior pattern of an n x n map, 4-5 rule ==\n");
        for (int n : {5, 6, 7}) {
            int k = (n - 2) * (n - 2);
            long long fixed = 0, cyc2 = 0;
            for (long long s = 0; s < (1LL << k); ++s) {
                Map m(n, n);
                for (int i = 0; i < k; ++i) m.at(1 + i % (n - 2), 1 + i / (n - 2)) = (s >> i & 1) ? Room : Wall;
                Map a = m;
                step(a, Rule{});
                if (a.t == m.t) {
                    ++fixed;
                    continue;
                }
                Map b = a;
                step(b, Rule{});
                cyc2 += b.t == m.t;
            }
            std::printf("  %dx%d: %lld patterns  fixed points %lld  on a 2-cycle %lld\n", n, n, 1LL << k, fixed, cyc2);
        }
    }

    if (want(argc, argv, "r2cycle")) {
        std::printf("== r2cycle: 80x50 p=0.45, R2 rule forever, 100 maps, up to 3000 iterations ==\n");
        std::map<int, int> hist;
        for (int s = 1; s <= 100; ++s) {
            Map m = make(W, H, s, 0.45f, 0);
            std::map<std::vector<uint8_t>, int> seen;
            int cyc = 0;
            for (int i = 0; i < 3000; ++i) {
                auto it = seen.find(m.t);
                if (it != seen.end()) {
                    cyc = i - it->second;
                    break;
                }
                seen[m.t] = i;
                step(m, Rule{5, 2});
            }
            ++hist[cyc];
        }
        std::printf("  cycle length (0 = no repeat within 3000):");
        for (auto [k, v] : hist) std::printf(" %d:%d", k, v);
        std::printf("\n");
    }

    if (want(argc, argv, "inplace")) {
        std::printf("== inplace: 160x100 p=0.45, 5 iterations, 100 seeds ==\n");
        // 相邻格子"同为墙或同为地面"的比例（去掉外圈 3 格），以及墙-地面边界的方向
        const int off[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
        for (int mode = 0; mode < 2; ++mode) {
            double same[4] = {}, cnt = 0, floor = 0, regions = 0;
            for (int s = 0; s < 100; ++s) {
                Map m(160, 100);
                Pcg32 rng{uint64_t(s + 1)};
                randomFill(m, rng, 0.45f);
                for (int i = 0; i < 5; ++i) mode ? stepInPlace(m, Rule{}) : step(m, Rule{});
                floor += m.floorCount() / interior(m), regions += labelRegions(m).size.size();
                for (int y = 3; y < m.h - 3; ++y)
                    for (int x = 3; x < m.w - 3; ++x) {
                        for (int k = 0; k < 4; ++k) same[k] += m.at(x, y) == m.at(x + off[k][0], y + off[k][1]);
                        cnt += 1;
                    }
            }
            std::printf("  %-8s floor %.1f%%  regions %.1f  same as neighbour: right %.4f  down %.4f  down-right %.4f  up-right %.4f\n",
                        mode ? "in-place" : "sync", floor /* 100 个比例之和 = 百分数 */, regions / 100, same[0] / cnt, same[1] / cnt, same[2] / cnt, same[3] / cnt);
        }
        // 质心偏移：地面的平均位置相对地图中心（单位：格）
        for (int mode = 0; mode < 2; ++mode) {
            double sx = 0, sy = 0, n = 0;
            for (int s = 0; s < 100; ++s) {
                Map m(160, 100);
                Pcg32 rng{uint64_t(s + 1)};
                randomFill(m, rng, 0.45f);
                for (int i = 0; i < 5; ++i) mode ? stepInPlace(m, Rule{}) : step(m, Rule{});
                for (int y = 0; y < m.h; ++y)
                    for (int x = 0; x < m.w; ++x)
                        if (m.floor(x, y)) sx += x - (m.w - 1) / 2.0, sy += y - (m.h - 1) / 2.0, n += 1;
            }
            std::printf("  %-8s floor centroid offset (%.2f, %.2f) cells\n", mode ? "in-place" : "sync", sx / n, sy / n);
        }
        // 一直更新到不再变化（同步更新遇到周期 2 时在 300 轮停下）：原地更新放大了多数一方
        for (float p : {0.45f, 0.50f, 0.55f})
            for (int mode = 0; mode < 2; ++mode) {
                double f = 0;
                for (int s = 0; s < 100; ++s) {
                    Map m(160, 100);
                    Pcg32 rng{uint64_t(s + 1)};
                    randomFill(m, rng, p);
                    for (int i = 0; i < 300; ++i)
                        if ((mode ? stepInPlace(m, Rule{}) : step(m, Rule{})) == 0) break;
                    f += m.floorCount() / interior(m);
                }
                std::printf("  p %.2f %-8s settled floor %.1f%%\n", p, mode ? "in-place" : "sync", f);
            }
    }

    if (want(argc, argv, "pillars")) {
        std::printf("== pillars: 80x50, %d seeds, 7 iterations total ==\n", kSeeds);
        std::printf("  p     variant          floor  regions  pillars  max radius  floor with radius>=4\n");
        for (float p : {0.40f, 0.45f}) {
            for (int v = 0; v < 3; ++v) {
                double f = 0, r = 0, pl = 0, mr = 0, big = 0;
                for (int s = 0; s < kSeeds; ++s) {
                    Map m(W, H);
                    Pcg32 rng{uint64_t(s + 1)};
                    randomFill(m, rng, p);
                    for (int i = 0; i < 7; ++i) step(m, (v == 1 && i < 4) || v == 2 ? Rule{5, 2} : Rule{});
                    std::vector<int> d = openRadius(m);
                    int mx = 0, b = 0;
                    for (int x : d) mx = std::max(mx, x), b += x >= 4;
                    f += m.floorCount() / interior(m), r += labelRegions(m).size.size(), pl += pillars(m), mr += mx;
                    big += double(b) / std::max(1, m.floorCount());
                }
                const char* name[] = {"4-5 x7", "R2 x4 + 4-5 x3", "R2 x7"};
                double n = kSeeds;
                std::printf("  %.2f  %-15s %5.1f%%  %6.1f  %7.1f  %10.2f  %8.1f%%\n", p, name[v], 100 * f / n, r / n, pl / n, mr / n, 100 * big / n);
            }
        }
    }

    if (want(argc, argv, "connect")) {
        std::printf("== connect: 80x50, %d seeds, 5 iterations ==\n", kSeeds);
        std::printf("  p     regions  keepLargest: floor lost | connect: tunnel cells  floor | dead ends  detour  diameter  sweep exact\n");
        for (float p : {0.40f, 0.45f, 0.48f, 0.50f}) {
            double reg = 0, lost = 0, tun = 0, fl = 0, de = 0, dt = 0, dia = 0, exact = 0, de2 = 0, dt2 = 0;
            for (int s = 0; s < kSeeds; ++s) {
                Map m = make(W, H, s + 1, p, 5);
                reg += labelRegions(m).size.size();
                Map k = m;
                lost += double(keepLargest(k)) / std::max(1, m.floorCount());
                Map c = m;
                tun += connectRegions(c);
                fl += c.floorCount() / interior(c);
                Pcg32 r2(1);
                de += dungeon::deadEnds(c), dt += dungeon::detourRatio(c, r2, 30, 30, 10);
                Pcg32 r3(1);
                de2 += dungeon::deadEnds(k), dt2 += dungeon::detourRatio(k, r3, 30, 30, 10);
                if (s < 50) {
                    dungeon::Diameter ex = dungeon::exactDiameter(c);
                    int st = 0;
                    while (c.t[st] == Wall) ++st;
                    dia += ex.length, exact += dungeon::doubleSweep(c, st).length == ex.length;
                }
            }
            double n = kSeeds;
            std::printf("  %.2f  %6.1f  %18.1f%% | %18.1f  %5.1f%% | %9.1f  %6.2f  %8.1f  %8.0f%%\n", p, reg / n, 100 * lost / n, tun / n,
                        100 * fl / n, de / n, dt / n, dia / 50, 100 * exact / 50);
            std::printf("        (keepLargest only: dead ends %.1f  detour %.2f)\n", de2 / n, dt2 / n);
        }
        // 对照：随机游走挖到同样的地面比例
        for (float cov : {0.45f, 0.55f}) {
            double de = 0, dt = 0;
            for (int s = 0; s < kSeeds; ++s) {
                Map m(W, H);
                Pcg32 rng{uint64_t(s + 1)};
                dungeon::WalkParams wp;
                wp.coverage = cov;
                dungeon::randomWalk(m, rng, wp);
                Pcg32 r2(1);
                de += dungeon::deadEnds(m), dt += dungeon::detourRatio(m, r2, 30, 30, 10);
            }
            std::printf("  random walk coverage %.2f: dead ends %.1f  detour %.2f\n", cov, de / kSeeds, dt / kSeeds);
        }
    }

    if (want(argc, argv, "scale")) {
        std::printf("== scale: 5 iterations ==\n");
        std::printf("  size      p     regions  per 1000 cells  largest/floor  largest/interior\n");
        for (float p : {0.45f, 0.48f})
            for (int k : {1, 2, 4, 8}) {
                int w = 40 * k, h = 25 * k, seeds = std::max(10, 200 / (k * k));
                double r = 0, l = 0, li = 0;
                for (int s = 0; s < seeds; ++s) {
                    Map m = make(w, h, s + 1, p, 5);
                    Regions rg = labelRegions(m);
                    r += rg.size.size();
                    double big = rg.size.empty() ? 0 : rg.size[rg.largest()];
                    l += big / std::max(1, m.floorCount()), li += big / interior(m);
                }
                std::printf("  %3dx%-4d  %.2f  %7.1f  %14.2f  %12.1f%%  %14.1f%%\n", w, h, p, r / seeds, 1000 * r / seeds / interior(Map(w, h)),
                            100 * l / seeds, 100 * li / seeds);
            }
    }

    if (want(argc, argv, "speed")) {
        std::printf("== speed ==\n");
        for (int n : {80, 512}) {
            int w = n, h = n == 80 ? 50 : n, reps = n == 80 ? 2000 : 40;
            Map m = make(w, h, 1, 0.45f, 0);
            auto time = [&](auto&& f) {
                auto t0 = Clock::now();
                for (int i = 0; i < reps; ++i) {
                    Map c = m;
                    f(c);
                }
                return std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / reps;
            };
            double direct = time([](Map& c) { step(c, Rule{}); });
            double direct2 = time([](Map& c) { step(c, Rule{5, 2}); });
            double sat = time([](Map& c) { stepSat(c, Rule{}); });
            double sat2 = time([](Map& c) { stepSat(c, Rule{5, 2}); });
            double inplace = time([](Map& c) { stepInPlace(c, Rule{}); });
            Map s5 = make(w, h, 1, 0.45f, 5);
            auto t0 = Clock::now();
            for (int i = 0; i < reps; ++i) {
                Map c = s5;
                connectRegions(c);
            }
            double con = std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / reps;
            std::printf("  %dx%d: step direct %.1f us (with R2 %.1f)  summed-area %.1f us (with R2 %.1f)  in-place 5x5 %.1f us  connectRegions %.1f us (%zu regions)\n",
                        w, h, direct, direct2, sat, sat2, inplace, con, labelRegions(s5).size.size());
        }
    }
}
