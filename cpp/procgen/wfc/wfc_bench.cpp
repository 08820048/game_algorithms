// 波函数坍缩：启发式与矛盾率、尺寸与回溯、输出分布的偏差、图块比例、开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/wfc/wfc_bench.cpp -o wfc_bench && ./wfc_bench [section]
//   section：heuristic size bias freq terrain cost（缺省全部）
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <vector>

#include "samples.h"
#include "wfc.h"

using namespace ga;
using namespace ga::wfc;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
static double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

struct Case {
    const char* name;
    Rules rules;
    bool periodic;
};
static std::vector<Case> cases() {
    std::vector<Case> out;
    out.push_back({"roads", fromSockets(roadTiles(false)), false});
    out.push_back({"roads, no dead ends", fromSockets(roadTiles(true)), false});
    out.push_back({"terrain corners", fromSockets(terrainTiles()), false});
    std::string al;
    out.push_back({"rooms N=3", fromSample(sampleFromText(roomsText(), &al), 3, true, false), false});
    out.push_back({"lakes N=3", fromSample(sampleFromText(lakesText(), &al), 3, true, true), true});
    out.push_back({"maze N=3", fromSample(sampleFromText(mazeText(), &al), 3, true, true), true});
    return out;
}
static const char* hName(Heuristic h) { return h == Heuristic::Entropy ? "entropy" : h == Heuristic::MinCount ? "min-count" : h == Heuristic::Scanline ? "scanline" : "random"; }

int main(int argc, char** argv) {
    if (want(argc, argv, "heuristic")) {
        std::printf("== heuristic: 40x40, no backtracking; success rate over runs ==\n");
        std::printf("  tileset               T    heuristic   success   ms/run\n");
        for (Case& c : cases()) {
            int runs = c.rules.T > 40 ? 100 : 400;
            for (Heuristic h : {Heuristic::Entropy, Heuristic::MinCount, Heuristic::Scanline, Heuristic::Random}) {
                int ok = 0;
                auto t0 = Clock::now();
                for (int s = 0; s < runs; ++s) {
                    Solver sol(c.rules, 40, 40, {h, c.periodic}, uint64_t(s + 1));
                    ok += sol.run() == Status::Done;
                }
                std::printf("  %-20s %3d   %-10s %6.1f%%  %7.2f\n", c.name, c.rules.T, hName(h), 100.0 * ok / runs, msSince(t0) / runs);
            }
        }
    }

    if (want(argc, argv, "size")) {
        std::printf("== size: success rate vs n (n x n output), entropy heuristic ==\n");
        std::string al;
        struct S {
            const char* name;
            Rules r;
            bool periodic;
            Heuristic h;
        };
        std::vector<S> ss = {
            {"roads no dead ends, random", fromSockets(roadTiles(true)), false, Heuristic::Random},
            {"rooms N=3", fromSample(sampleFromText(roomsText(), &al), 3, true, false), false, Heuristic::Entropy},
            {"rooms N=4", fromSample(sampleFromText(roomsText(), &al), 4, true, false), false, Heuristic::Entropy},
            {"maze N=3", fromSample(sampleFromText(mazeText(), &al), 3, true, true), true, Heuristic::Entropy},
            {"maze N=4", fromSample(sampleFromText(mazeText(), &al), 4, true, true), true, Heuristic::Entropy},
            {"lakes N=4", fromSample(sampleFromText(lakesText(), &al), 4, true, true), true, Heuristic::Entropy},
        };
        for (S& s : ss) {
            std::printf("  %s (T = %d)\n", s.name, s.r.T);
            for (int n : {16, 32, 64, 128}) {
                int runs = n >= 128 ? 20 : n >= 64 ? 50 : 100;
                int ok = 0, okB = 0;
                double obs = 0, bt = 0, ms = 0, msB = 0;
                for (int k = 0; k < runs; ++k) {
                    auto t0 = Clock::now();
                    Solver a(s.r, n, n, {s.h, s.periodic}, uint64_t(k + 1));
                    ok += a.run() == Status::Done;
                    ms += msSince(t0);
                    obs += a.stats().observations;
                    t0 = Clock::now();
                    Solver b(s.r, n, n, {s.h, s.periodic, true, 100000}, uint64_t(k + 1));
                    okB += b.run() == Status::Done;
                    msB += msSince(t0);
                    bt += b.stats().backtracks;
                }
                std::printf("    n=%-4d no backtracking %5.1f%% (%.1f ms)   backtracking %5.1f%%  backtracks/run %8.1f  (%.1f ms)\n", n, 100.0 * ok / runs,
                            ms / runs, 100.0 * okB / runs, bt / runs, msB / runs);
                (void)obs;
            }
        }
    }

    if (want(argc, argv, "bias")) {
        std::printf("== bias: distribution of outputs over all valid tilings ==\n");
        for (int weighted = 0; weighted < 2; ++weighted)
            for (auto [w, h] : {std::pair{2, 2}, std::pair{3, 2}}) {
                Rules r = fromSockets(roadTiles(true));
                if (!weighted) std::fill(r.weight.begin(), r.weight.end(), 1.0);
                // 穷举所有解与"目标"概率：权重统一时是均匀分布，否则 ∝ 各格权重之积
                std::vector<std::vector<int>> sols;
                std::vector<int> cur(size_t(w) * h);
                std::function<void(int)> rec = [&](int c) {
                    if (c == w * h) {
                        sols.push_back(cur);
                        return;
                    }
                    for (int t = 0; t < r.T; ++t) {
                        if (c % w > 0 && !r.allows(cur[c - 1], 0, t)) continue;
                        if (c / w > 0 && !r.allows(cur[c - w], 1, t)) continue;
                        cur[c] = t;
                        rec(c + 1);
                    }
                };
                rec(0);
                std::map<std::vector<int>, int> id;
                std::vector<double> target(sols.size());
                double z = 0;
                for (size_t i = 0; i < sols.size(); ++i) {
                    id[sols[i]] = int(i);
                    double p = 1;
                    for (int t : sols[i]) p *= r.weight[t];
                    target[i] = p, z += p;
                }
                for (double& p : target) p /= z;
                for (Heuristic hh : {Heuristic::Entropy, Heuristic::Scanline}) {
                    std::vector<double> got(sols.size(), 0);
                    const int n = 4000000;
                    for (int k = 0; k < n; ++k) {
                        Solver s(r, w, h, {hh}, uint64_t(k + 1));
                        s.run();
                        got[id[s.result()]] += 1.0 / n;
                    }
                    double tv = 0, lo = INFINITY, hi = 0;
                    for (size_t i = 0; i < sols.size(); ++i) {
                        tv += std::abs(got[i] - target[i]) / 2;
                        double ratio = got[i] / target[i];
                        lo = std::min(lo, ratio), hi = std::max(hi, ratio);
                    }
                    // 抽样噪声本身造成的 TV 距离（期望值约 √(K/(2πn))）
                    double noise = std::sqrt(double(sols.size()) / (2 * M_PI * n));
                    std::printf("  %dx%d %-8s %-8s solutions %6zu  TV distance %.3f (sampling noise ~%.3f)  P/target from %.2f to %.2f\n", w, h,
                                weighted ? "weighted" : "flat", hName(hh), sols.size(), tv, noise, lo, hi);
                }
            }
    }

    if (want(argc, argv, "freq")) {
        std::printf("== freq: tile shares in the output vs normalised weights, roads 64x64, 50 runs ==\n");
        Rules r = fromSockets(roadTiles(false));
        auto tiles = roadTiles(false);
        std::vector<double> share(r.T, 0);
        double tw = 0;
        for (double w : r.weight) tw += w;
        int runs = 0;
        for (int k = 0; k < 50; ++k) {
            Solver s(r, 64, 64, {}, uint64_t(k + 1));
            if (s.run() != Status::Done) continue;
            ++runs;
            for (int t : s.result()) share[t] += 1.0 / (64 * 64);
        }
        const char* kind[] = {"empty", "dead end", "straight", "corner", "T", "cross"};
        std::map<std::string, std::pair<double, double>> agg;
        for (int t = 0; t < r.T; ++t) {
            int m = roadMask(tiles[t]), k = __builtin_popcount(unsigned(m));
            const char* nm = k == 0 ? kind[0] : k == 1 ? kind[1] : k == 2 ? ((m == 5 || m == 10) ? kind[2] : kind[3]) : k == 3 ? kind[4] : kind[5];
            agg[nm].first += r.weight[t] / tw, agg[nm].second += share[t] / runs;
        }
        for (auto& [nm, v] : agg) std::printf("  %-9s weight share %.3f   output share %.3f\n", nm.c_str(), v.first, v.second);
        // 重叠模型：样图里墙的比例 vs 输出里墙的比例
        std::string al;
        for (int which = 0; which < 4; ++which) {  // 0 房间（不加对称）、1 房间（加对称）、2 湖泊、3 迷宫
            Sample smp = sampleFromText(which <= 1 ? roomsText() : which == 2 ? lakesText() : mazeText(), &al);
            std::vector<std::vector<uint8_t>> pats;
            bool per = which >= 2;
            Rules ro = fromSample(smp, 3, which != 0, per, &pats);
            int ncol = int(al.size());
            std::vector<double> in(ncol, 0), out(ncol, 0);
            for (uint8_t p : smp.px) in[p] += 1.0 / smp.px.size();
            int ok = 0;
            for (int k = 0; k < 20; ++k) {
                Solver s(ro, 48, 48, {Heuristic::Entropy, per, true, 100000}, uint64_t(k + 1));
                if (s.run() != Status::Done) continue;
                ++ok;
                for (int t : s.result()) out[pats[t][0]] += 1.0 / (48 * 48);
            }
            const char* names[] = {"rooms (no symmetry)", "rooms (symmetry)", "lakes", "maze"};
            std::printf("  %-20s T=%3d", names[which], ro.T);
            for (int c = 0; c < ncol; ++c)
                if (in[c] > 0) std::printf("  '%c' sample %.3f output %.3f", al[c], in[c], out[c] / std::max(1, ok));
            std::printf("\n");
        }
    }

    if (want(argc, argv, "terrain")) {
        std::printf("== terrain: weight of the uniform tiles vs map composition, 100 runs ==\n");
        auto corners = terrainCorners();
        for (int n : {24, 64}) {
            std::printf("  %dx%d\n", n, n);
            for (double uw : {20.0, 14.0, 13.0, 12.9, 12.8, 12.0, 8.0, 4.0, 1.0}) {
                Rules r = fromSockets(terrainTiles(uw));
                double f[3] = {0, 0, 0}, dom = 0;
                int both = 0, runs = n > 30 ? 40 : 100;
                for (int k = 0; k < runs; ++k) {
                    Solver s(r, n, n, {}, uint64_t(k + 1));
                    s.run();
                    double g[3] = {0, 0, 0};
                    for (int t : s.result()) {
                        const CornerTile& c = corners[t];
                        g[c.tl] += 0.25, g[c.tr] += 0.25, g[c.br] += 0.25, g[c.bl] += 0.25;
                    }
                    for (int j = 0; j < 3; ++j) f[j] += g[j] / (n * n) / runs;
                    both += g[0] > 0 && g[2] > 0;
                    dom += std::max(g[0], g[2]) / (n * n) / runs;
                }
                std::printf("    uniform weight %4.1f  water %.3f  sand %.3f  grass %.3f  larger of water/grass %.3f  maps with both %3.0f%%\n", uw, f[0], f[1], f[2],
                            dom, 100.0 * both / runs);
            }
        }
    }

    if (want(argc, argv, "terrain")) {
        // 熵的交叉点：紧挨纯色格子的格子剩 {w, 1, 1, 1} 四个候选，它的熵在 w ≈ 12.85 时等于两个等权候选的 log 2
        double lo = 1, hi = 20;
        for (int it = 0; it < 60; ++it) {
            double w = (lo + hi) / 2, H = std::log(w + 3) - w * std::log(w) / (w + 3);
            (H > std::log(2.0) ? lo : hi) = w;
        }
        std::printf("  H(w,1,1,1) = log 2 at w = %.3f\n", lo);
        std::printf("  other heuristics, 24x24, sand share:\n");
        auto corners = terrainCorners();
        for (Heuristic h : {Heuristic::Entropy, Heuristic::MinCount, Heuristic::Scanline, Heuristic::Random}) {
            std::printf("    %-9s", hName(h));
            for (double uw : {20.0, 12.9, 12.8, 4.0}) {
                Rules r = fromSockets(terrainTiles(uw));
                double sand = 0;
                for (int k = 0; k < 100; ++k) {
                    Solver s(r, 24, 24, {h}, uint64_t(k + 1));
                    s.run();
                    for (int t : s.result()) {
                        const CornerTile& c = corners[t];
                        sand += ((c.tl == 1) + (c.tr == 1) + (c.br == 1) + (c.bl == 1)) / 4.0 / (24 * 24) / 100;
                    }
                }
                std::printf("  w=%-4.1f %.3f", uw, sand);
            }
            std::printf("\n");
        }
    }

    if (want(argc, argv, "cost")) {
        std::printf("== cost: roads (16 tiles) and lakes N=3, n x n, entropy vs scanline ==\n");
        std::string al;
        Rules lakes = fromSample(sampleFromText(lakesText(), &al), 3, true, true);
        Rules roads = fromSockets(roadTiles(false));
        for (int which = 0; which < 2; ++which) {
            const Rules& r = which ? lakes : roads;
            std::printf("  %s (T = %d)\n", which ? "lakes N=3" : "roads", r.T);
            for (int n : {32, 64, 128, 256}) {
                for (Heuristic h : {Heuristic::Entropy, Heuristic::Scanline}) {
                    auto t0 = Clock::now();
                    Solver s(r, n, n, {h, which == 1, true, 100000}, 1);
                    Status st = s.run();
                    double ms = msSince(t0);
                    const SolveStats& k = s.stats();
                    double cells = double(n) * n;
                    std::printf("    n=%-4d %-9s %s  observations/cell %.3f  revisions/cell %6.1f  scanned/cell %9.1f  %9.1f ms\n", n, hName(h),
                                st == Status::Done ? "ok  " : "FAIL", k.observations / cells, k.revisions / cells, k.scanned / cells, ms);
                }
            }
        }
    }
}
