// 地牢生成：随机游走的效率与形状、BSP 的切割方向与连接方式、随机房间的饱和、环路与绕路、两次 BFS 求直径的误差、开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/dungeon/dungeon_bench.cpp -o dungeon_bench && ./dungeon_bench [section]
//   section：walk sites bsp rooms diameter speed（缺省全部）
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unordered_set>
#include <vector>

#include "dungeon.h"

using namespace ga;
using namespace ga::dungeon;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
constexpr int W = 80, H = 50;

struct Summary {
    double floor = 0, dead = 0, detour = 0, diam = 0, comps = 0, n = 0;
    void add(const Map& m, Pcg32& rng) {
        floor += double(m.floorCount()) / ((m.w - 2) * (m.h - 2));
        dead += deadEnds(m), comps += components(m);
        detour += detourRatio(m, rng, 30, 30, 10);
        int s = 0;
        while (m.t[s] == Wall) ++s;
        diam += doubleSweep(m, s).length;
        n += 1;
    }
    void print(const char* name) const {
        std::printf("  %-26s floor %5.1f%%  dead ends %6.1f  detour %5.2f  diameter %6.1f  components %.2f\n", name, 100 * floor / n, dead / n,
                    detour / n, diam / n, comps / n);
    }
};

// 走廊穿过了几个"既不是起点也不是终点"的房间
static int roomsCrossed(const std::vector<Rect>& rooms, int ra, int rb, const Link& l) {
    std::vector<char> hit(rooms.size(), 0);
    auto visit = [&](int x, int y) {
        for (size_t i = 0; i < rooms.size(); ++i) {
            const Rect& r = rooms[i];
            if (x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h) hit[i] = 1;
        }
    };
    if (l.horizontalFirst) {
        for (int x = std::min(l.ax, l.bx); x <= std::max(l.ax, l.bx); ++x) visit(x, l.ay);
        for (int y = std::min(l.ay, l.by); y <= std::max(l.ay, l.by); ++y) visit(l.bx, y);
    } else {
        for (int y = std::min(l.ay, l.by); y <= std::max(l.ay, l.by); ++y) visit(l.ax, y);
        for (int x = std::min(l.ax, l.bx); x <= std::max(l.ax, l.bx); ++x) visit(x, l.by);
    }
    int n = 0;
    for (size_t i = 0; i < rooms.size(); ++i) n += hit[i] && int(i) != ra && int(i) != rb;
    return n;
}

int main(int argc, char** argv) {
    const int seeds = 200;

    // ---- 随机游走：步数、浪费、形状 ----
    if (want(argc, argv, "walk")) {
        std::printf("== walk: 80x50 map (78x48 interior), %d seeds ==\n", seeds);
        std::printf("%-10s %-8s %12s %12s %10s %10s %12s\n", "coverage", "restart", "steps/cell", "border hits", "spread", "bbox", "detour");
        for (int restart : {0, 20})
            for (double cov : {0.1, 0.2, 0.3, 0.4, 0.5}) {
                double steps = 0, spread = 0, bbox = 0, detour = 0;
                for (int s = 0; s < seeds; ++s) {
                    Map m(W, H);
                    Pcg32 rng(s + 1);
                    WalkParams p;
                    p.coverage = cov, p.restartEvery = restart;
                    steps += double(randomWalk(m, rng, p)) / m.floorCount();
                    // spread：地面格子到地图中心的平均距离（归一化：中心到角为 1）；bbox：地面外接矩形占内部面积的比例
                    double sd = 0;
                    int minX = W, maxX = 0, minY = H, maxY = 0;
                    for (int y = 0; y < H; ++y)
                        for (int x = 0; x < W; ++x)
                            if (m.floor(x, y)) {
                                sd += std::hypot((x - W / 2.0) / (W / 2.0), (y - H / 2.0) / (H / 2.0)) / std::sqrt(2.0);
                                minX = std::min(minX, x), maxX = std::max(maxX, x), minY = std::min(minY, y), maxY = std::max(maxY, y);
                            }
                    spread += sd / m.floorCount();
                    bbox += double(maxX - minX + 1) * (maxY - minY + 1) / (78.0 * 48.0);
                    detour += detourRatio(m, rng, 10, 20, 10);
                }
                // 撞墙比例：单独跑一次统计（作废的步数 / 总步数）
                double hits = 0;
                for (int s = 0; s < 20; ++s) {
                    Map m(W, H);
                    Pcg32 rng(s + 1000);
                    WalkParams p;
                    p.coverage = cov, p.restartEvery = restart;
                    // 复刻 randomWalk 的循环来统计撞墙
                    const int target = int(cov * 78 * 48 + 0.5);
                    int x = W / 2, y = H / 2, carved = 1;
                    std::vector<int> cells{y * W + x};
                    m.at(x, y) = Corridor;
                    long long st = 0, bad = 0;
                    while (carved < target) {
                        if (restart > 0 && st > 0 && st % restart == 0) {
                            int c = cells[rng.below(uint32_t(cells.size()))];
                            x = c % W, y = c / W;
                        }
                        int k = int(rng.below(4)), nx = x + kDx4[k], ny = y + kDy4[k];
                        ++st;
                        if (nx < 1 || ny < 1 || nx > W - 2 || ny > H - 2) {
                            ++bad;
                            continue;
                        }
                        x = nx, y = ny;
                        if (m.at(x, y) == Wall) m.at(x, y) = Corridor, ++carved, cells.push_back(y * W + x);
                    }
                    hits += double(bad) / st;
                }
                std::printf("%-10.1f %-8d %12.1f %11.1f%% %10.3f %9.1f%% %12.2f\n", cov, restart, steps / seeds, 100 * hits / 20, spread / seeds,
                            100 * bbox / seeds, detour / seeds);
            }
    }

    // ---- 无边界的随机游走：访问过的不同格子数 S(n) 与 πn / ln n ----
    if (want(argc, argv, "sites")) {
        std::printf("\n== sites: distinct cells visited by an unbounded walk, S(n) vs pi*n/ln(n) ==\n");
        for (long long n : {1000LL, 10000LL, 100000LL, 1000000LL, 10000000LL}) {
            int reps = n >= 1000000 ? 5 : 50;
            double avg = 0;
            for (int r = 0; r < reps; ++r) {
                Pcg32 rng(r + 1);
                std::unordered_set<long long> seen;
                seen.reserve(size_t(n));
                long long x = 0, y = 0;
                seen.insert(0);
                for (long long s = 0; s < n; ++s) {
                    int k = int(rng.below(4));
                    x += kDx4[k], y += kDy4[k];
                    seen.insert((x << 32) ^ (y & 0xffffffff));
                }
                avg += double(seen.size());
            }
            avg /= reps;
            double theory = 3.14159265358979 * n / std::log(double(n));
            std::printf("  n %9lld  S(n) %10.0f  pi*n/ln n %10.0f  ratio %.3f  steps per new cell %.2f\n", n, avg, theory, avg / theory, n / avg);
        }
    }

    // ---- BSP ----
    if (want(argc, argv, "bsp")) {
        std::printf("\n== bsp: 80x50, minLeaf 8, %d seeds ==\n", seeds);
        for (int depth : {3, 4, 5, 99})
        for (bool longAxis : {true, false}) {
            double leaves = 0, aspMean = 0, asp2 = 0, aspMax = 0, roomAsp = 0;
            int nLeaves = 0;
            for (int s = 0; s < seeds; ++s) {
                Map m(W, H);
                Pcg32 rng(s + 1);
                BspParams p;
                p.longAxis = longAxis, p.maxDepth = depth;
                BspResult r = generateBsp(m, rng, p);
                leaves += r.leaves.size();
                for (int li : r.leaves) {
                    const Rect& a = r.nodes[li].area;
                    double asp = double(std::max(a.w, a.h)) / std::min(a.w, a.h);
                    aspMean += asp, asp2 += asp >= 2, aspMax = std::max(aspMax, asp), ++nLeaves;
                    const Rect& rm = r.nodes[li].room;
                    roomAsp += double(std::max(rm.w, rm.h)) / std::min(rm.w, rm.h);
                }
            }
            std::printf("  depth %2d  %-12s leaves %.1f  leaf aspect mean %.2f  >=2: %.1f%%  max %.2f  room aspect mean %.2f\n", depth, longAxis ? "long axis" : "random axis",
                        leaves / seeds, aspMean / nLeaves, 100 * asp2 / nLeaves, aspMax, roomAsp / nLeaves);
        }
        for (bool closest : {true, false}) {
            double corridorTiles = 0, crossed = 0, links = 0;
            Summary sum;
            for (int s = 0; s < seeds; ++s) {
                Map m(W, H);
                Pcg32 rng(s + 1);
                BspParams p;
                p.closest = closest;
                BspResult r = generateBsp(m, rng, p);
                std::vector<Rect> rooms;
                std::vector<int> idOf(r.nodes.size(), -1);
                for (int li : r.leaves) idOf[li] = int(rooms.size()), rooms.push_back(r.nodes[li].room);
                for (const Link& l : r.corridors) {
                    // 找出起点、终点所在的房间
                    int ra = -1, rb = -1;
                    for (size_t i = 0; i < rooms.size(); ++i) {
                        if (rooms[i].cx() == l.ax && rooms[i].cy() == l.ay) ra = int(i);
                        if (rooms[i].cx() == l.bx && rooms[i].cy() == l.by) rb = int(i);
                    }
                    crossed += roomsCrossed(rooms, ra, rb, l) > 0, links += 1;
                }
                for (uint8_t v : m.t) corridorTiles += v == Corridor;
                sum.add(m, rng);
            }
            std::printf("  %-12s corridor tiles %.1f  corridors through a third room %.1f%%\n", closest ? "closest pair" : "random pair", corridorTiles / seeds,
                        100 * crossed / links);
            sum.print(closest ? "  bsp, closest pair" : "  bsp, random pair");
        }
    }

    // ---- 随机房间：饱和与环路 ----
    if (want(argc, argv, "rooms")) {
        std::printf("\n== rooms: 80x50, size 4..11, spacing 2, %d seeds ==\n", seeds);
        for (int att : {10, 25, 50, 100, 200, 300, 400, 1000, 3000}) {
            double rooms = 0, area = 0;
            for (int s = 0; s < seeds; ++s) {
                Map m(W, H);
                Pcg32 rng(s + 1);
                RoomsParams p;
                p.attempts = att, p.loops = 0;
                RoomsResult r = generateRooms(m, rng, p);
                rooms += r.rooms.size();
                for (const Rect& x : r.rooms) area += x.w * x.h;
            }
            std::printf("  attempts %5d  rooms %5.1f  room area %5.1f%%\n", att, rooms / seeds, 100 * area / seeds / (78.0 * 48));
        }
        for (double loops : {0.0, 0.1, 0.2, 0.5, 1.0}) {
            Summary sum;
            double crossed = 0, links = 0;
            for (int s = 0; s < seeds; ++s) {
                Map m(W, H);
                Pcg32 rng(s + 1);
                RoomsParams p;
                p.loops = loops;
                RoomsResult r = generateRooms(m, rng, p);
                for (const auto* list : {&r.tree, &r.extra})
                    for (const Edge& e : *list) {
                        // 方向不影响"穿过哪些房间"的统计结论，这里两种拐法都算一次取平均
                        for (bool hf : {true, false}) {
                            Link l{r.rooms[e.a].cx(), r.rooms[e.a].cy(), r.rooms[e.b].cx(), r.rooms[e.b].cy(), hf};
                            crossed += 0.5 * (roomsCrossed(r.rooms, e.a, e.b, l) > 0);
                        }
                        links += 1;
                    }
                sum.add(m, rng);
            }
            char nm[64];
            std::snprintf(nm, sizeof nm, "rooms, loops %.1f", loops);
            sum.print(nm);
            std::printf("  %-26s corridors through a third room %.1f%%\n", "", 100 * crossed / links);
        }
        Summary walk;
        for (int s = 0; s < seeds; ++s) {
            Map m(W, H);
            Pcg32 rng(s + 1);
            WalkParams p;
            p.coverage = 0.35;
            randomWalk(m, rng, p);
            walk.add(m, rng);
        }
        walk.print("walk, coverage 0.35");
    }

    // ---- 两次 BFS 求直径 ----
    if (want(argc, argv, "diameter")) {
        std::printf("\n== diameter: double sweep vs exact (80x50, 100 seeds each, sweep from a random floor cell) ==\n");
        const char* names[] = {"walk 0.35", "bsp", "rooms loops 0", "rooms loops 0.5"};
        for (int g = 0; g < 4; ++g) {
            int exact = 0;
            double ratio = 0, worst = 1, tExact = 0, tSweep = 0;
            for (int s = 0; s < 100; ++s) {
                Map m(W, H);
                Pcg32 rng(s + 1);
                if (g == 0) {
                    WalkParams p;
                    p.coverage = 0.35;
                    randomWalk(m, rng, p);
                } else if (g == 1) {
                    generateBsp(m, rng, {});
                } else {
                    RoomsParams p;
                    p.loops = g == 2 ? 0 : 0.5;
                    generateRooms(m, rng, p);
                }
                std::vector<int> cells;
                for (int i = 0; i < int(m.t.size()); ++i)
                    if (m.t[i] != Wall) cells.push_back(i);
                auto t0 = Clock::now();
                Diameter e = exactDiameter(m);
                auto t1 = Clock::now();
                Diameter d = doubleSweep(m, cells[rng.below(uint32_t(cells.size()))]);
                auto t2 = Clock::now();
                tExact += std::chrono::duration<double, std::milli>(t1 - t0).count();
                tSweep += std::chrono::duration<double, std::milli>(t2 - t1).count();
                double r = double(d.length) / e.length;
                exact += d.length == e.length, ratio += r, worst = std::min(worst, r);
            }
            std::printf("  %-16s exact %3d%%  mean ratio %.4f  worst %.3f  time exact %.1f ms  sweep %.3f ms\n", names[g], exact, ratio / 100, worst,
                        tExact / 100, tSweep / 100);
        }
    }

    // ---- 开销 ----
    if (want(argc, argv, "speed")) {
        std::printf("\n== speed: one 80x50 map (min over 200 seeds) ==\n");
        auto timeIt = [&](const char* name, auto&& gen) {
            double best = 1e9;
            for (int s = 0; s < 200; ++s) {
                Map m(W, H);
                Pcg32 rng(s + 1);
                auto t0 = Clock::now();
                gen(m, rng);
                best = std::min(best, std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
            }
            std::printf("  %-20s %8.1f us\n", name, best);
        };
        timeIt("walk 0.35", [](Map& m, Pcg32& r) {
            WalkParams p;
            p.coverage = 0.35;
            randomWalk(m, r, p);
        });
        timeIt("bsp", [](Map& m, Pcg32& r) { generateBsp(m, r, {}); });
        timeIt("rooms (300 attempts)", [](Map& m, Pcg32& r) { generateRooms(m, r, {}); });
    }
}
