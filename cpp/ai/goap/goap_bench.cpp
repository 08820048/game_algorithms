// GOAP 规划器：正向与反向搜索、三种启发函数（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/goap/goap_bench.cpp -o goap_bench && ./goap_bench
#include <chrono>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "camp.h"
#include "goap.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

struct Row {
    double expanded = 0, us = 0, cost = 0;
    int failed = 0;
};

template <class F>
static Row measure(int reps, F&& plan) {
    Row r;
    auto t0 = Clock::now();
    for (int i = 0; i < reps; ++i) {
        Plan p = plan(i);
        r.expanded += p.expanded, r.cost += p.cost, r.failed += !p.found;
    }
    r.us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / reps;
    r.expanded /= reps, r.cost /= reps;
    return r;
}

// 合成的"合成树"领域：最底层是采集动作；每个高层物品有两种配方，各消耗两个低一层的物品。
// 另外加入 distractors 个与目标无关的动作（各自切换一个无关的事实），正向搜索会把它们也展开
struct Domain {
    std::vector<GoapAction> actions;
    uint64_t start = 0;
    Facts goal;
};
static Domain crafting(int levels, int width, int distractors, uint64_t seed) {
    Pcg32 r(seed);
    Domain d;
    auto fact = [&](int level, int i) { return level * width + i; };
    for (int i = 0; i < width; ++i) d.actions.push_back({"gather", float(1 + r.below(3)), Facts{}, Facts{}.set(fact(0, i))});
    for (int l = 1; l < levels; ++l)
        for (int i = 0; i < width; ++i)
            for (int recipe = 0; recipe < 2; ++recipe) {
                int a = r.below(width), b = (a + 1 + r.below(width - 1)) % width;
                GoapAction act{"craft", float(1 + r.below(3)), Facts{}.set(fact(l - 1, a)).set(fact(l - 1, b)),
                               Facts{}.set(fact(l, i)).set(fact(l - 1, a), false).set(fact(l - 1, b), false)};
                d.actions.push_back(act);
            }
    int base = levels * width;
    for (int k = 0; k < distractors; ++k) {
        d.actions.push_back({"wander on", 1, Facts{}.set(base + k, false), Facts{}.set(base + k)});
        d.actions.push_back({"wander off", 1, Facts{}.set(base + k), Facts{}.set(base + k, false)});
    }
    d.goal.set(fact(levels - 1, 0));
    return d;
}

int main() {
    const char* hName[] = {"none (Dijkstra)", "count", "admissible"};

    std::printf("1) camp domain, averaged over the 8 tool combinations\n");
    std::printf("  %-10s %-16s %10s %10s %10s\n", "search", "heuristic", "expanded", "us/plan", "cost");
    auto acts = camp::plain(camp::actions());
    measure(4000, [&](int i) { return planForward(acts, camp::start(i & 1, i & 2, i & 4), camp::goal()); });  // 预热
    for (int dir = 0; dir < 2; ++dir)
        for (int h = 0; h < 3; ++h) {
            Row r = measure(8 * 500, [&](int i) {
                int m = i % 8;
                uint64_t s = camp::start(m & 1, m & 2, m & 4);
                return dir == 0 ? planForward(acts, s, camp::goal(), Heuristic(h)) : planBackward(acts, s, camp::goal(), Heuristic(h));
            });
            std::printf("  %-10s %-16s %10.1f %10.2f %10.2f\n", dir == 0 ? "forward" : "backward", hName[h], r.expanded, r.us, r.cost);
        }

    std::printf("\n2) crafting tree (4 levels x 6 items, 2 recipes each), with irrelevant actions; 50 random domains each\n");
    std::printf("  %-12s %-10s %-16s %12s %10s %8s\n", "distractors", "search", "heuristic", "expanded", "us/plan", "failed");
    for (int distractors : {0, 4, 8, 12}) {
        for (int dir = 0; dir < 2; ++dir)
            for (int h : {0, 2}) {
                Row r = measure(50, [&](int i) {
                    Domain d = crafting(4, 6, distractors, 100 + i);
                    return dir == 0 ? planForward(d.actions, d.start, d.goal, Heuristic(h), 300000)
                                    : planBackward(d.actions, d.start, d.goal, Heuristic(h), 300000);
                });
                std::printf("  %-12d %-10s %-16s %12.0f %10.0f %8d\n", distractors, dir == 0 ? "forward" : "backward", hName[h], r.expanded, r.us,
                            r.failed);
            }
    }
}
