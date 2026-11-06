#include "goap.h"

#include <cmath>
#include <cstdio>
#include <limits>

#include "../../foundations/random/random.h"
#include "camp.h"
#include "check.h"

using namespace ga;

// 暴力对照：在全部 2^n 个状态上反复松弛（Bellman–Ford），得到到达每个状态的最小代价
static float bruteForce(const std::vector<GoapAction>& actions, int n, uint64_t start, Facts goal) {
    size_t count = size_t(1) << n;
    std::vector<float> dist(count, std::numeric_limits<float>::infinity());
    dist[start] = 0;
    for (bool changed = true; changed;) {
        changed = false;
        for (uint64_t s = 0; s < count; ++s) {
            if (!std::isfinite(dist[s])) continue;
            for (const GoapAction& a : actions)
                if (a.pre.satisfiedBy(s)) {
                    uint64_t t = a.eff.applyTo(s);
                    if (dist[s] + a.cost < dist[t] - 1e-6f) dist[t] = dist[s] + a.cost, changed = true;
                }
        }
    }
    float best = std::numeric_limits<float>::infinity();
    for (uint64_t s = 0; s < count; ++s)
        if (goal.satisfiedBy(s)) best = std::min(best, dist[s]);
    return best;
}

static bool reaches(const std::vector<GoapAction>& actions, uint64_t start, Facts goal, const Plan& p) {
    bool ok = false;
    uint64_t end = simulate(actions, start, p.actions, &ok);
    float cost = 0;
    for (int a : p.actions) cost += actions[a].cost;
    return ok && goal.satisfiedBy(end) && std::abs(cost - p.cost) < 1e-3f;
}

int main() {
    // ---- 野外求生 ----
    {
        auto acts = camp::plain(camp::actions());
        // 工具都在：斧头砍柴 + 火柴点火（5），钓鱼烤鱼（5）
        Plan p = planForward(acts, camp::start(true, true, true), camp::goal());
        CHECK(p.found && std::abs(p.cost - 10) < 1e-4f && reaches(acts, camp::start(true, true, true), camp::goal(), p));
        std::printf("camp, all tools:");
        for (int a : p.actions) std::printf(" [%s]", acts[a].name.c_str());
        std::printf("  cost %.0f\n", p.cost);
        // 什么工具都没有：捡树枝、钻木取火（14），采浆果（9）
        Plan q = planForward(acts, camp::start(false, false, false), camp::goal());
        CHECK(q.found && std::abs(q.cost - 23) < 1e-4f);
        // 正向、反向、暴力三者在 8 种工具组合下代价都相同
        for (int mask = 0; mask < 8; ++mask) {
            uint64_t s = camp::start(mask & 1, mask & 2, mask & 4);
            float ref = bruteForce(acts, camp::kFactCount, s, camp::goal());
            Plan f = planForward(acts, s, camp::goal()), b = planBackward(acts, s, camp::goal());
            CHECK(f.found && b.found && std::abs(f.cost - ref) < 1e-4f && std::abs(b.cost - ref) < 1e-4f);
            CHECK(reaches(acts, s, camp::goal(), f) && reaches(acts, s, camp::goal(), b));
        }
    }

    // ---- "未满足事实数"不可采纳：一个动作同时满足多个目标时，它会高估，A* 可能返回更贵的计划 ----
    {
        // 最优：prepare（1）→ finish（1，一次满足三个目标），共 2；另一条路：三个各满足一个目标的动作，共 3
        std::vector<GoapAction> acts = {
            {"prepare", 1, Facts{}, Facts{}.set(0)},
            {"finish", 1, Facts{}.set(0), Facts{}.set(1).set(2).set(3)},
            {"one", 1, Facts{}, Facts{}.set(1)},
            {"two", 1, Facts{}, Facts{}.set(2)},
            {"three", 1, Facts{}, Facts{}.set(3)},
        };
        Facts goal = Facts{}.set(1).set(2).set(3);
        CHECK(std::abs(planForward(acts, 0, goal, Heuristic::Count).cost - 3) < 1e-4f);       // 被高估误导
        CHECK(std::abs(planForward(acts, 0, goal, Heuristic::Admissible).cost - 2) < 1e-4f);  // ceil(3 / 3) × 1 = 1，不高估
    }

    // ---- 随机领域：与暴力解对照 ----
    Pcg32 rng(19);
    int trials = 0, reachable = 0, countSuboptimal = 0;
    double countWorst = 0;
    for (int t = 0; t < 3000; ++t) {
        int n = 4 + rng.below(8);  // 4 ~ 11 个事实
        std::vector<GoapAction> acts;
        int na = 3 + rng.below(12);
        for (int k = 0; k < na; ++k) {
            GoapAction a{"a", float(1 + rng.below(5)), {}, {}};
            for (int f = 0; f < n; ++f)
                if (rng.below(4) == 0) a.pre.set(f, rng.below(2));
            int effects = 1 + rng.below(4);
            for (int e = 0; e < effects; ++e) a.eff.set(rng.below(n), rng.below(3) != 0);
            acts.push_back(a);
        }
        uint64_t start = rng.next() & ((uint64_t(1) << n) - 1);
        Facts goal;
        int gcount = 1 + rng.below(5);
        for (int g = 0; g < gcount; ++g) goal.set(rng.below(n), rng.below(2));
        float ref = bruteForce(acts, n, start, goal);
        ++trials;
        Plan dijkstra = planForward(acts, start, goal, Heuristic::None);
        Plan fwd = planForward(acts, start, goal, Heuristic::Admissible);
        Plan bwd = planBackward(acts, start, goal, Heuristic::Admissible);
        Plan bwd0 = planBackward(acts, start, goal, Heuristic::None);
        Plan cnt = planForward(acts, start, goal, Heuristic::Count);
        bool can = std::isfinite(ref);
        CHECK(dijkstra.found == can && fwd.found == can && bwd.found == can && bwd0.found == can && cnt.found == can);
        if (!can) continue;
        ++reachable;
        for (const Plan* p : {&dijkstra, &fwd, &bwd, &bwd0}) {
            CHECK(std::abs(p->cost - ref) < 1e-3f);  // 最优
            CHECK(reaches(acts, start, goal, *p));   // 可执行，并且真的达成了目标
        }
        CHECK(reaches(acts, start, goal, cnt) && cnt.cost >= ref - 1e-3f);
        if (cnt.cost > ref + 1e-3f) ++countSuboptimal, countWorst = std::max(countWorst, double(cnt.cost / ref));
    }
    std::printf("random domains: %d trials, %d reachable; 'count' heuristic suboptimal in %d (worst %.2fx)\n", trials, reachable,
                countSuboptimal, countWorst);
    CHECK(reachable > 1200);
    return report("goap_test");
}
