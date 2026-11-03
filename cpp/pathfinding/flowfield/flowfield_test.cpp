#include "flowfield.h"

#include <cmath>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

// 暴力对照：反复松弛直到不再变化
static std::vector<int> bruteForce(const CostGrid& g, Cell goal) {
    std::vector<int> d(g.size(), kFlowUnreached);
    d[g.index(goal)] = 0;
    for (bool changed = true; changed;) {
        changed = false;
        for (int i = 0; i < g.size(); ++i) {
            if (d[i] == kFlowUnreached) continue;
            forEachNeighbor8(g, g.cellAt(i), [&](Cell n, int step) {
                if (d[i] + step < d[g.index(n)]) {
                    d[g.index(n)] = d[i] + step;
                    changed = true;
                }
            });
        }
    }
    return d;
}

int main() {
    Pcg32 rng(31);
    for (int round = 0; round < 200; ++round) {
        CostGrid g(20, 12);
        for (int i = 0; i < g.size(); ++i) {
            uint32_t r = rng.below(10);
            if (r < 2) g.setWall(g.cellAt(i), true);
            else if (r < 3) g.setCost(g.cellAt(i), 3);
            else if (r < 4) g.setCost(g.cellAt(i), 6);
        }
        Cell goal{static_cast<int>(rng.below(20)), static_cast<int>(rng.below(12))};
        g.setWall(goal, false);

        auto field = integrationField(g, {goal});
        CHECK(field == bruteForce(g, goal));  // 积分场 = 精确的最小代价
        auto flow = flowField(g, field);

        for (int i = 0; i < g.size(); ++i) {
            Cell c = g.cellAt(i);
            if (field[i] == kFlowUnreached) {
                CHECK(flow[i] == (Vec2{0, 0}));
                continue;
            }
            // 每个箭头都指向一个合法的相邻格子，且积分值严格下降：顺着走一定会到达目标
            Cell cur = c;
            int steps = 0;
            while (!(cur == goal) && steps < g.size()) {
                Vec2 d = flow[g.index(cur)];
                Cell next{cur.x + static_cast<int>(std::lround(d.x)), cur.y + static_cast<int>(std::lround(d.y))};
                bool legal = false;
                forEachNeighbor8(g, cur, [&](Cell n, int) { legal = legal || n == next; });
                CHECK(legal);
                if (!legal) break;
                CHECK(field[g.index(next)] < field[g.index(cur)]);
                cur = next;
                ++steps;
            }
            CHECK(cur == goal);
            // 在格子中心采样，插值结果与该格子的方向一致
            if (!(c == goal)) {
                Vec2 s = sampleFlow(g, flow, {c.x + 0.5f, c.y + 0.5f});
                CHECK(length(s - flow[i]) < 1e-4f);
            }
        }
    }
    // 人群：空地上 120 个单位，模拟 12 秒后几乎全部聚集到目标附近，且没有单位嵌进墙里
    {
        CostGrid g(24, 14);
        for (int y = 2; y <= 11; ++y) g.setWall({12, y}, true);
        Cell goal{21, 7};
        auto flow = flowField(g, integrationField(g, {goal}));
        std::vector<Agent> agents;
        for (int i = 0; i < 120; ++i) {
            agents.push_back({{1.0f + (i % 6) * 0.7f, 1.0f + (i / 6) * 0.6f}, {0, 0}});
        }
        CrowdParams prm;
        for (int t = 0; t < 60 * 12; ++t) stepCrowd(g, flow, goal, agents, prm, 1.0f / 60);
        int near = 0;
        for (const Agent& a : agents) {
            if (distance(a.pos, {goal.x + 0.5f, goal.y + 0.5f}) < 3.5f) ++near;
            Cell c{static_cast<int>(std::floor(a.pos.x)), static_cast<int>(std::floor(a.pos.y))};
            CHECK(g.passable(c));
        }
        CHECK(near >= 114);
    }
    return report("flowfield_test");
}
