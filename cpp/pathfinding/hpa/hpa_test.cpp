#include "hpa.h"

#include <cstdio>

#include "../../foundations/random/random.h"
#include "../dijkstra/dijkstra.h"
#include "check.h"

using namespace ga;

static bool validPath(const CostGrid& g, const std::vector<Cell>& p, Cell s, Cell t) {
    if (p.empty() || !(p.front() == s) || !(p.back() == t)) return false;
    for (size_t i = 0; i < p.size(); ++i) {
        if (!g.passable(p[i])) return false;
        if (i > 0 && std::abs(p[i].x - p[i - 1].x) + std::abs(p[i].y - p[i - 1].y) != 1) return false;
    }
    return true;
}

int main() {
    // 簇内 Dijkstra：正向与反向的代价在对称地图上相同，不对称时按"进入格子付费"区分
    {
        CostGrid g(3, 1);
        g.setCost({2, 0}, 5);
        Rect r{0, 0, 3, 1};
        CHECK(boundedDijkstra(g, r, {0, 0}).cost[2] == 1 + 5);               // 从左走到右
        CHECK(boundedDijkstra(g, r, {0, 0}, true).cost[2] == 1 + 1);         // 从右走到左（反向搜索）
    }
    Pcg32 rng(77);
    int queries = 0, optimal = 0;
    double excess = 0, worst = 0;
    int bucketN[3] = {}, bucketOpt[3] = {};
    double bucketEx[3] = {}, bucketWorst[3] = {};
    for (int round = 0; round < 150; ++round) {
        int w = 10 + static_cast<int>(rng.below(30)), h = 10 + static_cast<int>(rng.below(20));
        CostGrid g(w, h);
        uint32_t density = rng.below(30);
        for (int i = 0; i < g.size(); ++i) {
            uint32_t r = rng.below(100);
            if (r < density) g.setWall(g.cellAt(i), true);
            else if (r < density + 15) g.setCost(g.cellAt(i), 3);
            else if (r < density + 22) g.setCost(g.cellAt(i), 6);
        }
        HpaGraph hpa;
        hpa.build(g, 4 + static_cast<int>(rng.below(7)));
        for (int q = 0; q < 20; ++q) {
            Cell s{static_cast<int>(rng.below(w)), static_cast<int>(rng.below(h))};
            Cell t{static_cast<int>(rng.below(w)), static_cast<int>(rng.below(h))};
            if (!g.passable(s) || !g.passable(t) || s == t) continue;
            auto truth = dijkstraPath(g, s, t);
            auto path = hpa.findPath(s, t);
            CHECK(path.empty() == truth.empty());  // 完备：有路就一定能找到
            if (truth.empty()) continue;
            CHECK(validPath(g, path, s, t));
            int best = pathCost(g, truth), got = pathCost(g, path);
            CHECK(got >= best);
            ++queries;
            optimal += got == best;
            double e = static_cast<double>(got) / best - 1;
            excess += e;
            worst = std::max(worst, e);
            int b = best < 20 ? 0 : best < 60 ? 1 : 2;
            ++bucketN[b];
            bucketOpt[b] += got == best;
            bucketEx[b] += e;
            bucketWorst[b] = std::max(bucketWorst[b], e);
        }
    }
    std::printf("hpa: optimal %d / %d, mean excess %.2f%%, worst %.1f%%\n", optimal, queries, 100 * excess / queries,
                100 * worst);
    const char* names[3] = {"cost < 20", "20..59", ">= 60"};
    for (int b = 0; b < 3; ++b) {
        std::printf("  %-10s n=%4d optimal %5.1f%%  mean excess %5.2f%%  worst %6.1f%%\n", names[b], bucketN[b],
                    100.0 * bucketOpt[b] / bucketN[b], 100 * bucketEx[b] / bucketN[b], 100 * bucketWorst[b]);
    }
    return report("hpa_test");
}
