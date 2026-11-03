#include "random.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "check.h"

using namespace ga;

int main() {
    // PCG32 参考输出（pcg32-demo：seed 42, stream 54）
    {
        Pcg32 rng(42u, 54u);
        uint32_t expected[] = {0xa15c02b7, 0x7b47f409, 0xba1d3330, 0x83d2f293, 0xbfa4784b, 0xcbed606e};
        for (uint32_t e : expected) CHECK(rng.next() == e);
    }
    // 相同种子 -> 相同序列；不同种子 -> 不同序列
    {
        Pcg32 a(7), b(7), c(8);
        bool same = true, differ = false;
        for (int i = 0; i < 100; ++i) {
            uint32_t x = a.next(), y = b.next(), z = c.next();
            same = same && x == y;
            differ = differ || x != z;
        }
        CHECK(same);
        CHECK(differ);
    }
    // below / range / nextFloat 的取值范围与大致均匀
    {
        Pcg32 rng(1);
        int counts[6] = {};
        for (int i = 0; i < 60000; ++i) {
            int d = rng.range(1, 6);
            CHECK(d >= 1 && d <= 6);
            counts[d - 1]++;
            float f = rng.nextFloat();
            CHECK(f >= 0.0f && f < 1.0f);
        }
        for (int c : counts) CHECK(std::abs(c - 10000) < 500);
    }
    // Fisher–Yates：结果是原数组的一个排列，且每张牌落在每个位置的频率接近 1/n
    {
        Pcg32 rng(3);
        const int n = 5, trials = 50000;
        int freq[n][n] = {};
        for (int t = 0; t < trials; ++t) {
            std::vector<int> v(n);
            std::iota(v.begin(), v.end(), 0);
            shuffle(v, rng);
            std::vector<int> sorted = v;
            std::sort(sorted.begin(), sorted.end());
            CHECK(sorted == std::vector<int>({0, 1, 2, 3, 4}));
            for (int pos = 0; pos < n; ++pos) freq[v[pos]][pos]++;
        }
        for (auto& row : freq)
            for (int f : row) CHECK(std::abs(f - trials / n) < trials / n * 0.05);
    }
    // 朴素洗牌有偏：n = 3 时，"1 留在位置 0" 的概率不是 1/3
    {
        Pcg32 rng(4);
        const int trials = 90000;
        int count = 0;
        for (int t = 0; t < trials; ++t) {
            std::vector<int> v{0, 1, 2};
            naiveShuffle(v, rng);
            count += v[0] == 1;
        }
        // 理论值 10/27 ≈ 0.370，而不是 0.333
        CHECK(std::abs(count / double(trials) - 10.0 / 27.0) < 0.01);
    }
    // 加权随机
    {
        Pcg32 rng(5);
        WeightedTable table({60, 25, 10, 4, 1});
        CHECK(std::abs(table.probability(1) - 0.25) < 1e-12);
        int counts[5] = {};
        const int trials = 200000;
        for (int i = 0; i < trials; ++i) counts[table.pick(rng)]++;
        for (int i = 0; i < 5; ++i) CHECK(std::abs(counts[i] / double(trials) - table.probability(i)) < 0.005);
    }
    // 洗牌袋：每轮 4 次抽取恰好覆盖 4 种结果
    {
        Pcg32 rng(6);
        ShuffleBag<int> bag({0, 1, 2, 3});
        for (int round = 0; round < 50; ++round) {
            int seen[4] = {};
            for (int i = 0; i < 4; ++i) seen[bag.next(rng)]++;
            for (int s : seen) CHECK(s == 1);
        }
    }
    // PRD：25% 对应的常数 C ≈ 0.0847，实测平均概率仍是 25%，且最长"干旱"明显更短
    {
        CHECK(std::abs(prdConstant(0.25) - 0.0847) < 5e-4);
        CHECK(std::abs(prdAverageProbability(prdConstant(0.4)) - 0.4) < 1e-9);

        Pcg32 rng(9);
        PrdRoll prd(0.25);
        int hits = 0, drought = 0, longestPrd = 0, longestIndependent = 0, d2 = 0, hits2 = 0;
        const int trials = 200000;
        for (int i = 0; i < trials; ++i) {
            if (prd.roll(rng)) { ++hits; drought = 0; } else longestPrd = std::max(longestPrd, ++drought);
            if (rng.chance(0.25)) { ++hits2; d2 = 0; } else longestIndependent = std::max(longestIndependent, ++d2);
        }
        CHECK(std::abs(hits / double(trials) - 0.25) < 0.005);
        CHECK(longestPrd <= 11);  // 第 12 次时 12·C > 1，必然触发
        CHECK(longestIndependent > 25);
    }

    return report("random_test");
}
