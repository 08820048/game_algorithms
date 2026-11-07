// 效用 AI：组合方式的偏差、选择器的开销、惯性与抖动、士兵场景的消融实验（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/utility/utility_bench.cpp -o utility_bench && ./utility_bench
#include <chrono>
#include <cstdio>
#include <cstring>

#include "../../foundations/random/random.h"
#include "soldier.h"
#include "utility.h"

using namespace ga;
using namespace ga::utility;
using Clock = std::chrono::steady_clock;

static const char* modeName[] = {"product", "compensated", "geometric"};

// 计时：重复 25 轮取最快的一轮，减少噪声。返回每次决策的纳秒数，evals 为平均计算的考量个数
template <size_t N>
static double timeChoose(const std::vector<Action>& acts, const std::vector<int>& order, const std::vector<std::array<float, N>>& inputs,
                         const Options& opt, double* evals, int* checksum) {
    double best = 1e30;
    for (int round = 0; round < 25; ++round) {
        long long e = 0;
        int sum = 0;
        auto t0 = Clock::now();
        for (int rep = 0; rep < 10; ++rep)
            for (auto& in : inputs) {
                Choice c = choose(acts, order, in.data(), opt);
                e += c.evaluated, sum += c.action;
            }
        best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / (10 * inputs.size()));
        *evals = double(e) / (10 * inputs.size()), *checksum = sum;
    }
    return best;
}

// 士兵打 10 分钟，统计切换次数、短命的动作（持续不到 0.2 秒）、被击倒与击杀次数
struct SimResult {
    double switches = 0, flickers = 0, downs = 0, kills = 0;
};
static SimResult simulate(const std::vector<Action>& acts, Options opt, float decideEvery, int seeds) {
    using namespace ga::soldier;
    auto order = orderByWeight(acts);
    SimResult r;
    const float dt = 1 / 30.0f;
    for (int seed = 1; seed <= seeds; ++seed) {
        World w;
        Pcg32 rng(seed);
        float sinceDecision = 1e9f, sinceSwitch = 0;
        for (int i = 0; i < 30 * 600; ++i) {
            moveEnemy(w, dt, rng);
            if ((sinceDecision += dt) >= decideEvery - 1e-4f) {
                sinceDecision = 0;
                auto in = inputs(w);
                opt.current = w.action;
                int next = choose(acts, order, in.data(), opt).action;
                if (next != w.action && w.action >= 0) {
                    ++r.switches;
                    if (sinceSwitch < 0.2f) ++r.flickers;
                    sinceSwitch = 0;
                }
                w.action = next;
            }
            sinceSwitch += dt;
            act(w, w.action, dt, rng);
        }
        r.downs += w.downs, r.kills += w.kills;
    }
    r.switches /= seeds * 10.0, r.flickers /= seeds * 10.0, r.downs /= seeds, r.kills /= seeds;  // 每分钟 / 每局
    return r;
}

int main() {
    Pcg32 rng(5);

    // ---- 1) 考量个数带来的偏差 ----
    std::printf("1) score vs number of considerations\n");
    std::printf("  %-3s %28s   %28s\n", "n", "all considerations = 0.8", "uniform random scores (mean)");
    std::printf("  %-3s %9s %9s %9s   %9s %9s %9s\n", "", "product", "compens.", "geomean", "product", "compens.", "geomean");
    for (int n = 1; n <= 8; ++n) {
        Action a{"a", 1, {}};
        for (int k = 0; k < n; ++k) a.considerations.push_back({"c", k, 0, 1, linear()});
        float same[8];
        std::fill(same, same + 8, 0.8f);
        double mean[3] = {};
        const int trials = 200000;
        float raw[8];
        for (int t = 0; t < trials; ++t) {
            for (float& v : raw) v = rng.nextFloat();
            for (int m = 0; m < 3; ++m) mean[m] += score(a, raw, Combine(m));
        }
        std::printf("  %-3d %9.3f %9.3f %9.3f   %9.3f %9.3f %9.3f\n", n, score(a, same, Combine::Product), score(a, same, Combine::Compensated),
                    score(a, same, Combine::GeometricMean), mean[0] / trials, mean[1] / trials, mean[2] / trials);
    }

    // ---- 2) 选择器的开销：剪枝 vs 全算 ----
    std::printf("\n2) decision cost (random tables, 2-6 considerations per action, weights 1/2/3; 2000 decisions each)\n");
    std::printf("  %-8s %-12s %12s %12s %14s %14s\n", "actions", "combine", "full ns", "pruned ns", "full evals", "pruned evals");
    for (int count : {8, 32, 128, 512}) {
        std::vector<Action> acts(count);
        for (Action& a : acts) {
            a.name = "a";
            a.weight = rng.below(10) == 0 ? 3.0f : rng.below(4) == 0 ? 2.0f : 1.0f;  // 少数高优先级动作
            int n = 2 + rng.below(5);
            for (int k = 0; k < n; ++k) {
                Curve c = rng.below(2) ? logistic(rng.nextFloat(), (rng.nextFloat() * 2 - 1) * 20) : power(0.5f + rng.nextFloat() * 2);
                a.considerations.push_back({"c", int(rng.below(8)), 0, 1, c});
            }
        }
        auto order = orderByWeight(acts);
        std::vector<std::array<float, 8>> inputs(2000);
        for (auto& in : inputs)
            for (float& v : in) v = rng.nextFloat();
        for (int m : {0, 1}) {
            double ns[2], evals[2];
            int checksum[2] = {};
            for (int pruned = 0; pruned < 2; ++pruned) {
                Options opt;
                opt.mode = Combine(m);
                opt.earlyExit = pruned;
                ns[pruned] = timeChoose(acts, order, inputs, opt, &evals[pruned], &checksum[pruned]);
            }
            if (checksum[0] != checksum[1]) std::printf("  MISMATCH\n");
            std::printf("  %-8d %-12s %12.0f %12.0f %14.1f %14.1f\n", count, modeName[m], ns[0], ns[1], evals[0], evals[1]);
        }
    }
    {
        auto acts = soldier::actions();
        auto order = orderByWeight(acts);
        std::vector<std::array<float, soldier::kInputCount>> inputs(4000);
        for (auto& in : inputs) in = {rng.nextFloat() * 100, float(rng.below(9)), rng.nextFloat() * 500, rng.nextFloat() * 500, rng.nextFloat() * 200};
        for (int pruned = 0; pruned < 2; ++pruned) {
            Options opt;
            opt.earlyExit = pruned;
            double evals;
            int sum;
            double ns = timeChoose(acts, order, inputs, opt, &evals, &sum);
            std::printf("  soldier (6 actions, 15 considerations) %s: %.0f ns, %.1f evals  [%d]\n", pruned ? "pruned" : "full  ", ns, evals, sum % 7);
        }
    }

    // ---- 3) 惯性与决策间隔：抖动 ----
    const int seeds = 12;
    auto acts = soldier::actions();
    std::printf("\n3) soldier, %d seeds x 10 minutes, decide every frame (30 Hz) unless noted\n", seeds);
    std::printf("  %-26s %12s %14s %10s %10s\n", "setting", "switch/min", "flicker/min", "downs", "kills");
    for (float m : {0.0f, 0.05f, 0.1f, 0.25f, 0.5f, 1.0f}) {
        char label[40];
        std::snprintf(label, sizeof label, "momentum %.2f", m);
        SimResult r = simulate(acts, Options{Combine::Compensated, -1, m}, 0, seeds);
        std::printf("  %-26s %12.1f %14.2f %10.2f %10.1f\n", label, r.switches, r.flickers, r.downs, r.kills);
    }
    for (float every : {0.25f, 0.5f, 1.0f}) {
        char label[40];
        std::snprintf(label, sizeof label, "momentum 0, every %.2fs", every);
        SimResult r = simulate(acts, Options{Combine::Compensated, -1, 0}, every, seeds);
        std::printf("  %-26s %12.1f %14.2f %10.2f %10.1f\n", label, r.switches, r.flickers, r.downs, r.kills);
    }

    // ---- 4) 消融：去掉某个考量、换一种组合方式 ----
    std::printf("\n4) ablation (momentum 0.1)\n");
    std::printf("  %-34s %12s %14s %10s %10s\n", "variant", "switch/min", "flicker/min", "downs", "kills");
    auto without = [&](const char* action, const char* consideration) {
        auto v = soldier::actions();
        for (Action& a : v)
            if (!std::strcmp(a.name, action))
                std::erase_if(a.considerations, [&](const Consideration& c) { return !std::strcmp(c.name, consideration); });
        return v;
    };
    struct Variant {
        const char* label;
        std::vector<Action> acts;
        Combine mode;
    };
    std::vector<Variant> variants = {
        {"full model (compensated)", acts, Combine::Compensated},
        {"heal without 'safe route'", without("heal", "safe route"), Combine::Compensated},
        {"flee without 'no medkit'", without("flee", "no medkit"), Combine::Compensated},
        {"both removed", without("flee", "no medkit"), Combine::Compensated},
        {"product (no compensation)", acts, Combine::Product},
        {"geometric mean", acts, Combine::GeometricMean},
    };
    for (Action& a : variants[3].acts)
        if (!std::strcmp(a.name, "heal")) std::erase_if(a.considerations, [](const Consideration& c) { return !std::strcmp(c.name, "safe route"); });
    for (const Variant& v : variants) {
        SimResult r = simulate(v.acts, Options{v.mode, -1, 0.1f}, 0, seeds);
        std::printf("  %-34s %12.1f %14.2f %10.2f %10.1f\n", v.label, r.switches, r.flickers, r.downs, r.kills);
    }
}
