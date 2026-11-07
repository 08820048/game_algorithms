#include "utility.h"

#include <cmath>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "check.h"
#include "soldier.h"

using namespace ga;
using namespace ga::utility;

static bool near(float a, float b, float eps = 1e-4f) { return std::abs(a - b) < eps; }

static Curve randomCurve(Pcg32& rng) {
    Curve c;
    c.type = Curve::Type(rng.below(3));
    c.m = (rng.nextFloat() * 2 - 1) * (c.type == Curve::Logistic ? 30 : 2);
    c.k = c.type == Curve::Power ? 0.2f + rng.nextFloat() * 4 : 0.5f + rng.nextFloat() * 0.5f;
    c.b = rng.nextFloat() * 1.2f - 0.3f;
    c.c = rng.nextFloat();
    return c;
}

int main() {
    // ---- 响应曲线 ----
    CHECK(near(linear()(0.3f), 0.3f) && near(inverseLinear()(0.3f), 0.7f));
    CHECK(near(power(2)(0.5f), 0.25f) && near(inversePower(2)(0.5f), 0.75f));
    CHECK(near(logistic(0.5f, 10)(0.5f), 0.5f) && logistic(0.5f, 10)(0.9f) > 0.98f && logistic(0.5f, -10)(0.9f) < 0.02f);
    CHECK(linear(2)(0.9f) == 1 && linear(-1)(0.5f) == 0);  // 截断到 [0, 1]
    {
        Consideration c{"x", 0, 100, 0, linear()};  // lo > hi：方向相反
        float raw[] = {25};
        CHECK(near(c.score(raw), 0.75f));
        raw[0] = -50;
        CHECK(c.score(raw) == 1);  // 超出范围的输入被截断
    }

    // ---- 补偿：n = 1 时不变；单调、不超过 1、不低于原值 ----
    for (int n = 1; n <= 8; ++n) {
        float prev = -1;
        for (int i = 0; i <= 1000; ++i) {
            float s = i / 1000.0f, c = compensate(s, n);
            CHECK(c >= prev && c <= 1 && c >= s - 1e-7f);
            if (n == 1) CHECK(c == s);
            prev = c;
        }
    }
    {
        // 四个考量都是 0.8：乘积 0.41，几何平均仍是 0.8，补偿介于两者之间
        Action a{"a", 1, {{"", 0, 0, 1, linear()}, {"", 0, 0, 1, linear()}, {"", 0, 0, 1, linear()}, {"", 0, 0, 1, linear()}}};
        float raw[] = {0.8f};
        float p = score(a, raw, Combine::Product), c = score(a, raw, Combine::Compensated), g = score(a, raw, Combine::GeometricMean);
        CHECK(near(p, 0.4096f) && near(g, 0.8f) && p < c && c < g);
        std::printf("four considerations at 0.8: product %.3f, compensated %.3f, geometric mean %.3f\n", p, c, g);
    }

    // ---- 剪枝后的选择器与暴力全算：随机动作表，结果逐位相同 ----
    Pcg32 rng(34);
    long long bruteEvals = 0, fastEvals = 0;
    for (int t = 0; t < 20000; ++t) {
        int inputs = 1 + rng.below(6);
        std::vector<float> raw(inputs);
        for (float& v : raw) v = rng.nextFloat() * 1.4f - 0.2f;
        std::vector<Action> acts(1 + rng.below(24));
        for (Action& a : acts) {
            a.name = "a";
            // 权重取几个离散档位，制造大量并列，考验平局处理
            const float weights[] = {0.5f, 1, 1, 2, 3};
            a.weight = weights[rng.below(5)];
            int n = rng.below(6);
            for (int k = 0; k < n; ++k) {
                float lo = rng.nextFloat(), hi = rng.nextFloat();
                if (lo == hi) hi = lo + 0.5f;
                Curve c = rng.below(8) == 0 ? linear(0, float(rng.below(2))) : randomCurve(rng);  // 常数 0 / 1 曲线制造并列
                a.considerations.push_back({"c", int(rng.below(inputs)), lo, hi, c});
            }
        }
        auto order = orderByWeight(acts);
        Options opt;
        opt.mode = Combine(rng.below(3));
        opt.current = rng.below(3) == 0 ? -1 : int(rng.below(acts.size()));
        opt.momentum = rng.below(3) == 0 ? 0 : rng.nextFloat() * 0.5f;
        Choice ref = chooseBrute(acts, raw.data(), opt);
        Choice fast = choose(acts, order, raw.data(), opt);
        CHECK(fast.action == ref.action && fast.score == ref.score);
        CHECK(fast.evaluated <= ref.evaluated);
        opt.earlyExit = false;
        Choice full = choose(acts, order, raw.data(), opt);
        CHECK(full.action == ref.action && full.score == ref.score && full.evaluated == ref.evaluated);
        bruteEvals += ref.evaluated, fastEvals += fast.evaluated;
    }
    std::printf("random tables: early exit evaluated %.1f%% of considerations\n", 100.0 * fastEvals / bruteEvals);

    // ---- 加权随机：只在分数带内选，频率与得分成正比 ----
    {
        std::vector<Action> acts = {{"a", 1.0f, {}}, {"b", 0.9f, {}}, {"c", 0.6f, {}}, {"d", 0.85f, {}}};
        float raw[] = {0};
        int count[4] = {};
        const int n = 200000;
        for (int i = 0; i < n; ++i) ++count[chooseWeighted(acts, raw, Combine::Product, 0.2f, rng)];
        CHECK(count[2] == 0);  // 0.6 < 0.8 × 1.0，不在分数带内
        float total = 1 + 0.9f + 0.85f;
        CHECK(std::abs(count[0] / double(n) - 1 / total) < 0.01 && std::abs(count[1] / double(n) - 0.9f / total) < 0.01);
        for (int i = 0; i < 1000; ++i) CHECK(chooseWeighted(acts, raw, Combine::Product, 0, rng) == 0);  // band = 0：总是最高分
    }

    // ---- 士兵：典型情境下的选择 ----
    {
        using namespace ga::soldier;
        auto acts = soldier::actions();
        auto order = orderByWeight(acts);
        auto pick = [&](float health, float ammo, float enemy, float medkit, float path = 0) {
            float raw[] = {health, ammo, enemy, medkit, path};
            return choose(acts, order, raw, Options{}).action;
        };
        CHECK(pick(100, 8, 120, 300) == Attack);   // 满血满弹，敌人在射程内
        CHECK(pick(100, 0, 120, 300) == Reload);   // 没子弹
        CHECK(pick(100, 8, 280, 300) == Chase);    // 敌人在射程外、视野内
        CHECK(pick(100, 8, 500, 300) == Patrol);   // 看不见敌人
        CHECK(pick(15, 8, 80, 300) == Flee);       // 快死了，敌人很近
        CHECK(pick(30, 8, 380, 150, 0) == Heal);   // 受伤，敌人很远
        CHECK(pick(30, 8, 250, 300, 200) != Heal); // 去医疗点要穿过敌人的射程
        CHECK(pick(100, 3, 380, 300) == Reload);   // 敌人很远：趁机把弹夹补满

        // 长时间模拟：动作的分布合理，士兵不会一直被击倒
        World w;
        Pcg32 r(7);
        int histogram[kActionCount] = {};
        for (int i = 0; i < 30 * 600; ++i) {  // 10 分钟
            moveEnemy(w, 1 / 30.0f, r);
            auto in = inputs(w);
            w.action = choose(acts, order, in.data(), Options{Combine::Compensated, w.action, 0.1f}).action;
            act(w, w.action, 1 / 30.0f, r);
            ++histogram[w.action];
        }
        std::printf("soldier, 10 minutes:");
        for (int a = 0; a < kActionCount; ++a) std::printf(" %s %.0f%%", acts[a].name, 100.0 * histogram[a] / (30 * 600));
        std::printf(", downed %d times, %d kills\n", w.downs, w.kills);
        for (int a = 0; a < kActionCount; ++a) CHECK(histogram[a] > 0);  // 每个动作都用得上
        CHECK(w.downs <= 3);
    }
    return report("utility_test");
}
