#pragma once

// 效用 AI（Utility AI）
//   - 每个候选动作由若干"考量"（consideration）组成：读取一个输入（血量、距离、弹药……），
//     先归一化到 [0, 1]，再经过一条响应曲线，得到 [0, 1] 的分数
//   - 动作的得分 = 权重 × 各考量分数之积（任何一个考量为 0，这个动作就被否决）
//   - 每次决策选得分最高的动作；可选：给当前动作加一点"惯性"，避免在两个动作之间来回抖动
//
// 乘积有一个副作用：考量越多，得分越低。这里提供三种组合方式，以及剪枝后与暴力全算结果完全一致的选择器

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ga::utility {

// #region curve
// 响应曲线：把归一化的输入 x ∈ [0, 1] 映射为分数，结果截断到 [0, 1]
//   Linear    y = m (x − c) + b
//   Power     y = m (x − c)^k + b          （x − c < 0 时按 0 计）
//   Logistic  y = k / (1 + e^(−m (x − c))) + b   （S 形，m 控制陡峭程度，c 是中点）
struct Curve {
    enum Type { Linear, Power, Logistic } type = Linear;
    float m = 1, k = 1, b = 0, c = 0;

    float operator()(float x) const {
        float y = 0;
        switch (type) {
        case Linear: y = m * (x - c) + b; break;
        case Power: y = m * std::pow(std::max(x - c, 0.0f), k) + b; break;
        case Logistic: y = k / (1 + std::exp(-m * (x - c))) + b; break;
        }
        return std::clamp(y, 0.0f, 1.0f);
    }
};

// 常用曲线的简写
inline Curve linear(float m = 1, float b = 0) { return {Curve::Linear, m, 1, b, 0}; }
inline Curve inverseLinear() { return {Curve::Linear, -1, 1, 1, 0}; }
inline Curve power(float k, float m = 1, float b = 0) { return {Curve::Power, m, k, b, 0}; }
inline Curve inversePower(float k) { return {Curve::Power, -1, k, 1, 0}; }  // 1 − x^k
inline Curve logistic(float mid, float steep) { return {Curve::Logistic, steep, 1, 0, mid}; }  // steep < 0 为下降
// #endregion

// #region consideration
// 输入由游戏在每次决策前填好（raw[input]），考量负责归一化并查曲线
struct Consideration {
    const char* name;
    int input;
    float lo, hi;  // 原始输入的范围：lo 映射为 0，hi 映射为 1（lo > hi 时方向相反）
    Curve curve;

    float normalized(const float* raw) const { return std::clamp((raw[input] - lo) / (hi - lo), 0.0f, 1.0f); }
    float score(const float* raw) const { return curve(normalized(raw)); }
};

struct Action {
    const char* name;
    float weight = 1;  // 动作的"上限"：所有考量都是 1 时的得分。没有考量的动作得分就是权重本身
    std::vector<Consideration> considerations;
};
// #endregion

// #region combine
enum class Combine {
    Product,      // 直接相乘：考量越多，得分系统性地越低
    Compensated,  // Dave Mark 的补偿：每个分数先向 1 拉回一部分，拉回的比例随考量个数增加
    GeometricMean,// 乘积开 n 次方：各考量都是 s 时得分仍为 s
};

// 补偿后的单个考量分数。它在 [0, 1] 上单调递增且不超过 1，所以部分乘积始终是最终乘积的上界
inline float compensate(float s, int n) {
    float mod = 1 - 1.0f / n;
    return s + (1 - s) * mod * s;
}

inline float finish(float product, int n, Combine mode) {
    if (mode == Combine::GeometricMean && n > 1 && product > 0) return std::pow(product, 1.0f / n);
    return product;
}
// #endregion

// 完整计算一个动作的得分（暴力版本，选择器的对照）
inline float score(const Action& a, const float* raw, Combine mode, int* evaluated = nullptr) {
    int n = static_cast<int>(a.considerations.size());
    float p = 1;
    for (const Consideration& c : a.considerations) {
        float s = c.score(raw);
        if (evaluated) ++*evaluated;
        p *= mode == Combine::Compensated ? compensate(s, n) : s;
    }
    return a.weight * finish(p, n, mode);
}

struct Options {
    Combine mode = Combine::Compensated;
    int current = -1;       // 当前正在执行的动作
    float momentum = 0;     // 惯性：当前动作的得分乘以 (1 + momentum)
    bool earlyExit = true;  // 剪枝：上界已经不可能超过当前最好的动作时，停止计算
};

struct Choice {
    int action = -1;
    float score = 0;
    int evaluated = 0;  // 实际计算了多少个考量（衡量开销）
};

// 得分相同时取编号小的动作，保证剪枝版本与暴力版本选出完全相同的结果
inline bool better(float s, int i, float best, int bestIndex) { return s > best || (s == best && i < bestIndex); }

// 暴力版本：每个动作都完整计算
inline Choice chooseBrute(const std::vector<Action>& actions, const float* raw, const Options& opt) {
    Choice best;
    for (int i = 0; i < static_cast<int>(actions.size()); ++i) {
        float s = score(actions[i], raw, opt.mode, &best.evaluated);
        if (i == opt.current) s *= 1 + opt.momentum;
        if (best.action < 0 || better(s, i, best.score, best.action)) best.action = i, best.score = s;
    }
    return best;
}

// #region choose
// order：按权重从大到小排好的动作编号（只需在动作表变化时排一次）
inline Choice choose(const std::vector<Action>& actions, const std::vector<int>& order, const float* raw, const Options& opt) {
    Choice best;
    bool currentSeen = opt.current < 0 || opt.momentum == 0;
    for (int i : order) {
        const Action& a = actions[i];
        currentSeen |= i == opt.current;
        float bonus = i == opt.current ? 1 + opt.momentum : 1;
        int n = static_cast<int>(a.considerations.size());
        // 上界 = 权重 × 部分乘积。考量分数都不超过 1，乘得越多只会越小
        auto hopeless = [&](float p) {
            float bound = a.weight * finish(p, n, opt.mode) * bonus;
            return best.action >= 0 && !better(bound, i, best.score, best.action);
        };
        if (opt.earlyExit && hopeless(1)) {
            if (currentSeen) break;  // 后面的权重更小，全都不可能赢
            continue;                // 有惯性时，排在后面的当前动作可能还有机会
        }
        float p = 1;
        bool pruned = false;
        for (const Consideration& c : a.considerations) {
            float s = c.score(raw);
            ++best.evaluated;
            p *= opt.mode == Combine::Compensated ? compensate(s, n) : s;
            if (opt.earlyExit && hopeless(p)) { pruned = true; break; }
        }
        if (pruned) continue;
        float s = a.weight * finish(p, n, opt.mode) * bonus;
        if (best.action < 0 || better(s, i, best.score, best.action)) best.action = i, best.score = s;
    }
    return best;
}
// #endregion

inline std::vector<int> orderByWeight(const std::vector<Action>& actions) {
    std::vector<int> order(actions.size());
    for (int i = 0; i < static_cast<int>(order.size()); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return actions[a].weight > actions[b].weight; });
    return order;
}

// #region weighted
// 加权随机：在得分不低于 (1 − band) × 最高分的动作中，按得分比例随机挑一个。
// band = 0 退化为选最高分；适合"几个差不多好的选择"之间增加变化
template <class Rng>
int chooseWeighted(const std::vector<Action>& actions, const float* raw, Combine mode, float band, Rng& rng) {
    std::vector<float> s(actions.size());
    float best = 0;
    for (size_t i = 0; i < actions.size(); ++i) best = std::max(best, s[i] = score(actions[i], raw, mode));
    if (best <= 0) return -1;
    float total = 0;
    for (float& v : s) total += v = v >= (1 - band) * best ? v : 0;
    float r = rng.nextFloat() * total;
    for (size_t i = 0; i < s.size(); ++i)
        if (s[i] > 0 && (r -= s[i]) < 0) return static_cast<int>(i);
    for (size_t i = s.size(); i-- > 0;)
        if (s[i] > 0) return static_cast<int>(i);  // 浮点误差兜底
    return -1;
}
// #endregion

}  // namespace ga::utility
