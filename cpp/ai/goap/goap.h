#pragma once

// GOAP（Goal-Oriented Action Planning，目标导向行为规划）
//   - 世界状态是一组布尔"事实"，用 64 位整数的每一位表示（有斧头、有木头、火已点燃……）
//   - 每个动作有前提条件、效果和代价
//   - 规划器用 A* 搜索：从当前状态出发，找到代价最小的动作序列，使目标的所有事实成立
//   - 正向搜索从当前状态往前推；反向搜索从目标往回推（F.E.A.R. 的做法）
//
// 与卷一的寻路完全相同的算法，只是"格子"换成了世界状态，"走一步"换成了执行一个动作

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace ga {

// #region facts
// 一组对事实的要求：mask 中为 1 的位是"关心的事实"，value 给出它们应该的取值
struct Facts {
    uint64_t mask = 0, value = 0;

    bool satisfiedBy(uint64_t state) const { return (state & mask) == value; }
    uint64_t applyTo(uint64_t state) const { return (state & ~mask) | value; }  // 作为效果：改写这些位
    int unsatisfied(uint64_t state) const { return std::popcount((state ^ value) & mask); }

    Facts& set(int fact, bool v = true) {
        mask |= uint64_t(1) << fact;
        if (v) value |= uint64_t(1) << fact;
        else value &= ~(uint64_t(1) << fact);
        return *this;
    }
};

inline bool operator==(const Facts& a, const Facts& b) { return a.mask == b.mask && a.value == b.value; }

struct GoapAction {
    std::string name;
    float cost;
    Facts pre;  // 前提条件
    Facts eff;  // 效果
};
// #endregion

struct Plan {
    bool found = false;
    float cost = 0;
    std::vector<int> actions;  // 按执行顺序
    int expanded = 0;          // 搜索中展开了多少个节点
};

enum class Heuristic {
    None,        // h = 0，退化为 Dijkstra
    Count,       // 未满足的目标事实个数：直观，但一个动作能同时满足多个事实时会高估，不可采纳
    Admissible,  // 未满足的事实数 ÷ 单个动作最多能改变的事实数 × 最小代价：绝不高估
};

namespace detail {
struct HeuristicParams {
    float minCost = 1;
    int maxEffects = 1;
};
inline HeuristicParams heuristicParams(const std::vector<GoapAction>& actions) {
    HeuristicParams p{1e30f, 1};
    for (const GoapAction& a : actions) p.minCost = std::min(p.minCost, a.cost), p.maxEffects = std::max(p.maxEffects, std::popcount(a.eff.mask));
    if (actions.empty()) p.minCost = 1;
    return p;
}
inline float estimate(Heuristic h, int unsatisfied, const HeuristicParams& p) {
    switch (h) {
        case Heuristic::None: return 0;
        case Heuristic::Count: return float(unsatisfied);
        default: return std::ceil(float(unsatisfied) / p.maxEffects) * p.minCost;
    }
}

// 通用的 A*：Key 是搜索节点的标识（正向：世界状态；反向：尚未满足的要求）
template <class Key, class KeyHash, class Expand, class IsGoal, class H>
Plan astar(Key start, Expand&& expand, IsGoal&& isGoal, H&& h, int maxExpanded, bool reverseResult) {
    struct Node {
        Key key;
        float g;
        int parent, action;
    };
    std::vector<Node> nodes{{start, 0, -1, -1}};
    std::unordered_map<Key, float, KeyHash> best{{start, 0.0f}};
    using Item = std::pair<float, int>;  // (f, 节点下标)
    std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
    open.push({h(start), 0});
    Plan plan;
    while (!open.empty() && plan.expanded < maxExpanded) {
        auto [f, i] = open.top();
        open.pop();
        Node node = nodes[i];
        if (node.g > best[node.key]) continue;  // 过期的条目
        ++plan.expanded;
        if (isGoal(node.key)) {
            plan.found = true, plan.cost = node.g;
            for (int k = i; nodes[k].parent >= 0; k = nodes[k].parent) plan.actions.push_back(nodes[k].action);
            if (!reverseResult) std::reverse(plan.actions.begin(), plan.actions.end());
            return plan;
        }
        expand(node.key, [&](Key next, int action, float cost) {
            float g = node.g + cost;
            auto it = best.find(next);
            if (it != best.end() && it->second <= g) return;
            best[next] = g;
            nodes.push_back({next, g, i, action});
            open.push({g + h(next), static_cast<int>(nodes.size()) - 1});
        });
    }
    return plan;
}

struct FactsHash {
    size_t operator()(const Facts& f) const { return std::hash<uint64_t>()(f.mask * 0x9E3779B97F4A7C15ull ^ f.value); }
};
}  // namespace detail

// #region forward
// 正向搜索：节点是世界状态；从当前状态出发，执行前提满足的动作，直到目标成立
inline Plan planForward(const std::vector<GoapAction>& actions, uint64_t start, Facts goal, Heuristic h = Heuristic::Admissible,
                        int maxExpanded = 200000) {
    auto params = detail::heuristicParams(actions);
    return detail::astar<uint64_t, std::hash<uint64_t>>(
        start,
        [&](uint64_t s, auto&& emit) {
            for (int a = 0; a < static_cast<int>(actions.size()); ++a)
                if (actions[a].pre.satisfiedBy(s)) emit(actions[a].eff.applyTo(s), a, actions[a].cost);
        },
        [&](uint64_t s) { return goal.satisfiedBy(s); },
        [&](uint64_t s) { return detail::estimate(h, goal.unsatisfied(s), params); }, maxExpanded, false);
}
// #endregion

// #region backward
// 反向搜索（回归）：节点是"还需要满足的要求"。从目标出发，选一个能满足其中某些要求、
// 又不破坏其他要求的动作，把它满足的要求换成它的前提条件，直到当前状态已经满足全部要求
inline Plan planBackward(const std::vector<GoapAction>& actions, uint64_t start, Facts goal, Heuristic h = Heuristic::Admissible,
                         int maxExpanded = 200000) {
    auto params = detail::heuristicParams(actions);
    Facts root{goal.mask, goal.value & goal.mask};
    return detail::astar<Facts, detail::FactsHash>(
        root,
        [&](const Facts& need, auto&& emit) {
            for (int a = 0; a < static_cast<int>(actions.size()); ++a) {
                const GoapAction& act = actions[a];
                uint64_t overlap = act.eff.mask & need.mask;
                if (!overlap) continue;                                       // 与要求无关
                if ((act.eff.value ^ need.value) & overlap) continue;         // 会破坏某个要求
                uint64_t restMask = need.mask & ~act.eff.mask;  // 这个动作满足不了的要求，仍然需要
                uint64_t restValue = need.value & restMask;
                if ((act.pre.value ^ restValue) & act.pre.mask & restMask) continue;  // 前提与剩余要求矛盾
                emit(Facts{restMask | act.pre.mask, restValue | act.pre.value}, a, act.cost);
            }
        },
        [&](const Facts& need) { return need.satisfiedBy(start); },
        [&](const Facts& need) { return detail::estimate(h, need.unsatisfied(start), params); }, maxExpanded, true);
}
// #endregion

// 按顺序执行计划，检查每一步的前提，返回最终状态（某一步前提不满足时 ok = false）
inline uint64_t simulate(const std::vector<GoapAction>& actions, uint64_t state, const std::vector<int>& plan, bool* ok) {
    *ok = true;
    for (int a : plan) {
        if (!actions[a].pre.satisfiedBy(state)) *ok = false;
        state = actions[a].eff.applyTo(state);
    }
    return state;
}

}  // namespace ga
