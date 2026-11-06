#pragma once

// 分层有限状态机（HFSM）。没有父状态的状态机就是普通的有限状态机（FSM）
//   - 状态可以嵌套：父状态的转移对它所有的子状态都有效，而且优先于子状态自己的转移
//   - 切换状态时，从当前叶子一路退出到两者的最近公共祖先，再一路进入目标；
//     目标若是复合状态，沿着"初始子状态"继续往下进入，直到叶子
//   - 历史转移：回到复合状态时，恢复上次离开时所在的子状态，而不是初始子状态
//
// 定义与实例分开：HfsmDef 描述状态和转移，所有使用同一种 AI 的角色共享一份；
// Hfsm 只记录某个角色"当前在哪个状态"，很小。
// Ctx 是状态机读写的"上下文"（感知输入、计时器等），所有回调都以它为参数

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

namespace ga {

// #region def
template <class Ctx>
class HfsmDef {
public:
    using Action = std::function<void(Ctx&)>;
    using Condition = std::function<bool(const Ctx&)>;

    struct State {
        std::string name;
        int parent = -1;
        int depth = 0;
        int initial = -1;  // 复合状态的初始子状态；-1 表示叶子
        Action onEnter, onUpdate, onExit;
        std::vector<int> transitions;  // 这个状态"自己"的转移，按添加顺序
    };

    struct Transition {
        int from, to;
        Condition when;
        bool resume;  // true：历史转移，进入复合状态时恢复上次所在的子状态
    };

    int addState(std::string name, int parent = -1, Action onEnter = {}, Action onUpdate = {}, Action onExit = {}) {
        int depth = parent < 0 ? 0 : states_[parent].depth + 1;
        states_.push_back({std::move(name), parent, depth, -1, std::move(onEnter), std::move(onUpdate), std::move(onExit), {}});
        int id = static_cast<int>(states_.size()) - 1;
        if (parent >= 0 && states_[parent].initial < 0) states_[parent].initial = id;  // 第一个子状态默认为初始子状态
        return id;
    }
    void setInitial(int parent, int child) { states_[parent].initial = child; }

    // from 可以是复合状态：只要它的任何一个子孙处于活动状态，这条转移就会被检查
    void addTransition(int from, int to, Condition when, bool resume = false) {
        transitions_.push_back({from, to, std::move(when), resume});
        states_[from].transitions.push_back(static_cast<int>(transitions_.size()) - 1);
    }

    const std::vector<State>& states() const { return states_; }
    const std::vector<Transition>& transitions() const { return transitions_; }

private:
    std::vector<State> states_;
    std::vector<Transition> transitions_;
};
// #endregion

template <class Ctx>
class Hfsm {
public:
    static constexpr int kMaxDepth = 16;  // 嵌套层数上限

    explicit Hfsm(const HfsmDef<Ctx>& def) : def_(&def), lastChild_(def.states().size(), -1) {}

    void start(int initialState, Ctx& ctx) {
        leaf_ = -1;
        enterDown(initialState, -1, false, ctx);
    }

    // #region update
    // 每次更新：从最外层的祖先到当前叶子，依次检查每一层的转移，第一个满足的立即执行（外层优先）；
    // 然后从外到内执行活动状态的 onUpdate。每次更新最多发生一次转移
    void update(Ctx& ctx) {
        const auto& states = def_->states();
        int chain[kMaxDepth];
        int n = fillChain(chain);  // 根 → 叶
        for (int k = 0; k < n; ++k)
            for (int ti : states[chain[k]].transitions) {
                const auto& t = def_->transitions()[ti];
                if (t.when(ctx)) {
                    changeTo(t.to, t.resume, ctx);
                    ++transitionsTaken_;
                    n = fillChain(chain);
                    goto updated;
                }
            }
    updated:
        for (int k = 0; k < n; ++k)
            if (states[chain[k]].onUpdate) states[chain[k]].onUpdate(ctx);
    }
    // #endregion

    int leaf() const { return leaf_; }
    const std::string& leafName() const { return def_->states()[leaf_].name; }
    bool isActive(int s) const {
        for (int x = leaf_; x >= 0; x = def_->states()[x].parent)
            if (x == s) return true;
        return false;
    }
    std::vector<int> activeChain() const {
        int chain[kMaxDepth];
        int n = fillChain(chain);
        return {chain, chain + n};
    }
    long transitionsTaken() const { return transitionsTaken_; }

private:
    // 当前活动的状态链（根 → 叶）写进固定大小的数组：每帧都要用，避免分配内存
    int fillChain(int* out) const {
        int n = 0;
        for (int x = leaf_; x >= 0; x = def_->states()[x].parent) out[n++] = x;
        std::reverse(out, out + n);
        return n;
    }

    // #region change
    void changeTo(int target, bool resume, Ctx& ctx) {
        const auto& states = def_->states();
        // 最近公共祖先。目标是当前叶子本身或它的祖先时，视为"外部转移"：先退出目标，再重新进入
        int a = leaf_, b = target;
        while (states[a].depth > states[b].depth) a = states[a].parent;
        while (states[b].depth > states[a].depth) b = states[b].parent;
        while (a != b) a = states[a].parent, b = states[b].parent;
        int lca = a;
        if (lca == target) lca = states[target].parent;
        // 1. 从叶子往上退出，直到最近公共祖先（不含）
        for (int s = leaf_; s != lca; s = states[s].parent) {
            if (states[s].onExit) states[s].onExit(ctx);
            if (int p = states[s].parent; p >= 0) lastChild_[p] = s;  // 记下离开时所在的子状态
        }
        // 2. 从最近公共祖先往下进入目标
        enterDown(target, lca, resume, ctx);
    }

    // 进入 target：先进入 from（不含）到 target 之间的所有祖先，再进入 target，
    // 然后沿初始子状态（或历史）一路进入到叶子
    void enterDown(int target, int from, bool resume, Ctx& ctx) {
        const auto& states = def_->states();
        int path[kMaxDepth], n = 0;
        for (int s = target; s != from; s = states[s].parent) path[n++] = s;
        for (int k = n - 1; k >= 0; --k)
            if (states[path[k]].onEnter) states[path[k]].onEnter(ctx);
        int s = target;
        while (states[s].initial >= 0) {
            int child = resume && lastChild_[s] >= 0 ? lastChild_[s] : states[s].initial;
            if (states[child].onEnter) states[child].onEnter(ctx);
            s = child;
        }
        leaf_ = s;
    }
    // #endregion

    const HfsmDef<Ctx>* def_;
    std::vector<int> lastChild_;  // 每个复合状态上次活动的子状态（历史转移用）
    int leaf_ = -1;
    long transitionsTaken_ = 0;
};

}  // namespace ga
