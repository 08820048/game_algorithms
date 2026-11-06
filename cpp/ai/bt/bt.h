#pragma once

// 行为树（Behavior Tree）
//   - 每个节点被"执行"（tick）后返回三种结果之一：成功、失败、运行中
//   - 叶子：动作（Action）与条件（Condition），由游戏提供
//   - 组合节点：顺序（Sequence，全部成功才成功）、选择（Selector，一个成功就成功）、并行（Parallel）
//   - 装饰节点：取反（Inverter）、重复（Repeat）、重试（Retry）、总是成功（AlwaysSucceed）
//   - 组合节点有两种：有记忆的从上次运行中的子节点继续；响应式的每次从头检查，可以被更高优先级的分支打断
//   - 被打断的"运行中"节点会收到 halt 通知，让动作有机会清理（停下脚步、取消动画……）
//
// 与 3.1 的状态机一样，定义（BtDef）与实例（Bt）分开：树的结构所有角色共享，每个角色只存每个节点的一点运行状态

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ga {

enum class Status : uint8_t { Success, Failure, Running };

// #region def
template <class Ctx>
class BtDef {
public:
    enum class Kind : uint8_t { Action, Condition, Sequence, Selector, Parallel, Inverter, Repeat, Retry, AlwaysSucceed };

    struct Node {
        Kind kind;
        std::string name;
        std::vector<int> children;
        bool memory = true;  // 组合节点：true 有记忆，false 响应式
        int param = 0;       // Parallel：需要几个子节点成功；Repeat / Retry：次数
        std::function<Status(Ctx&)> act;
        std::function<bool(const Ctx&)> test;
        std::function<void(Ctx&)> onHalt;
    };

    int action(std::string name, std::function<Status(Ctx&)> act, std::function<void(Ctx&)> onHalt = {}) {
        return add({Kind::Action, std::move(name), {}, true, 0, std::move(act), {}, std::move(onHalt)});
    }
    int condition(std::string name, std::function<bool(const Ctx&)> test) {
        return add({Kind::Condition, std::move(name), {}, true, 0, {}, std::move(test), {}});
    }
    int sequence(std::string name, std::vector<int> children, bool memory = true) {
        return add(make(Kind::Sequence, std::move(name), std::move(children), memory));
    }
    int selector(std::string name, std::vector<int> children, bool memory = true) {
        return add(make(Kind::Selector, std::move(name), std::move(children), memory));
    }
    int parallel(std::string name, std::vector<int> children, int successNeeded) {
        return add(make(Kind::Parallel, std::move(name), std::move(children), false, successNeeded));
    }
    int inverter(int child) { return add(make(Kind::Inverter, "not", {child})); }
    int repeat(int child, int times) { return add(make(Kind::Repeat, "repeat " + std::to_string(times), {child}, true, times)); }
    int retry(int child, int times) { return add(make(Kind::Retry, "retry " + std::to_string(times), {child}, true, times)); }
    int alwaysSucceed(int child) { return add(make(Kind::AlwaysSucceed, "always succeed", {child})); }

    void setRoot(int n) { root_ = n; }
    int root() const { return root_; }
    const std::vector<Node>& nodes() const { return nodes_; }

private:
    static Node make(Kind kind, std::string name, std::vector<int> children, bool memory = true, int param = 0) {
        Node n;
        n.kind = kind, n.name = std::move(name), n.children = std::move(children), n.memory = memory, n.param = param;
        return n;
    }
    int add(Node n) {
        nodes_.push_back(std::move(n));
        return root_ = static_cast<int>(nodes_.size()) - 1;  // 最后添加的节点默认为根
    }
    std::vector<Node> nodes_;
    int root_ = -1;
};
// #endregion

template <class Ctx>
class Bt {
public:
    explicit Bt(const BtDef<Ctx>& def)
        : def_(&def), cursor_(def.nodes().size(), 0), running_(def.nodes().size(), 0), last_(def.nodes().size(), Status::Failure),
          lastTick_(def.nodes().size(), 0) {}

    Status tick(Ctx& ctx) {
        ++tick_;
        return run(def_->root(), ctx);
    }

    // 中止整棵树（比如角色死亡）
    void haltAll(Ctx& ctx) { halt(def_->root(), ctx); }

    // 供可视化使用：节点上一次被执行的结果，以及是否在本次 tick 中被执行
    Status lastStatus(int n) const { return last_[n]; }
    bool visitedThisTick(int n) const { return lastTick_[n] == tick_; }
    bool isRunning(int n) const { return running_[n] != 0; }
    uint32_t ticks() const { return tick_; }
    long visits() const { return visits_; }

private:
    using Kind = typename BtDef<Ctx>::Kind;

    // #region run
    Status run(int n, Ctx& ctx) {
        const auto& node = def_->nodes()[n];
        ++visits_;
        lastTick_[n] = tick_;
        Status s = Status::Failure;
        switch (node.kind) {
            case Kind::Action: s = node.act(ctx); break;
            case Kind::Condition: s = node.test(ctx) ? Status::Success : Status::Failure; break;
            case Kind::Sequence: s = runComposite(n, ctx, Status::Success); break;
            case Kind::Selector: s = runComposite(n, ctx, Status::Failure); break;
            case Kind::Parallel: s = runParallel(n, ctx); break;
            case Kind::Inverter: {
                Status c = run(node.children[0], ctx);
                s = c == Status::Running ? c : c == Status::Success ? Status::Failure : Status::Success;
                break;
            }
            case Kind::Repeat:  // 子节点成功 times 次才成功；每次 tick 最多完成一轮，避免一帧内死循环
            case Kind::Retry: {  // 子节点失败了就再试，最多 times 次
                Status c = run(node.children[0], ctx);
                Status keepGoing = node.kind == Kind::Repeat ? Status::Success : Status::Failure;
                if (c == Status::Running) s = c;
                else if (c != keepGoing) cursor_[n] = 0, s = c;
                else if (++cursor_[n] >= node.param) cursor_[n] = 0, s = c;
                else s = Status::Running;
                break;
            }
            case Kind::AlwaysSucceed: s = run(node.children[0], ctx) == Status::Running ? Status::Running : Status::Success; break;
        }
        running_[n] = s == Status::Running;
        last_[n] = s;
        return s;
    }
    // #endregion

    // #region composite
    // 顺序与选择是同一个逻辑的两面：continueOn = Success 是顺序（子节点成功就继续），
    // continueOn = Failure 是选择（子节点失败就试下一个）
    Status runComposite(int n, Ctx& ctx, Status continueOn) {
        const auto& node = def_->nodes()[n];
        int count = static_cast<int>(node.children.size());
        int i = node.memory ? cursor_[n] : 0;  // 有记忆：从上次运行中的子节点继续；响应式：每次从头开始
        Status result = continueOn;
        for (; i < count; ++i) {
            Status c = run(node.children[i], ctx);
            if (c == continueOn) continue;
            result = c;
            break;
        }
        // 这次没有执行到的、上次却还在运行的子节点（被更靠前的分支打断了），要通知它们停下
        for (int j = i + 1; j < count; ++j) halt(node.children[j], ctx);
        cursor_[n] = result == Status::Running ? i : 0;
        return result;
    }
    // #endregion

    // 并行：每次执行所有子节点；成功的数量达到 param 就成功，不可能再达到就失败
    Status runParallel(int n, Ctx& ctx) {
        const auto& node = def_->nodes()[n];
        int ok = 0, fail = 0, count = static_cast<int>(node.children.size());
        for (int c : node.children) {
            Status s = run(c, ctx);
            ok += s == Status::Success;
            fail += s == Status::Failure;
        }
        if (ok < node.param && count - fail >= node.param) return Status::Running;
        for (int c : node.children) halt(c, ctx);
        return ok >= node.param ? Status::Success : Status::Failure;
    }

    // #region halt
    void halt(int n, Ctx& ctx) {
        if (!running_[n]) return;
        const auto& node = def_->nodes()[n];
        for (int c : node.children) halt(c, ctx);
        if (node.kind == Kind::Action && node.onHalt) node.onHalt(ctx);
        running_[n] = 0;
        cursor_[n] = 0;
    }
    // #endregion

    const BtDef<Ctx>* def_;
    std::vector<int> cursor_;        // 组合节点：运行中的子节点；Repeat / Retry：已完成的次数
    std::vector<uint8_t> running_;   // 上一次执行的结果是否为"运行中"
    std::vector<Status> last_;
    std::vector<uint32_t> lastTick_;
    uint32_t tick_ = 0;
    long visits_ = 0;
};

}  // namespace ga
