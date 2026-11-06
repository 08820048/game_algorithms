#pragma once

// 行为树最常见的写法：每种节点一个类，虚函数 tick()，每个角色拥有一整棵节点对象。
// 用作单元测试的独立对照实现，以及性能对比的基准。语义与 bt.h 完全相同

#include <memory>
#include <vector>

#include "bt.h"

namespace ga::oop {

// #region oop
template <class Ctx>
struct Node {
    virtual ~Node() = default;
    Status tick(Ctx& ctx) {
        Status s = update(ctx);
        running = s == Status::Running;
        return s;
    }
    void halt(Ctx& ctx) {
        if (!running) return;
        onHalt(ctx);
        running = false;
    }
    bool running = false;

protected:
    virtual Status update(Ctx& ctx) = 0;
    virtual void onHalt(Ctx&) {}
};

template <class Ctx>
struct Action : Node<Ctx> {
    std::function<Status(Ctx&)> act;
    std::function<void(Ctx&)> stop;
    Status update(Ctx& ctx) override { return act(ctx); }
    void onHalt(Ctx& ctx) override {
        if (stop) stop(ctx);
    }
};

template <class Ctx>
struct Condition : Node<Ctx> {
    std::function<bool(const Ctx&)> test;
    Status update(Ctx& ctx) override { return test(ctx) ? Status::Success : Status::Failure; }
};

template <class Ctx>
struct Composite : Node<Ctx> {
    std::vector<std::unique_ptr<Node<Ctx>>> children;
    bool memory = true;
    bool isSequence = true;
    size_t current = 0;

    Status update(Ctx& ctx) override {
        Status continueOn = isSequence ? Status::Success : Status::Failure;
        size_t i = memory ? current : 0;
        Status result = continueOn;
        for (; i < children.size(); ++i) {
            Status s = children[i]->tick(ctx);
            if (s != continueOn) {
                result = s;
                break;
            }
        }
        for (size_t j = i + 1; j < children.size(); ++j) children[j]->halt(ctx);
        current = result == Status::Running ? i : 0;
        return result;
    }
    void onHalt(Ctx& ctx) override {
        for (auto& c : children) c->halt(ctx);
        current = 0;
    }
};
// #endregion

template <class Ctx>
struct Parallel : Node<Ctx> {
    std::vector<std::unique_ptr<Node<Ctx>>> children;
    int successNeeded = 1;
    Status update(Ctx& ctx) override {
        int ok = 0, fail = 0;
        for (auto& c : children) {
            Status s = c->tick(ctx);
            ok += s == Status::Success;
            fail += s == Status::Failure;
        }
        if (ok < successNeeded && int(children.size()) - fail >= successNeeded) return Status::Running;
        for (auto& c : children) c->halt(ctx);
        return ok >= successNeeded ? Status::Success : Status::Failure;
    }
    void onHalt(Ctx& ctx) override {
        for (auto& c : children) c->halt(ctx);
    }
};

template <class Ctx>
struct Decorator : Node<Ctx> {
    std::unique_ptr<Node<Ctx>> child;
    typename BtDef<Ctx>::Kind kind;
    int times = 1, count = 0;
    Status update(Ctx& ctx) override {
        using K = typename BtDef<Ctx>::Kind;
        Status s = child->tick(ctx);
        if (s == Status::Running) return s;
        switch (kind) {
            case K::Inverter: return s == Status::Success ? Status::Failure : Status::Success;
            case K::AlwaysSucceed: return Status::Success;
            case K::Repeat:
                if (s == Status::Failure) return count = 0, s;
                if (++count >= times) return count = 0, s;
                return Status::Running;
            default:  // Retry
                if (s == Status::Success) return count = 0, s;
                if (++count >= times) return count = 0, s;
                return Status::Running;
        }
    }
    void onHalt(Ctx& ctx) override {
        child->halt(ctx);
        count = 0;
    }
};

// 按同一份定义构造一棵对象树（每个角色一棵）
template <class Ctx>
std::unique_ptr<Node<Ctx>> build(const BtDef<Ctx>& def, int n) {
    using K = typename BtDef<Ctx>::Kind;
    const auto& d = def.nodes()[n];
    switch (d.kind) {
        case K::Action: {
            auto a = std::make_unique<Action<Ctx>>();
            a->act = d.act, a->stop = d.onHalt;
            return a;
        }
        case K::Condition: {
            auto c = std::make_unique<Condition<Ctx>>();
            c->test = d.test;
            return c;
        }
        case K::Sequence:
        case K::Selector: {
            auto c = std::make_unique<Composite<Ctx>>();
            c->memory = d.memory, c->isSequence = d.kind == K::Sequence;
            for (int ch : d.children) c->children.push_back(build(def, ch));
            return c;
        }
        case K::Parallel: {
            auto p = std::make_unique<Parallel<Ctx>>();
            p->successNeeded = d.param;
            for (int ch : d.children) p->children.push_back(build(def, ch));
            return p;
        }
        default: {
            auto dec = std::make_unique<Decorator<Ctx>>();
            dec->kind = d.kind, dec->times = d.param, dec->child = build(def, d.children[0]);
            return dec;
        }
    }
}

}  // namespace ga::oop
