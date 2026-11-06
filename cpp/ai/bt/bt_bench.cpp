// 行为树的每帧开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/bt/bt_bench.cpp -o bt_bench && ./bt_bench
#include <chrono>
#include <cstdio>
#include <vector>

#include "bt.h"
#include "bt_oop.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

// 一个简化的"搬箱子的工人"：与演示中的树结构相同，动作只是计数
struct Ctx {
    uint32_t rng = 1;
    float hunger = 0;
    bool enemyNear = false, doorOpen = false, locked = true, hasKey = false, carrying = false;
    int busy = 0;  // 当前动作还要运行几帧
    uint32_t next() { return rng = rng * 1664525u + 1013904223u; }
};

static Status work(Ctx& c, int ticks) {
    if (c.busy == 0) c.busy = ticks;
    return --c.busy == 0 ? Status::Success : Status::Running;
}

static int buildWorker(BtDef<Ctx>& d, bool reactive) {
    auto act = [&](const char* name, int ticks) {
        return d.action(name, [ticks](Ctx& c) { return work(c, ticks); }, [](Ctx& c) { c.busy = 0; });
    };
    int flee = d.sequence("flee", {d.condition("enemy near", [](const Ctx& c) { return c.enemyNear; }), act("run away", 5)}, false);
    int eat = d.sequence("eat", {d.condition("hungry", [](const Ctx& c) { return c.hunger > 70; }), act("go to food", 8),
                                 d.action("eat", [](Ctx& c) { c.hunger = 0; return Status::Success; })});
    int open = d.selector("open door", {
        d.condition("door open", [](const Ctx& c) { return c.doorOpen; }),
        d.sequence("open", {d.condition("not locked", [](const Ctx& c) { return !c.locked; }), act("open", 2)}),
        d.sequence("unlock", {d.condition("has key", [](const Ctx& c) { return c.hasKey; }), act("unlock", 3), act("open", 2)}),
        d.retry(d.action("kick", [](Ctx& c) { return c.next() % 3 == 0 ? Status::Success : Status::Failure; }), 3),
    });
    int fetch = d.sequence("fetch", {act("go to door", 10), open, act("go to chest", 10), act("pick up", 2), act("go to door", 10),
                                     act("go to drop", 10), act("drop", 2)});
    int idle = act("idle", 20);
    return d.selector("root", {flee, eat, fetch, idle}, !reactive);
}

static void randomize(Ctx& c) {
    c.hunger += 0.5f;
    if (c.next() % 200 == 0) c.enemyNear = !c.enemyNear;
    if (c.next() % 50 == 0) c.locked = !c.locked;
    if (c.next() % 80 == 0) c.hasKey = !c.hasKey;
    c.doorOpen = c.next() % 4 == 0;
}

// 一个永远不满足条件的分支：条件 + 动作
static int d_seq(BtDef<Ctx>& def, int i) {
    return def.sequence("branch " + std::to_string(i),
                        {def.condition("never", [](const Ctx& c) { return c.busy < 0; }), def.action("x", [](Ctx&) { return Status::Success; })});
}

int main() {
    const int agents = 10000, ticks = 200;
    auto bench = [&](const char* name, bool reactive, bool shared) {
        BtDef<Ctx> def;
        buildWorker(def, reactive);
        double best = 1e30;
        long visits = 0;
        for (int rep = 0; rep < 5; ++rep) {
            std::vector<Ctx> ctx(agents);
            for (int a = 0; a < agents; ++a) ctx[a].rng = a * 7919 + 1;
            std::vector<Bt<Ctx>> flat;
            std::vector<std::unique_ptr<oop::Node<Ctx>>> trees;
            if (shared) flat.assign(agents, Bt<Ctx>(def));
            else
                for (int a = 0; a < agents; ++a) trees.push_back(oop::build(def, def.root()));
            auto t0 = Clock::now();
            for (int t = 0; t < ticks; ++t)
                for (int a = 0; a < agents; ++a) {
                    randomize(ctx[a]);
                    if (shared) flat[a].tick(ctx[a]);
                    else trees[a]->tick(ctx[a]);
                }
            best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / (double(agents) * ticks));
            if (shared) {
                visits = 0;
                for (auto& f : flat) visits += f.visits();
            }
        }
        if (shared) std::printf("  %-44s %6.1f ns per agent per tick, %.1f nodes visited per tick\n", name, best, double(visits) / agents / ticks);
        else std::printf("  %-44s %6.1f ns per agent per tick\n", name, best);
    };
    std::printf("1) %d workers x %d ticks (17 leaves, 26 nodes)\n", agents, ticks);
    bench("object tree per agent, reactive root", true, false);
    bench("shared definition, reactive root", true, true);
    bench("object tree per agent, memory root", false, false);
    bench("shared definition, memory root", false, true);

    std::printf("\n2) reactive root re-checks every higher-priority branch each tick\n");
    std::printf("   root = selector of N guarded branches; only the last one is running\n");
    for (int n : {1, 4, 16, 64, 256}) {
        for (bool reactive : {true, false}) {
            BtDef<Ctx> def;
            std::vector<int> branches;
            for (int i = 0; i < n - 1; ++i)
                branches.push_back(d_seq(def, i));
            branches.push_back(def.action("long task", [](Ctx& c) { return work(c, 1000000); }));
            def.selector("root", branches, !reactive);
            std::vector<Ctx> ctx(1000);
            std::vector<Bt<Ctx>> trees(1000, Bt<Ctx>(def));
            for (int a = 0; a < 1000; ++a) trees[a].tick(ctx[a]);  // 进入最后一个分支
            auto t0 = Clock::now();
            for (int t = 0; t < 200; ++t)
                for (int a = 0; a < 1000; ++a) trees[a].tick(ctx[a]);
            double ns = std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / (1000.0 * 200);
            std::printf("   N = %-4d %-9s %8.1f ns per tick\n", n, reactive ? "reactive" : "memory", ns);
        }
    }
}
