#include "bt.h"

#include <cstdio>
#include <string>

#include "../../foundations/random/random.h"
#include "bt_oop.h"
#include "check.h"

using namespace ga;

// 手写例子用的上下文：记录调用与中止
struct Log {
    std::string events;
    bool flag = false;
};

// 一个"运行 n 次后返回 result"的动作，调用时记录名字，中止时记录 "!名字"
static int timed(BtDef<Log>& def, const std::string& name, int runningTicks, Status result, int* counter) {
    return def.action(
        name,
        [=](Log& l) {
            l.events += name;
            return (*counter)++ < runningTicks ? Status::Running : (*counter = 0, result);
        },
        [=](Log& l) {
            l.events += "!" + name;
            *counter = 0;
        });
}

// ---- 随机树对拍用的上下文：动作和条件的结果由 (节点, 第几次调用) 决定，两种实现看到的完全一样 ----
struct FuzzCtx {
    std::vector<int> calls;  // 每个叶子被调用的次数
    std::vector<int> log;    // +id 调用，-id 中止
};
static uint32_t mix(uint32_t a, uint32_t b) {
    uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA77u;
    h ^= h >> 15, h *= 0x2C1B3C6Du, h ^= h >> 12;
    return h;
}

static int randomTree(BtDef<FuzzCtx>& def, Pcg32& r, int depth) {
    int leafChance = depth >= 4 ? 100 : 35;
    if (int(r.below(100)) < leafChance) {
        int id = static_cast<int>(def.nodes().size()) + 1;
        if (r.below(3) == 0)
            return def.condition("c", [id](const FuzzCtx& c) { return mix(id, c.calls[0]) % 2 == 0; });
        return def.action(
            "a",
            [id](FuzzCtx& c) {
                int k = c.calls[id]++;
                c.log.push_back(id);
                uint32_t h = mix(id, k) % 10;
                return h < 4 ? Status::Running : h < 7 ? Status::Success : Status::Failure;
            },
            [id](FuzzCtx& c) { c.log.push_back(-id); });
    }
    int kind = r.below(8);
    auto kids = [&] {
        std::vector<int> v;
        int n = 1 + r.below(4);
        for (int i = 0; i < n; ++i) v.push_back(randomTree(def, r, depth + 1));
        return v;
    };
    switch (kind) {
        case 0: return def.sequence("s", kids(), true);
        case 1: return def.sequence("s", kids(), false);
        case 2: return def.selector("?", kids(), true);
        case 3: return def.selector("?", kids(), false);
        case 4: {
            auto v = kids();
            return def.parallel("=", v, 1 + r.below(static_cast<uint32_t>(v.size())));
        }
        case 5: return def.inverter(randomTree(def, r, depth + 1));
        case 6: return r.below(2) ? def.repeat(randomTree(def, r, depth + 1), 1 + r.below(3)) : def.retry(randomTree(def, r, depth + 1), 1 + r.below(3));
        default: return def.alwaysSucceed(randomTree(def, r, depth + 1));
    }
}

int main() {
    // ---- 顺序与选择的真值表 ----
    for (int a = 0; a < 2; ++a)
        for (int b = 0; b < 2; ++b) {
            BtDef<Log> def;
            int ca = def.condition("A", [a](const Log&) { return a == 1; });
            int cb = def.condition("B", [b](const Log&) { return b == 1; });
            int seq = def.sequence("seq", {ca, cb});
            int sel = def.selector("sel", {ca, cb});
            Log l;
            def.setRoot(seq);
            Bt<Log> t1(def);
            CHECK((t1.tick(l) == Status::Success) == (a && b));
            def.setRoot(sel);
            Bt<Log> t2(def);
            CHECK((t2.tick(l) == Status::Success) == (a || b));
        }

    // ---- 有记忆的顺序：从运行中的子节点继续，前面成功的不再执行 ----
    {
        BtDef<Log> def;
        int ca = 0, cb = 0;
        int A = timed(def, "A", 0, Status::Success, &ca), B = timed(def, "B", 2, Status::Success, &cb);
        def.sequence("seq", {A, B}, true);
        Bt<Log> t(def);
        Log l;
        CHECK(t.tick(l) == Status::Running && l.events == "AB");
        CHECK(t.tick(l) == Status::Running && l.events == "ABB");   // A 不再执行
        CHECK(t.tick(l) == Status::Success && l.events == "ABBB");
        CHECK(t.tick(l) == Status::Running && l.events == "ABBBAB");  // 完成后从头开始
    }

    // ---- 响应式顺序：条件每次都重新检查，失败时中止运行中的动作 ----
    {
        BtDef<Log> def;
        int cw = 0;
        int cond = def.condition("C", [](const Log& l) { return l.flag; });
        int W = timed(def, "W", 100, Status::Success, &cw);
        def.sequence("guarded", {cond, W}, false);
        Bt<Log> t(def);
        Log l;
        l.flag = true;
        CHECK(t.tick(l) == Status::Running && t.tick(l) == Status::Running && l.events == "WW");
        l.flag = false;
        CHECK(t.tick(l) == Status::Failure && l.events == "WW!W");  // 恰好中止一次
        CHECK(t.tick(l) == Status::Failure && l.events == "WW!W");  // 已经停了，不会再中止
    }

    // ---- 响应式选择：高优先级分支变得可行时，打断低优先级的运行中分支 ----
    {
        BtDef<Log> def;
        int ce = 0, cw = 0;
        int hungry = def.condition("hungry", [](const Log& l) { return l.flag; });
        int eat = def.sequence("eat", {hungry, timed(def, "E", 1, Status::Success, &ce)});
        int work = timed(def, "W", 100, Status::Success, &cw);
        def.selector("root", {eat, work}, false);
        Bt<Log> t(def);
        Log l;
        CHECK(t.tick(l) == Status::Running && l.events == "W");
        l.flag = true;
        CHECK(t.tick(l) == Status::Running && l.events == "WE!W");  // 开始吃，工作被中止
        CHECK(t.tick(l) == Status::Success && l.events == "WE!WE");
        l.flag = false;
        CHECK(t.tick(l) == Status::Running && l.events == "WE!WEW");  // 工作从头开始
        CHECK(cw == 1);
    }

    // ---- 有记忆的选择不会被打断 ----
    {
        BtDef<Log> def;
        int ce = 0, cw = 0;
        int hungry = def.condition("hungry", [](const Log& l) { return l.flag; });
        int eat = def.sequence("eat", {hungry, timed(def, "E", 1, Status::Success, &ce)});
        def.selector("root", {eat, timed(def, "W", 2, Status::Success, &cw)}, true);
        Bt<Log> t(def);
        Log l;
        t.tick(l);
        l.flag = true;
        CHECK(t.tick(l) == Status::Running && l.events == "WW");  // 仍在工作
        CHECK(t.tick(l) == Status::Success && l.events == "WWW");
    }

    // ---- 装饰节点 ----
    {
        BtDef<Log> def;
        int c1 = 0, c2 = 0;
        int ok = timed(def, "S", 0, Status::Success, &c1), bad = timed(def, "F", 0, Status::Failure, &c2);
        int rep = def.repeat(ok, 3), ret = def.retry(bad, 3), inv = def.inverter(bad), always = def.alwaysSucceed(bad);
        Log l;
        def.setRoot(rep);
        Bt<Log> t1(def);
        CHECK(t1.tick(l) == Status::Running && t1.tick(l) == Status::Running && t1.tick(l) == Status::Success);
        def.setRoot(ret);
        Bt<Log> t2(def);
        CHECK(t2.tick(l) == Status::Running && t2.tick(l) == Status::Running && t2.tick(l) == Status::Failure);
        def.setRoot(inv);
        Bt<Log> t3(def);
        CHECK(t3.tick(l) == Status::Success);
        def.setRoot(always);
        Bt<Log> t4(def);
        CHECK(t4.tick(l) == Status::Success);
    }

    // ---- 并行：两个成功就成功，并中止仍在运行的第三个 ----
    {
        BtDef<Log> def;
        int ca = 0, cb = 0, cc = 0;
        def.parallel("par", {timed(def, "A", 0, Status::Success, &ca), timed(def, "B", 1, Status::Success, &cb),
                             timed(def, "C", 5, Status::Success, &cc)},
                     2);
        Bt<Log> t(def);
        Log l;
        CHECK(t.tick(l) == Status::Running && l.events == "ABC");
        CHECK(t.tick(l) == Status::Success && l.events == "ABCABC!C");
    }

    // ---- 随机树：与面向对象的独立实现逐事件对拍 ----
    Pcg32 rng(31);
    long events = 0, haltsSeen = 0;
    for (int trial = 0; trial < 3000; ++trial) {
        BtDef<FuzzCtx> def;
        int root = randomTree(def, rng, 0);
        def.setRoot(root);
        size_t leaves = def.nodes().size() + 2;
        FuzzCtx a{std::vector<int>(leaves, 0), {}}, b = a;
        Bt<FuzzCtx> flat(def);
        auto tree = oop::build(def, root);
        bool same = true;
        for (int tick = 0; tick < 40 && same; ++tick) {
            a.calls[0] = b.calls[0] = tick;  // 条件的结果随 tick 变化
            Status sa = flat.tick(a), sb = tree->tick(b);
            same = sa == sb && a.log == b.log;
        }
        CHECK(same);
        events += static_cast<long>(a.log.size());
        for (int e : a.log) haltsSeen += e < 0;
    }
    std::printf("random trees: %ld events compared (%ld halts)\n", events, haltsSeen);
    CHECK(haltsSeen > 1000);
    return report("bt_test");
}
