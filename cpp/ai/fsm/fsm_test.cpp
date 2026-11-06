#include "fsm.h"

#include <cstdio>
#include <map>
#include <string>

#include "../../foundations/random/random.h"
#include "check.h"
#include "guard.h"

using namespace ga;

// 记录进入、退出的顺序
struct LogCtx {
    std::string log;
    int signal = 0;
};

int main() {
    // ---- 进入 / 退出的顺序、外层优先、外部自转移、历史 ----
    {
        HfsmDef<LogCtx> m;
        auto named = [&](const char* name, int parent) {
            std::string n = name;
            return m.addState(name, parent, [n](LogCtx& c) { c.log += "+" + n; }, {}, [n](LogCtx& c) { c.log += "-" + n; });
        };
        int a = named("A", -1), a1 = named("A1", a), x = named("x", a1), y = named("y", a1), a2 = named("A2", a), z = named("z", a2);
        int b = named("B", -1);
        m.addTransition(a, b, [](const LogCtx& c) { return c.signal == 9; });        // 外层：优先于内层
        m.addTransition(x, y, [](const LogCtx& c) { return c.signal == 1 || c.signal == 9; });
        m.addTransition(y, z, [](const LogCtx& c) { return c.signal == 2; });
        m.addTransition(a2, a1, [](const LogCtx& c) { return c.signal == 3; });       // 回到 A1：初始子状态 x
        m.addTransition(z, a1, [](const LogCtx& c) { return c.signal == 4; }, true);  // 历史：回到上次离开 A1 时的 y
        m.addTransition(y, a1, [](const LogCtx& c) { return c.signal == 5; });        // 目标是自己的祖先：退出再进入
        m.addTransition(b, a, [](const LogCtx& c) { return c.signal == 6; }, true);
        LogCtx c;
        Hfsm<LogCtx> run(m);
        run.start(a, c);
        CHECK(c.log == "+A+A1+x" && run.leaf() == x);
        auto fire = [&](int s) {
            c.log.clear();
            c.signal = s;
            run.update(c);
            return c.log;
        };
        CHECK(fire(1) == "-x+y");
        CHECK(fire(2) == "-y-A1+A2+z");  // 最近公共祖先是 A：A 不退出
        CHECK(fire(3) == "-z-A2+A1+x");
        CHECK(fire(1) == "-x+y");
        CHECK(fire(5) == "-y-A1+A1+x");  // 外部自转移
        CHECK(fire(1) == "-x+y");
        CHECK(fire(2) == "-y-A1+A2+z");
        CHECK(fire(4) == "-z-A2+A1+y");   // 历史转移恢复 y
        CHECK(fire(9) == "-y-A1-A+B");    // x→y 的条件也满足，但外层的 A→B 优先
        CHECK(run.leaf() == b);
        CHECK(fire(6) == "-B+A+A1+y");  // 深层历史：A 记得 A1，A1 记得 y
        CHECK(fire(0).empty());         // 没有转移满足：什么都不做
    }

    // ---- 守卫：三种实现的状态序列完全相同 ----
    HfsmDef<GuardCtx> hierDef, flatDef;
    GuardStates gs = buildGuardHfsm(hierDef);
    int flatInitial = buildGuardFlat(flatDef);
    std::printf("hierarchical: %zu states (%d leaves), %zu transitions; flat: %zu states, %zu transitions\n", hierDef.states().size(), 7,
                hierDef.transitions().size(), flatDef.states().size(), flatDef.transitions().size());
    CHECK(hierDef.transitions().size() == 12 && flatDef.transitions().size() == 33);

    Pcg32 rng(12);
    std::map<std::string, long> visits;
    std::map<std::string, long> resumedInto;
    long ticks = 0, mismatches = 0;
    for (int run = 0; run < 300; ++run) {
        GuardCtx ch, cf, cs;  // 三份独立的上下文（计时器由各自的状态机写入）
        Hfsm<GuardCtx> hier(hierDef), flat(flatDef);
        hier.start(gs.alive, ch);
        flat.start(flatInitial, cf);
        GuardSwitch sw;
        GuardCtx input;
        input.health = 100;
        std::string prev = "Patrol";
        for (int tick = 0; tick < 1500; ++tick) {
            // 随机但有持续性的输入
            input.t += 0.1f;
            if (rng.nextFloat() < 0.05f) input.seesPlayer = !input.seesPlayer;
            if (input.seesPlayer) input.lastSeen = input.t;
            input.playerDist = std::clamp(input.playerDist + (rng.nextFloat() - 0.5f) * 1.2f, 0.0f, 8.0f);
            input.heardNoise = rng.nextFloat() < 0.03f;
            input.stunEvent = rng.nextFloat() < 0.01f;
            float r = rng.nextFloat();
            if (r < 0.012f) input.health -= 25;             // 挨了一下
            else if (input.health < 100) input.health += 0.5f;  // 慢慢回血
            if (rng.nextFloat() < 0.0002f) input.health = 0;   // 偶尔直接被击杀
            for (GuardCtx* c : {&ch, &cf, &cs}) {
                float inv = c->investigateUntil, stun = c->stunUntil;
                *c = input;
                c->investigateUntil = inv, c->stunUntil = stun;
            }
            hier.update(ch);
            flat.update(cf);
            sw.update(cs);
            std::string h = hier.leafName(), f = flat.leafName(), s = guardName(sw.state);
            if (h != f || h != s) ++mismatches;
            ++visits[h];
            if (prev == "Stunned" && h != "Stunned" && h != "Dead") ++resumedInto[h];
            prev = h;
            ++ticks;
        }
    }
    CHECK(mismatches == 0);
    std::printf("%ld ticks, %ld mismatches. visits:", ticks, mismatches);
    for (auto& [name, n] : visits) std::printf(" %s %ld", name.c_str(), n);
    std::printf("\nresumed after stun into:");
    for (auto& [name, n] : resumedInto) std::printf(" %s %ld", name.c_str(), n);
    std::printf("\n");
    for (const char* name : {"Patrol", "Investigate", "Chase", "Attack", "Flee", "Stunned", "Dead"}) CHECK(visits[name] > 0);
    for (const char* name : {"Patrol", "Investigate", "Chase", "Attack", "Flee"}) CHECK(resumedInto[name] > 0);  // 每种历史恢复都发生过
    return report("fsm_test");
}
