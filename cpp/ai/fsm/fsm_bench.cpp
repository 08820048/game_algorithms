// 状态机的每帧开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/fsm/fsm_bench.cpp -o fsm_bench && ./fsm_bench
#include <chrono>
#include <cstdio>
#include <vector>

#include "../../foundations/random/random.h"
#include "guard.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

int main() {
    const int agents = 10000, ticks = 100;
    // 预先生成所有守卫每一帧的输入，计时只包含状态机本身
    Pcg32 rng(3);
    std::vector<GuardCtx> inputs(static_cast<size_t>(agents) * ticks);
    for (int a = 0; a < agents; ++a) {
        GuardCtx in;
        for (int t = 0; t < ticks; ++t) {
            in.t += 0.1f;
            if (rng.nextFloat() < 0.05f) in.seesPlayer = !in.seesPlayer;
            if (in.seesPlayer) in.lastSeen = in.t;
            in.playerDist = std::clamp(in.playerDist + (rng.nextFloat() - 0.5f) * 1.2f, 0.0f, 8.0f);
            in.heardNoise = rng.nextFloat() < 0.03f;
            in.stunEvent = rng.nextFloat() < 0.01f;
            if (rng.nextFloat() < 0.012f) in.health -= 25;
            else if (in.health < 100) in.health += 0.5f;
            inputs[static_cast<size_t>(t) * agents + a] = in;
        }
    }

    auto bench = [&](const char* name, auto&& makeAgents, auto&& tickAgent) {
        double best = 1e30;
        long transitions = 0;
        for (int rep = 0; rep < 5; ++rep) {
            auto state = makeAgents();
            std::vector<GuardCtx> ctx(agents);
            long changed = 0;
            auto t0 = Clock::now();
            for (int t = 0; t < ticks; ++t)
                for (int a = 0; a < agents; ++a) {
                    GuardCtx& c = ctx[a];
                    const GuardCtx& in = inputs[static_cast<size_t>(t) * agents + a];
                    float inv = c.investigateUntil, stun = c.stunUntil;
                    c = in;
                    c.investigateUntil = inv, c.stunUntil = stun;
                    changed += tickAgent(state, a, c);
                }
            best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / (double(agents) * ticks));
            transitions = changed;
        }
        std::printf("  %-36s %7.1f ns per agent per tick  (%ld transitions)\n", name, best, transitions);
    };

    std::printf("%d guards x %d ticks\n", agents, ticks);
    bench("switch", [&] { return std::vector<GuardSwitch>(agents); },
          [](std::vector<GuardSwitch>& v, int a, GuardCtx& c) {
              Guard before = v[a].state;
              v[a].update(c);
              return v[a].state != before ? 1 : 0;
          });
    // 每个守卫各有一份完整的定义（状态、转移、回调都复制一遍）：数据分散在内存各处
    bench("flat, definition per agent", [&] {
              std::vector<HfsmDef<GuardCtx>> defs(agents);
              std::vector<Hfsm<GuardCtx>> v;
              v.reserve(agents);
              GuardCtx dummy;
              for (auto& d : defs) {
                  int init = buildGuardFlat(d);
                  v.emplace_back(d).start(init, dummy);
              }
              return std::pair{std::move(defs), std::move(v)};
          },
          [](auto& p, int a, GuardCtx& c) {
              auto& m = p.second[a];
              int before = m.leaf();
              m.update(c);
              return m.leaf() != before ? 1 : 0;
          });
    bench("hierarchical, definition per agent", [&] {
              std::vector<HfsmDef<GuardCtx>> defs(agents);
              std::vector<Hfsm<GuardCtx>> v;
              v.reserve(agents);
              GuardCtx dummy;
              for (auto& d : defs) {
                  int init = buildGuardHfsm(d).alive;
                  v.emplace_back(d).start(init, dummy);
              }
              return std::pair{std::move(defs), std::move(v)};
          },
          [](auto& p, int a, GuardCtx& c) {
              auto& m = p.second[a];
              int before = m.leaf();
              m.update(c);
              return m.leaf() != before ? 1 : 0;
          });
    // 定义只有一份，所有守卫共享；每个守卫只存自己的当前状态和历史
    static HfsmDef<GuardCtx> flatDef, hierDef;
    static int flatInit = buildGuardFlat(flatDef), hierInit = buildGuardHfsm(hierDef).alive;
    bench("flat, shared definition", [&] {
              std::vector<Hfsm<GuardCtx>> v(agents, Hfsm<GuardCtx>(flatDef));
              GuardCtx dummy;
              for (auto& m : v) m.start(flatInit, dummy);
              return v;
          },
          [](std::vector<Hfsm<GuardCtx>>& v, int a, GuardCtx& c) {
              int before = v[a].leaf();
              v[a].update(c);
              return v[a].leaf() != before ? 1 : 0;
          });
    bench("hierarchical, shared definition", [&] {
              std::vector<Hfsm<GuardCtx>> v(agents, Hfsm<GuardCtx>(hierDef));
              GuardCtx dummy;
              for (auto& m : v) m.start(hierInit, dummy);
              return v;
          },
          [](std::vector<Hfsm<GuardCtx>>& v, int a, GuardCtx& c) {
              int before = v[a].leaf();
              v[a].update(c);
              return v[a].leaf() != before ? 1 : 0;
          });
}
