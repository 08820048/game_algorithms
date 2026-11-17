// 帧同步与状态同步的测量（全部是模拟时间，与机器负载无关）
//   c++ -std=c++20 -O2 -Icpp/common cpp/architecture/netsync/net_bench.cpp -o /tmp/netb && /tmp/netb [section]
//   section: lockstep | batch | feel | bandwidth | interp | join（默认全部）

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "net.h"

using namespace ga;
using namespace ga::net;

struct NetCase {
    const char* name;
    LinkConfig link;
};
static const NetCase kNets[] = {
    {"LAN 5+2 ms", {5, 2, 0}},
    {"regional 30+10 ms 1%", {30, 10, 0.01}},
    {"cross-country 75+30 ms 2%", {75, 30, 0.02}},
    {"intercontinental 125+50 ms 3%", {125, 50, 0.03}},
};
constexpr int kTicks = 3600;  // 1 分钟

static void lockstep() {
    std::printf("== lockstep, 4 peers, 3600 ticks: game speed (frames / ticks) / stalled ticks of the worst peer, vs input delay D ==\n");
    for (const NetCase& n : kNets) {
        std::printf("%-32s", n.name);
        int first = -1;
        for (int d : {1, 2, 3, 4, 6, 8, 10, 12, 15}) {
            LockstepConfig c;
            c.delay = d, c.link = n.link;
            LockstepSession s(c);
            for (int t = 0; t < kTicks; ++t) s.tick();
            const auto& p = s.peers()[0];
            double speed = 100.0 * p.sim.frame / kTicks;
            long stalls = 0;
            for (auto& q : s.peers()) stalls = std::max(stalls, q.stallTicks);
            std::printf(" D%-2d %5.1f%%/%-4ld", d, speed, stalls);
            if (first < 0 && stalls == 0) first = d;
        }
        std::printf("\n%-32s smallest D with no stall: %d (%.0f ms); worst one-way %.0f ms = %.1f ticks\n", "", first, first * 1000 / 60.0,
                    n.link.latencyMs + n.link.jitterMs, (n.link.latencyMs + n.link.jitterMs) * 60 / 1000);
    }
}

static void batch() {
    std::printf("\n== lockstep batching (regional network): send every k ticks ==\n");
    std::printf("%-6s %14s %16s %18s %22s\n", "k", "packets/s/peer", "bytes/s/peer", "header share", "smallest D, no stall");
    for (int k : {1, 2, 3, 6}) {
        int best = -1;
        double bytes = 0, packets = 0, payload = 0;
        for (int d = 1; d <= 20 && best < 0; ++d) {
            LockstepConfig c;
            c.delay = d, c.sendEvery = k, c.link = kNets[1].link;
            LockstepSession s(c);
            for (int t = 0; t < kTicks; ++t) s.tick();
            bool ok = true;
            for (auto& p : s.peers()) ok &= p.stallTicks == 0;
            if (ok || d == 20) {
                best = d;
                bytes = double(s.bytesSentBy(0)) / 60, packets = double(s.packetsSentBy(0)) / 60;
                payload = bytes - packets * kUdpIpHeader;
            }
        }
        std::printf("%-6d %14.0f %16.0f %17.0f%% %15d (%.0f ms)\n", k, packets, bytes, 100 * (1 - payload / bytes), best, best * 1000 / 60.0);
    }
}

// 按键到看见自己的角色动：帧同步 = 输入延迟；状态同步（不预测）= 上行 + 等快照 + 下行 + 插值延迟
static void feel() {
    std::printf("\n== press-to-see latency for your own character (ms) ==\n");
    std::printf("%-32s %18s %22s %22s\n", "network", "lockstep (min D)", "state, interp 50 ms", "state, interp 100 ms");
    for (const NetCase& n : kNets) {
        LinkConfig noLoss = n.link;
        noLoss.loss = 0;  // 只看延迟本身（丢包时偶尔更晚）
        long lock = -1;
        for (int d = 1; d <= 20 && lock < 0; ++d) {
            LockstepConfig c;
            c.delay = d, c.link = noLoss, c.pressAt = 600;
            LockstepSession s(c);
            Fx x0;
            long moved = -1;
            bool stalled = false;
            for (int t = 0; t < 900; ++t) {
                s.tick();
                if (t == 599) x0 = s.peers()[0].sim.players[0].x;
                if (t >= 600 && moved < 0 && s.peers()[0].sim.players[0].x != x0) moved = t;
            }
            for (auto& p : s.peers()) stalled |= p.stallTicks > 0;
            if (!stalled) lock = moved - 600;
        }
        double st[2];
        int k = 0;
        for (double interp : {50.0, 100.0}) {
            StateConfig c;
            c.link = noLoss, c.interpMs = interp, c.pressAt = 600;
            StateSession s(c);
            double x0 = 0;
            long moved = -1;
            for (int t = 0; t < 900; ++t) {
                s.tick();
                auto& v = s.clients()[0].players;
                if (v.empty()) continue;
                if (t == 599) x0 = v[0].x;
                if (t >= 600 && moved < 0 && std::fabs(v[0].x - x0) > 1e-9) moved = t;
            }
            st[k++] = (moved - 600) * 1000 / 60.0;
        }
        std::printf("%-32s %18.0f %22.0f %22.0f\n", n.name, lock * 1000 / 60.0, st[0], st[1]);
    }
}

static void bandwidth() {
    std::printf("\n== bandwidth (regional network, 4 players, snapshots at 20 Hz), bytes per second ==\n");
    std::printf("%-8s %16s %16s %16s %16s %12s\n", "units", "lockstep/peer", "full/client", "quantized/cl.", "delta/client", "changed/snap");
    for (int units : {50, 200, 1000, 5000}) {
        LockstepConfig lc;
        lc.units = units, lc.delay = 4, lc.link = kNets[1].link;
        LockstepSession ls(lc);
        StateConfig sc;
        sc.units = units, sc.link = kNets[1].link;
        StateSession ss(sc);
        int changedSum = 0, snaps = 0;
        QState prev;
        for (int t = 0; t < kTicks; ++t) {
            ls.tick(), ss.tick();
            if (ss.server().frame % 3 == 0) {
                QState q = quantize(ss.server());
                if (!prev.units.empty()) {
                    for (size_t i = 0; i < q.units.size(); ++i) changedSum += !(q.units[i] == prev.units[i]);
                    ++snaps;
                }
                prev = q;
            }
        }
        double sec = kTicks / 60.0;
        std::printf("%-8d %16.0f %16.0f %16.0f %16.0f %11.1f%%\n", units, ls.bytesSentBy(0) / sec, ss.encodedBytes(Encoding::Full) / sec / 4,
                    ss.encodedBytes(Encoding::Quantized) / sec / 4, ss.encodedBytes(Encoding::Delta) / sec / 4, 100.0 * changedSum / snaps / units);
    }
}

static void interp() {
    std::printf("\n== interpolation (200 units, 4 players moving 90 px/s): extrapolated frames and error of rendered players ==\n");
    std::printf("%-28s %-8s %10s %12s %12s %12s\n", "network", "rate", "interp ms", "extrapolated", "avg err px", "max err px");
    const LinkConfig nets[] = {{30, 0, 0}, {30, 20, 0}, {30, 20, 0.05}};
    const char* names[] = {"30 ms, no jitter", "30 ms, jitter 20", "30 ms, jitter 20, loss 5%"};
    for (int n = 0; n < 3; ++n)
        for (int every : {6, 3}) {
            for (double ms : {0.0, 33.0, 50.0, 100.0, 150.0}) {
                StateConfig c;
                c.link = nets[n], c.sendEvery = every, c.interpMs = ms;
                StateSession s(c);
                for (int t = 0; t < kTicks; ++t) s.tick();
                const auto& cl = s.clients()[0];
                char rate[16];
                std::snprintf(rate, sizeof rate, "%d Hz", 60 / every);
                std::printf("%-28s %-8s %10.0f %11.1f%% %12.3f %12.2f\n", names[n], rate, ms, 100.0 * cl.extrapolated / cl.rendered,
                            cl.errorSum / double(cl.rendered * 4), cl.errorMax);
            }
        }
}

// 中途加入 / 断线重连：帧同步要拿到完整状态，或者从头重放所有输入
static void join() {
    std::printf("\n== joining a 4-player game after 10 minutes ==\n");
    std::printf("%-8s %18s %22s %22s\n", "units", "full state", "all inputs so far", "replay time (sim)");
    for (int units : {200, 1000, 5000}) {
        Arena a = Arena::make(1, 4, units);
        std::vector<Input> in(4);
        uint32_t frames = 36000;
        double inputs = frames * 4 * sizeof(Input);
        // 重放 10 分钟要多久：先测 600 帧
        auto t0 = std::chrono::steady_clock::now();
        for (uint32_t f = 0; f < 600; ++f) {
            for (int p = 0; p < 4; ++p) in[size_t(p)] = botInput(1, p, f);
            a.step(in.data());
        }
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() * frames / 600;
        std::printf("%-8d %15.1f KB %19.1f KB %19.0f ms\n", units, a.stateBytes() / 1024.0, inputs / 1024, ms);
    }
}

int main(int argc, char** argv) {
    auto want = [&](const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; };
    if (want("lockstep")) lockstep();
    if (want("batch")) batch();
    if (want("feel")) feel();
    if (want("bandwidth")) bandwidth();
    if (want("interp")) interp();
    if (want("join")) join();
}
