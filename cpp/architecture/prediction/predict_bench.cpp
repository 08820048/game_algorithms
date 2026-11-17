// 客户端预测、回滚与延迟补偿的测量（除 cost 以外全部是模拟时间，与机器负载无关）
//   c++ -std=c++20 -O2 -ffp-contract=off -Icpp/common cpp/architecture/prediction/predict_bench.cpp -o /tmp/predb && /tmp/predb [section]
//   section: predict | crowd | rollback | period | cost | lagcomp | cap（默认全部）

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "predict.h"

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
constexpr int kTicks = 3600;
static const char* kModes[] = {"none", "predict", "reconcile", "smooth"};

struct OwnStats {
    double avgErr = 0, maxErr = 0, maxStep = 0;
    long teleports = 0, corrections = 0;
    double corrAvg = 0, corrMax = 0;
};
// 玩家 0 画出的自己 vs 服务器处理完同一个输入后的位置；一帧移动超过 4 像素（正常最多 2√2）算一次跳动
static OwnStats ownStats(const PredictSession& s) {
    OwnStats o;
    const auto& c = s.clients()[0];
    long n = 0;
    for (size_t t = 1; t < c.shownHistory.size(); ++t) {
        Field::Player p;
        auto [x, y] = c.shownHistory[t];
        if (s.serverAfter(0, uint32_t(t), &p)) {
            double e = std::hypot(x - p.x.toDouble(), y - p.y.toDouble());
            o.avgErr += e, o.maxErr = std::max(o.maxErr, e), ++n;
        }
        double step = std::hypot(x - c.shownHistory[t - 1].first, y - c.shownHistory[t - 1].second);
        o.maxStep = std::max(o.maxStep, step);
        o.teleports += step > 4;
    }
    o.avgErr /= std::max(1L, n);
    o.corrections = c.corrections, o.corrAvg = c.correctionSum / std::max(1L, c.corrections), o.corrMax = c.correctionMax;
    return o;
}

static double pressToSee(Mode m, LinkConfig link) {
    PredictConfig c;
    c.mode = m, c.link = link, c.pressAt = 600;
    c.link.loss = 0;
    PredictSession s(c);
    double x0 = 0;
    long moved = -1;
    for (int t = 0; t < 900; ++t) {
        s.tick();
        double x = s.clients()[0].shownX;
        if (t == 599) x0 = x;
        if (t >= 600 && moved < 0 && std::fabs(x - x0) > 1e-9) moved = t;
    }
    return (moved - 600) * 1000 / 60.0;
}

static void predict() {
    std::printf("== own character, 4 players, 1 minute: press-to-see / error vs server / teleports / corrections ==\n");
    std::printf("%-32s %-10s %8s %9s %9s %10s %9s %8s %9s\n", "network", "mode", "press ms", "avg err", "max err", "pops>4px", "max step", "corr", "corr avg");
    for (const NetCase& n : kNets) {
        for (int m = 0; m < 4; ++m) {
            PredictConfig c;
            c.mode = Mode(m), c.link = n.link;
            PredictSession s(c);
            for (int t = 0; t < kTicks; ++t) s.tick();
            OwnStats o = ownStats(s);
            std::printf("%-32s %-10s %8.0f %9.2f %9.2f %10ld %9.2f %8ld %9.2f\n", n.name, kModes[m], pressToSee(Mode(m), n.link), o.avgErr, o.maxErr, o.teleports,
                        o.maxStep, o.corrections, o.corrAvg);
        }
    }
}

static void crowd() {
    std::printf("\n== mispredictions caused by other players (reconcile), 1 minute, all clients summed ==\n");
    std::printf("%-32s %-8s %12s %12s %12s %16s %16s\n", "network", "players", "corrections", "avg px", "max px", "pops (rec.)", "pops (smooth)");
    for (int ni : {0, 1, 3})
        for (int players : {1, 4, 8, 16}) {
            long corr = 0, tele[2] = {};
            double sum = 0, mx = 0;
            for (int k = 0; k < 2; ++k) {
                PredictConfig c;
                c.players = players, c.link = kNets[ni].link, c.mode = k ? Mode::Smooth : Mode::Reconcile;
                PredictSession s(c);
                for (int t = 0; t < kTicks; ++t) s.tick();
                for (const auto& cl : s.clients()) {
                    if (k == 0) corr += cl.corrections, sum += cl.correctionSum, mx = std::max(mx, cl.correctionMax);
                    for (size_t t = 1; t < cl.shownHistory.size(); ++t)
                        tele[k] += std::hypot(cl.shownHistory[t].first - cl.shownHistory[t - 1].first, cl.shownHistory[t].second - cl.shownHistory[t - 1].second) > 4;
                }
            }
            std::printf("%-32s %-8d %12ld %12.2f %12.2f %16ld %16ld\n", kNets[ni].name, players, corr, sum / std::max(1L, corr), mx, tele[0], tele[1]);
        }
}

static void rollback() {
    std::printf("\n== rollback, 4 peers, 200 units, window 8, 1 minute ==\n");
    std::printf("%-32s %3s %7s %7s %9s %10s %11s %9s %10s %9s\n", "network", "D", "speed", "stalls", "wrong %", "rollbk/s", "resim/tick", "max/tick", "jump avg", "jump max");
    for (const NetCase& n : kNets)
        for (int d : {0, 1, 2, 3, 4, 6, 8}) {
            RollbackConfig c;
            c.delay = d, c.link = n.link;
            RollbackSession s(c);
            for (int t = 0; t < kTicks; ++t) s.tick();
            long stalls = 0, maxSteps = 0, rb = 0, resim = 0, guessed = 0, wrong = 0, jumps = 0;
            double jsum = 0, jmax = 0;
            for (auto& p : s.peers()) {
                stalls = std::max(stalls, p.stallTicks), maxSteps = std::max(maxSteps, p.maxStepsPerTick);
                rb += p.rollbacks, resim += p.resimFrames, guessed += p.guessed, wrong += p.wrongGuesses;
                jumps += p.jumps, jsum += p.jumpSum, jmax = std::max(jmax, p.jumpMax);
            }
            std::printf("%-32s %3d %6.1f%% %7ld %8.1f%% %10.2f %11.2f %9ld %10.2f %9.2f\n", n.name, d, 100.0 * s.peers()[0].sim.frame / kTicks, stalls,
                        100.0 * wrong / std::max(1L, guessed), rb / 4.0 / 60, resim / 4.0 / kTicks, maxSteps, jsum / std::max(1L, jumps), jmax);
        }
}

static void period() {
    std::printf("\n== how often inputs change (regional network, D = 0, window 8) ==\n");
    std::printf("%-12s %9s %10s %11s %9s %10s %9s\n", "period", "wrong %", "rollbk/s", "resim/tick", "max/tick", "jump avg", "jump max");
    for (int per : {5, 10, 20, 40, 120}) {
        RollbackConfig c;
        c.botPeriod = per, c.link = kNets[1].link;
        RollbackSession s(c);
        for (int t = 0; t < kTicks; ++t) s.tick();
        long maxSteps = 0, rb = 0, resim = 0, guessed = 0, wrong = 0, jumps = 0;
        double jsum = 0, jmax = 0;
        for (auto& p : s.peers()) {
            maxSteps = std::max(maxSteps, p.maxStepsPerTick), rb += p.rollbacks, resim += p.resimFrames, guessed += p.guessed, wrong += p.wrongGuesses;
            jumps += p.jumps, jsum += p.jumpSum, jmax = std::max(jmax, p.jumpMax);
        }
        char name[32];
        std::snprintf(name, sizeof name, "%d frames", per);
        std::printf("%-12s %8.1f%% %10.2f %11.2f %9ld %10.2f %9.2f\n", name, 100.0 * wrong / std::max(1L, guessed), rb / 4.0 / 60, resim / 4.0 / kTicks, maxSteps,
                    jsum / std::max(1L, jumps), jmax);
    }
}

// 真实的 CPU 时间（受机器负载影响）：一帧模拟、保存一次状态各要多久，最坏的 tick（窗口 8：重算 8 帧 + 新算 1 帧）要多久
static void cost() {
    std::printf("\n== cost per frame (wall clock, noisy) ==\n");
    std::printf("%-8s %12s %12s %22s %22s\n", "units", "step us", "save us", "worst tick (9 steps)", "share of 16.7 ms");
    for (int units : {200, 1000, 5000, 20000}) {
        Arena a = Arena::make(1, 4, units), copy;
        std::vector<Input> in(4);
        int reps = std::max(20, 400000 / units);
        auto t0 = std::chrono::steady_clock::now();
        for (int f = 0; f < reps; ++f) {
            for (int p = 0; p < 4; ++p) in[size_t(p)] = botInput(1, p, uint32_t(f));
            a.step(in.data());
        }
        double step = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / reps;
        t0 = std::chrono::steady_clock::now();
        uint64_t sink = 0;
        for (int f = 0; f < reps; ++f) copy = a, sink += copy.units[size_t(f) % copy.units.size()].x.raw;
        double save = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / reps;
        double worst = 9 * (step + save);
        std::printf("%-8d %12.2f %12.2f %19.0f us %21.1f%%%s\n", units, step, save, worst, 100 * worst / 16667, sink == 42 ? " " : "");
    }
}

static void lagcomp() {
    std::printf("\n== lag compensation: shooter interpolates 100 ms, aims with 3 px error; target 16 px wide, 120 px/s, 10 minutes ==\n");
    std::printf("%-10s %8s %10s %12s %12s %14s %14s %14s\n", "one-way", "shots", "hit (now)", "hit (rewind)", "rewind ms", "behind cover", "cover avg ms", "cover max ms");
    for (double lat : {0.0, 15.0, 40.0, 75.0, 125.0, 200.0}) {
        LagCompConfig c;
        c.link = {lat, 0, 0};
        LagCompSession s(c);
        for (int t = 0; t < kTicks * 10; ++t) s.tick();
        long hn = 0, hb = 0, cover = 0;
        double rew = 0, csum = 0, cmax = 0;
        for (auto& r : s.results()) {
            hn += r.hitNow, hb += r.hitBack, cover += r.behindCover, rew += (r.serverFrame - r.shot.renderFrame) * 1000 / 60;
            if (r.behindCover) csum += r.coverMs, cmax = std::max(cmax, r.coverMs);
        }
        double n = double(s.results().size());
        std::printf("%-7.0f ms %8.0f %9.1f%% %11.1f%% %12.0f %13.1f%% %14.0f %14.0f\n", lat, n, 100 * hn / n, 100 * hb / n, rew / n, 100.0 * cover / std::max(1L, hb),
                    csum / std::max(1L, cover), cmax);
    }
}

static void cap() {
    std::printf("\n== rewind cap: hit rate with compensation ==\n");
    std::printf("%-10s", "one-way");
    std::printf("%14s", "no cap");
    for (double capMs : {250.0, 200.0, 150.0}) std::printf("    cap %3.0f ms", capMs);
    std::printf("\n");
    for (double lat : {15.0, 40.0, 75.0, 125.0, 200.0}) {
        std::printf("%-7.0f ms", lat);
        for (double capMs : {1e9, 250.0, 200.0, 150.0}) {
            LagCompConfig c;
            c.link = {lat, 0, 0}, c.maxRewindMs = capMs;
            LagCompSession s(c);
            for (int t = 0; t < kTicks * 10; ++t) s.tick();
            long hb = 0;
            for (auto& r : s.results()) hb += r.hitBack;
            std::printf("%13.1f%%", 100.0 * hb / double(s.results().size()));
        }
        std::printf("\n");
    }
}

int main(int argc, char** argv) {
    auto want = [&](const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; };
    if (want("predict")) predict();
    if (want("crowd")) crowd();
    if (want("rollback")) rollback();
    if (want("period")) period();
    if (want("cost")) cost();
    if (want("lagcomp")) lagcomp();
    if (want("cap")) cap();
}
