// 录像、跳转与撤销的测量
//   c++ -std=c++20 -O2 -Icpp/common cpp/architecture/command/command_bench.cpp -o /tmp/cmdb && /tmp/cmdb [section]
//   section: storage | chaos | seek | undo | drift（默认全部）

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "command.h"

using namespace ga;
using namespace ga::command;

template <class F>
static double ms(F&& f) {
    auto t0 = std::chrono::steady_clock::now();
    f();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// 类似真人的输入：平均每 20 帧（1/3 秒）改变一次
static Recording humanLike(uint64_t seed, int balls, int frames) {
    Recording rec{seed, balls, {}};
    Pcg32 r{seed + 100};
    Input cur;
    for (int f = 0; f < frames; ++f) {
        if (r.next() % 20 == 0) cur = {int8_t(int(r.next() % 3) - 1), int8_t(int(r.next() % 3) - 1), r.next() % 4 == 0};
        rec.inputs.push_back(cur);
    }
    return rec;
}

static void storage() {
    std::printf("== storage for one hour at 60 fps (216000 frames), 40 balls ==\n");
    Recording rec = humanLike(1, 40, 216000);
    auto runs = rec.compress();
    Sim s = Sim::make(1, 40);
    double raw = double(rec.inputs.size() * sizeof(Input)), rle = double(runs.size() * (sizeof(Input) + sizeof(uint16_t)));
    std::printf("inputs raw          %10.0f KB\n", raw / 1024);
    std::printf("inputs run-length   %10.0f KB  (%zu runs)\n", rle / 1024, runs.size());
    std::printf("state per frame     %10.0f KB  (%zu bytes per state)\n", double(s.bytes()) * 216000 / 1024, s.bytes());
    for (int balls : {200, 1000})
        std::printf("state, %4d balls   %10.0f MB\n", balls, double(Sim::make(1, balls).bytes()) * 216000 / 1048576);
}

// 初始状态的微小差别多久会放大到肉眼可见
static void chaos() {
    std::printf("\n== chaos: perturb one ball's x at frame 0, frames until the worst ball is off by > 1 px / > 9 px (radius) ==\n");
    std::printf("%-14s %10s %10s\n", "perturbation", "> 1 px", "> 9 px");
    Recording rec = humanLike(2, 40, 20000);
    auto run = [&](auto perturb) {
        Sim a = Sim::make(2, 40), b = a;
        perturb(b);
        int f1 = -1, f9 = -1;
        for (int f = 0; f < 20000 && f9 < 0; ++f) {
            a.step(rec.inputs[f]), b.step(rec.inputs[f]);
            float d = a.maxDistance(b);
            if (f1 < 0 && d > 1) f1 = f;
            if (d > Sim::kR) f9 = f;
        }
        return std::pair{f1, f9};
    };
    for (float eps : {1.0f, 1e-1f, 1e-2f, 1e-3f, 1e-4f, 1e-5f}) {
        auto [f1, f9] = run([eps](Sim& s) { s.balls[0].p.x += eps; });
        std::printf("%-14g %10d %10d\n", eps, f1, f9);
    }
    auto [f1, f9] = run([](Sim& s) { s.balls[0].p.x = std::nextafter(s.balls[0].p.x, 1e9f); });
    std::printf("%-14s %10d %10d\n", "1 ulp", f1, f9);
    // 太小的扰动被舍入吃掉：x ≈ 几百，float 的间隔约 3e-5
    auto [g1, g9] = run([](Sim& s) { s.balls[0].p.x += 1e-6f; });
    std::printf("%-14s %10d %10d   (x += 1e-6 rounds back to the same float)\n", "1e-6", g1, g9);
}

static void seek() {
    std::printf("\n== seeking in a one-hour replay (40 balls): keyframe every K frames ==\n");
    Recording rec = humanLike(3, 40, 216000);
    double stepUs = ms([&] { rec.replay(20000); }) * 1000 / 20000;
    std::printf("one simulation step: %.2f us\n", stepUs);
    std::printf("%-8s %12s %12s %14s %14s\n", "K", "keyframes", "memory", "avg seek ms", "max seek ms");
    for (int k : {1, 10, 60, 600, 3600, 36000}) {
        Seeker sk(rec, k);
        Pcg32 r{4};
        double total = 0, worst = 0;
        const int seeks = 200;
        for (int i = 0; i < seeks; ++i) {
            size_t n = r.next() % 216001;
            double t = ms([&] { volatile uint32_t f = sk.seek(n).frame; (void)f; });
            total += t, worst = std::max(worst, t);
        }
        std::printf("%-8d %12zu %10.1f MB %14.3f %14.3f\n", k, sk.keyframes(), double(sk.bytes()) / 1048576, total / seeks, worst);
    }
}

// 瓦片编辑器 256×256，2000 笔，每笔约 60 次绘制（经常重复画同一格）
static void undo() {
    std::printf("\n== undo: 256x256 tile map (64 KB), 2000 strokes of ~60 edits ==\n");
    std::printf("%-26s %12s %16s\n", "strategy", "memory", "avg undo us");
    Pcg32 r{5};
    std::vector<std::vector<Edit>> strokes;
    for (int s = 0; s < 2000; ++s) {
        std::vector<Edit> e;
        uint32_t c = r.next() % 65536;
        for (int i = 0; i < 60; ++i) {
            int dir = int(r.next() % 4);
            c = (c + (dir == 0 ? 1 : dir == 1 ? 65535 : dir == 2 ? 256 : 65280)) % 65536;
            e.push_back({c, uint8_t(1 + r.next() % 7)});
        }
        strokes.push_back(e);
    }
    {
        Grid g(256, 256);
        UndoStack us;
        for (auto& e : strokes) us.apply(g, e, true);
        size_t b = us.bytes();
        double t = ms([&] { while (us.undo(g)) {} });
        std::printf("%-26s %10.1f KB %16.3f\n", "command (old values)", double(b) / 1024, t * 1000 / 2000);
    }
    {
        Grid g(256, 256);
        UndoStack us;
        for (auto& e : strokes) us.apply(g, e, false);
        std::printf("%-26s %10.1f KB %16s\n", "command, not merged", double(us.bytes()) / 1024, "-");
    }
    {
        Grid g(256, 256);
        SnapshotStack ss(g);
        for (auto& e : strokes) ss.apply(g, e);
        size_t b = ss.bytes();
        double t = ms([&] { while (ss.undo(g)) {} });
        std::printf("%-26s %10.1f KB %16.3f\n", "full snapshot", double(b) / 1024, t * 1000 / 2000);
    }
    for (int k : {16, 128}) {
        Grid g(256, 256);
        ReplayStack rs(g, k);
        for (auto& e : strokes) rs.apply(g, e);
        size_t b = rs.bytes();
        long replayed = 0;
        double t = ms([&] { while (rs.undo(g, &replayed)) {} });
        char name[40];
        std::snprintf(name, sizeof name, "replay, checkpoint / %d", k);
        std::printf("%-26s %10.1f KB %16.3f   (%.1f strokes replayed per undo)\n", name, double(b) / 1024, t * 1000 / 2000, replayed / 2000.0);
    }
}

// 用"反向操作"撤销 vs 记住旧值：旋转 θ 再旋转 −θ
static void drift() {
    std::printf("\n== undo by inverse operation: rotate by 0.1 rad then by -0.1 rad, repeated ==\n");
    std::printf("%-12s %16s %16s\n", "cycles", "inverse error", "stored old value");
    for (long cycles : {1L, 1000L, 1000000L}) {
        Vec2 p{100, 0}, q = p;
        for (long i = 0; i < cycles; ++i) {
            p = rotate(p, 0.1f), p = rotate(p, -0.1f);
            Vec2 old = q;
            q = rotate(q, 0.1f);
            q = old;  // 撤销：写回旧值
        }
        std::printf("%-12ld %16.3g %16.3g\n", cycles, double(length(p - Vec2{100, 0})), double(length(q - Vec2{100, 0})));
    }    // 每次转一个随机角度：误差像随机游走一样累积
    for (long cycles : {1000L, 1000000L}) {
        Pcg32 r{6};
        Vec2 p{100, 0};
        for (long i = 0; i < cycles; ++i) {
            float a = r.nextFloat() * 6.28f;
            p = rotate(p, a), p = rotate(p, -a);
        }
        std::printf("random angle, %-8ld inverse error %.3g px\n", cycles, double(length(p - Vec2{100, 0})));
    }
    // 不可逆的操作：向右移动 30，但被 x = 100 的墙挡住；"反向操作"向左移动 30
    float x = 90, moved = std::min(x + 30, 100.0f), back = moved - 30;
    std::printf("clamped move: start %.0f, after +30 against a wall at 100: %.0f, inverse -30: %.0f\n", x, moved, back);
}

int main(int argc, char** argv) {
    auto want = [&](const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; };
    if (want("storage")) storage();
    if (want("chaos")) chaos();
    if (want("seek")) seek();
    if (want("undo")) undo();
    if (want("drift")) drift();
}
