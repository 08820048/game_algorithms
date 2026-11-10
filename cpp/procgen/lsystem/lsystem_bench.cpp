// L-System：长度增长、整串与流式展开的开销、航向的累积误差、盒维数、随机植物的差异（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/lsystem/lsystem_bench.cpp -o lsystem_bench && ./lsystem_bench [section]
//   section：growth memory drift dimension variation（缺省全部）
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "lsystem.h"
#include "presets.h"

using namespace ga;
using namespace ga::lsys;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
static double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
static const Preset& preset(const char* name) {
    static std::vector<Preset> ps = presets();
    for (const Preset& p : ps)
        if (std::string(p.name) == name) return p;
    std::abort();
}
template <class F>
static double best(int rounds, F&& f) {
    double b = 1e30;
    for (int r = 0; r < rounds; ++r) {
        auto t0 = Clock::now();
        f();
        b = std::min(b, msSince(t0));
    }
    return b;
}

// 长度和每代的增长倍数；最大特征值用幂迭代从长度矩阵算
static void growth() {
    std::printf("== growth: length per generation, ratio -> dominant eigenvalue\n");
    for (const Preset& p : presets()) {
        std::printf("%-11s", p.name);
        double prev = 0;
        for (int n = 0; n <= 12; ++n) {
            double len = predictedLength(p.sys, n);
            if (n % 2 == 0 || n == 12) std::printf("  n%-2d %-10.4g", n, len);
            if (n == 12) std::printf(" ratio %.4f", len / prev);
            prev = len;
        }
        auto cnt = predictedCounts(p.sys, 12);
        double draw = cnt['F'] + cnt['G'];
        std::printf("  draw share %.2f\n", draw / predictedLength(p.sys, 12));
    }
    LSystem t = treeSystem();
    std::printf("%-11s", "tree");
    for (int n = 0; n <= 12; n += 2) std::printf("  n%-2d %-10.4g", n, predictedLength(t, n));
    std::printf(" ratio %.4f\n", predictedLength(t, 12) / predictedLength(t, 11));
}

// 整串展开 + 海龟 vs 流式展开 + 海龟。内存：整串 = 两代字符串的容量；流式 = 栈 + 每代一个计数器。线段另算（存下来要 28 字节一段）
static void memory() {
    std::printf("== memory: materialize vs stream (both feed a turtle that only counts segments); best of 3\n");
    for (const char* name : {"plant", "weed", "dragon"}) {
        const Preset& p = preset(name);
        int n0 = std::string(name) == "dragon" ? 16 : 6, n1 = std::string(name) == "dragon" ? 24 : 11;
        for (int n = n0; n <= n1; n += (std::string(name) == "dragon" ? 2 : 1)) {
            size_t peakBytes = 0, len = 0;
            uint64_t segA = 0, segB = 0;
            double tA = best(3, [&] {
                std::string cur = p.sys.axiom, next;
                for (int level = 0; level < n; ++level) {
                    next.clear();
                    next.reserve(size_t(predictedLength(p.sys, level + 1)));
                    for (size_t i = 0; i < cur.size(); ++i) {
                        const std::string* r = choose(p.sys, cur[i], 0, level, i);
                        if (r) next += *r;
                        else next += cur[i];
                    }
                    peakBytes = std::max(peakBytes, cur.capacity() + next.capacity());
                    cur.swap(next);
                }
                len = cur.size();
                Turtle<double> t(p.turtle);
                for (char c : cur) t.feed(c);
                segA = t.drawn();
            });
            StreamStats st;
            int turtleStack = 0;
            double tB = best(3, [&] {
                Turtle<double> t(p.turtle);
                stream(p.sys, n, 0, [&](char c) { t.feed(c); }, &st);
                segB = t.drawn();
                turtleStack = t.maxStack();
            });
            size_t streamBytes = size_t(st.maxFrames) * 24 + size_t(n + 1) * 8;
            std::printf("%-7s n%-2d len %-11zu seg %-10llu  materialize %8.1f ms %9.1f MB | stream %8.1f ms %5zu B (frames %d, turtle stack %d)  seg-geom %.0f MB%s\n",
                        name, n, len, (unsigned long long)segA, tA, peakBytes / 1e6, tB, streamBytes, st.maxFrames, turtleStack, segA * double(sizeof(Segment)) / 1e6,
                        segA == segB ? "" : "  MISMATCH");
        }
    }
}

// 航向的四种写法和精确格点比较：最大偏差（沿整条曲线）和终点偏差，单位 = 一步的长度
template <class Real>
static void driftOne(const Preset& p, int n, Heading mode, double* maxErr, double* endErr, double* hlen) {
    TurtleOptions o = p.turtle;
    o.mode = mode;
    Turtle<Real> t(o);
    LatticeWalk w(p.lattice);
    double m = 0;
    stream(p.sys, n, 0, [&](char c) {
        t.feed(c), w.feed(c);
        if (c == 'F' || c == 'G') m = std::max(m, std::hypot(double(t.x()) - w.x(), double(t.y()) - w.y()));
    });
    *maxErr = m;
    *endErr = std::hypot(double(t.x()) - w.x(), double(t.y()) - w.y());
    // 方向向量的长度：再走一步看位移
    double x0 = double(t.x()), y0 = double(t.y());
    t.feed('f');
    *hlen = std::hypot(double(t.x()) - x0, double(t.y()) - y0);
}
static void drift() {
    std::printf("== drift: deviation from exact lattice walk, in step lengths (max over path / at end); |dir| after the walk\n");
    struct C {
        const char* name;
        int n0, n1;
    };
    for (C c : {C{"dragon", 12, 24}, C{"koch", 4, 10}, C{"sierpinski", 8, 15}, C{"hilbert", 6, 11}}) {
        const Preset& p = preset(c.name);
        for (int n = c.n0; n <= c.n1; n += 2) {
            double steps = 0;
            for (int k = 0; k < 128; ++k)
                if (k == 'F' || k == 'G') steps += predictedCounts(p.sys, n)[size_t(k)];
            std::printf("%-10s n%-2d steps %-9.3g", c.name, n, steps);
            const char* names[4] = {"double angle", "double rotate", "float angle", "float rotate"};
            for (int v = 0; v < 4; ++v) {
                double me, ee, hl;
                Heading h = v % 2 == 0 ? Heading::Angle : Heading::Rotate;
                if (v < 2) driftOne<double>(p, n, h, &me, &ee, &hl);
                else driftOne<float>(p, n, h, &me, &ee, &hl);
                std::printf(" | %s %.2g/%.2g", names[v], me, ee);
                if (v % 2 == 1) std::printf(" |h| %.6f", hl);
            }
            std::printf("\n");
        }
    }
}

// 盒维数：把图形放进 2^k × 2^k 的网格，数被线段碰到的格子 N(k)，log N 对 k·log 2 的斜率
static double boxDimension(const std::vector<Segment>& segs, int k0, int k1, std::vector<double>* perK = nullptr) {
    float minX = 1e30f, minY = 1e30f, maxX = -1e30f, maxY = -1e30f;
    for (const Segment& s : segs) {
        minX = std::min({minX, s.x0, s.x1}), maxX = std::max({maxX, s.x0, s.x1});
        minY = std::min({minY, s.y0, s.y1}), maxY = std::max({maxY, s.y0, s.y1});
    }
    double L = std::max(maxX - minX, maxY - minY) * 1.0001;
    std::vector<double> xs, ys;
    for (int k = k0; k <= k1; ++k) {
        int g = 1 << k;
        double box = L / g;
        std::vector<uint8_t> hit(size_t(g) * g, 0);
        long long count = 0;
        for (const Segment& s : segs) {
            double len = std::hypot(s.x1 - s.x0, s.y1 - s.y0);
            int m = std::max(1, int(std::ceil(len / (box * 0.25))));
            for (int i = 0; i <= m; ++i) {
                double t = double(i) / m;
                int cx = std::min(g - 1, int((s.x0 + (s.x1 - s.x0) * t - minX) / box));
                int cy = std::min(g - 1, int((s.y0 + (s.y1 - s.y0) * t - minY) / box));
                uint8_t& h = hit[size_t(cy) * g + cx];
                if (!h) h = 1, ++count;
            }
        }
        xs.push_back(k * std::log(2.0)), ys.push_back(std::log(double(count)));
        if (perK) perK->push_back(double(count));
    }
    double mx = 0, my = 0;
    for (size_t i = 0; i < xs.size(); ++i) mx += xs[i], my += ys[i];
    mx /= xs.size(), my /= xs.size();
    double sxy = 0, sxx = 0;
    for (size_t i = 0; i < xs.size(); ++i) sxy += (xs[i] - mx) * (ys[i] - my), sxx += (xs[i] - mx) * (xs[i] - mx);
    return sxy / sxx;
}
static void dimension() {
    std::printf("== dimension: box counting, slope of log N(k) vs k log 2\n");
    struct C {
        const char* name;
        int n;
        double theory;
    };
    for (C c : {C{"koch", 7, std::log(4.0) / std::log(3.0)}, C{"sierpinski", 11, std::log(3.0) / std::log(2.0)}, C{"hilbert", 9, 2}, C{"dragon", 18, 2}, C{"plant", 8, 0}, C{"weed", 7, 0}}) {
        const Preset& p = preset(c.name);
        std::vector<Segment> segs;
        Turtle<double> t(p.turtle, &segs);
        stream(p.sys, c.n, 0, [&](char ch) { t.feed(ch); });
        std::printf("%-10s n%-2d segs %-8zu", c.name, c.n, segs.size());
        for (auto [k0, k1] : {std::pair{2, 5}, std::pair{4, 7}, std::pair{6, 9}, std::pair{8, 11}}) std::printf("  k%d-%d %.3f", k0, k1, boxDimension(segs, k0, k1));
        if (c.theory) std::printf("  theory %.3f", c.theory);
        std::printf("\n");
    }
}

// 随机植物：同一套规则、不同种子之间差多少；以及"按顺序取随机数"在流式展开下为什么会变成另一棵树
static void variation() {
    std::printf("== variation: tree system, 2000 seeds\n");
    LSystem s = treeSystem();
    TurtleOptions o;
    o.angle = 24, o.heading = 90, o.lengthScale = 0.75, o.widthScale = 0.7;
    for (int n = 3; n <= 8; ++n) {
        std::vector<double> segs, h, w;
        for (int seed = 1; seed <= 2000; ++seed) {
            Turtle<double> t(o);
            stream(s, n, uint64_t(seed), [&](char c) { t.feed(c); });
            segs.push_back(double(t.drawn())), h.push_back(t.maxY() - t.minY()), w.push_back(t.maxX() - t.minX());
        }
        auto stat = [](std::vector<double>& v, const char* nm) {
            std::sort(v.begin(), v.end());
            double m = 0, m2 = 0;
            for (double x : v) m += x, m2 += x * x;
            m /= v.size();
            double sd = std::sqrt(m2 / v.size() - m * m);
            std::printf("  %s mean %-8.4g cv %.2f p5 %-7.4g p95 %-7.4g", nm, m, sd / m, v[v.size() / 20], v[v.size() * 19 / 20]);
        };
        std::printf("n%d", n);
        stat(segs, "segs");
        stat(h, "height");
        stat(w, "width");
        std::printf("  predicted segs %.4g\n", predictedCounts(s, n)['F']);
    }
    // 按顺序取随机数：整串展开按"一代一代、从左到右"消耗随机数，流式展开按深度优先的顺序消耗，同一个种子给出不同的树
    int differ = 0, differHash = 0;
    for (int seed = 1; seed <= 200; ++seed) {
        auto pick = [&](Pcg32& rng, char c) -> const std::string* {
            const auto& rs = s.rules[uint8_t(c)];
            if (rs.empty()) return nullptr;
            double total = 0;
            for (const Production& p : rs) total += p.weight;
            double u = rng.nextDouble() * total;
            for (const Production& p : rs) {
                if (u < p.weight) return &p.rhs;
                u -= p.weight;
            }
            return &rs.back().rhs;
        };
        const int n = 6;
        Pcg32 r1{uint64_t(seed)};
        std::string cur = s.axiom, next;
        for (int level = 0; level < n; ++level) {
            next.clear();
            for (char c : cur) {
                const std::string* r = pick(r1, c);
                next += r ? *r : std::string(1, c);
            }
            cur.swap(next);
        }
        Pcg32 r2{uint64_t(seed)};
        std::string dfs;
        std::function<void(char, int)> rec = [&](char c, int level) {
            if (level == n) {
                dfs += c;
                return;
            }
            const std::string* r = pick(r2, c);
            if (!r) return rec(c, level + 1);
            for (char d : *r) rec(d, level + 1);
        };
        for (char c : s.axiom) rec(c, 0);
        differ += cur != dfs;
        std::string a = expand(s, n, uint64_t(seed)), b;
        stream(s, n, uint64_t(seed), [&](char c) { b += c; });
        differHash += a != b;
    }
    std::printf("same seed, materialize vs depth-first: sequential rng differ %d/200, hashed (level,index) differ %d/200\n", differ, differHash);
}

int main(int argc, char** argv) {
    if (want(argc, argv, "growth")) growth();
    if (want(argc, argv, "memory")) memory();
    if (want(argc, argv, "drift")) drift();
    if (want(argc, argv, "dimension")) dimension();
    if (want(argc, argv, "variation")) variation();
}
