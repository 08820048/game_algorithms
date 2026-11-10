#include "lsystem.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "check.h"
#include "presets.h"

using namespace ga;
using namespace ga::lsys;

// ---- 参照写法 ----
// 递归改写：把字符 c 从第 level 代改写到第 n 代。位置计数器按代数分开（深度优先恰好从左到右）
static void rewriteRef(const LSystem& s, char c, int level, int n, uint64_t seed, std::vector<uint64_t>& idx, std::string& out) {
    if (level == n) {
        out += c;
        return;
    }
    const std::string* r = choose(s, c, seed, level, idx[size_t(level)]++);
    if (!r) {
        rewriteRef(s, c, level + 1, n, seed, idx, out);
        return;
    }
    for (char d : *r) rewriteRef(s, d, level + 1, n, seed, idx, out);
}
static std::string expandRef(const LSystem& s, int n, uint64_t seed) {
    std::vector<uint64_t> idx(size_t(n) + 1, 0);
    std::string out;
    for (char c : s.axiom) rewriteRef(s, c, 0, n, seed, idx, out);
    return out;
}
static std::string streamed(const LSystem& s, int n, uint64_t seed, StreamStats* st = nullptr) {
    std::string out;
    stream(s, n, seed, [&](char c) { out += c; }, st);
    return out;
}

static LSystem randomSystem(Pcg32& rng) {
    const char alphabet[] = "ABXF+-[]";
    LSystem s;
    int al = int(rng.below(6)) + 1;
    for (int i = 0; i < al; ++i) s.axiom += alphabet[rng.below(8)];
    for (int k = 0; k < 4; ++k) {
        if (rng.chance(0.25)) continue;  // 有的字符没有规则
        int np = int(rng.below(3)) + 1;
        for (int p = 0; p < np; ++p) {
            std::string rhs;
            int len = int(rng.below(5));  // 可以为空：这个字符在下一代消失
            for (int i = 0; i < len; ++i) rhs += alphabet[rng.below(8)];
            s.rule("ABXF"[k], rhs, 0.5 + rng.nextDouble());
        }
    }
    return s;
}

// ---- 展开 ----
static void testExpandAgreesWithReference() {
    Pcg32 rng(11);
    int compared = 0, stochasticCases = 0;
    for (int trial = 0; trial < 3000; ++trial) {
        LSystem s = randomSystem(rng);
        int n = int(rng.below(7));
        if (predictedLength(s, n) > 2e5) continue;
        uint64_t seed = rng.next();
        std::string a = expand(s, n, seed);
        StreamStats st;
        std::string b = streamed(s, n, seed, &st);
        std::string c = expandRef(s, n, seed);
        CHECK(a == c);
        CHECK(b == c);
        CHECK(st.emitted == c.size());
        CHECK(st.maxFrames <= n + 1);
        ++compared;
        stochasticCases += s.stochastic();
    }
    CHECK(compared > 2000);
    CHECK(stochasticCases > 1000);
}

static void testSmallExamples() {
    LSystem algae;
    algae.axiom = "A";
    algae.rule('A', "AB").rule('B', "A");
    CHECK(expand(algae, 0) == "A");
    CHECK(expand(algae, 1) == "AB");
    CHECK(expand(algae, 4) == "ABAABABA");
    // 长度是斐波那契数
    long long f0 = 1, f1 = 2;
    for (int n = 1; n <= 25; ++n) {
        CHECK(expand(algae, n).size() == size_t(f1));
        CHECK(predictedLength(algae, n) == double(f1));
        long long t = f0 + f1;
        f0 = f1, f1 = t;
    }
    LSystem vanish;
    vanish.axiom = "AXA";
    vanish.rule('A', "");
    CHECK(expand(vanish, 1) == "X");
    // 每代都有规则时，栈恰好深 n + 1 帧
    for (int n = 1; n <= 10; ++n) {
        StreamStats st;
        streamed(algae, n, 0, &st);
        CHECK(st.maxFrames == n + 1);
    }
    CHECK(streamed(vanish, 3, 0) == "X");
}

// 长度矩阵：确定性系统精确相等；随机系统是期望
static void testPredictedLength() {
    for (const Preset& p : presets()) {
        if (p.sys.stochastic()) continue;
        for (int n = 0; n <= p.maxGen; ++n) {
            std::string s = expand(p.sys, n);
            auto cnt = predictedCounts(p.sys, n);
            std::map<char, long long> actual;
            for (char c : s) ++actual[c];
            for (int c = 0; c < 128; ++c) CHECK(cnt[size_t(c)] == double(actual[char(c)]));
        }
    }
    for (const LSystem& s : {presets()[6].sys, treeSystem()}) {
        const int n = 5, trials = 4000;
        double sum = 0, sum2 = 0;
        for (int t = 0; t < trials; ++t) {
            double len = double(expand(s, n, uint64_t(t) * 7 + 1).size());
            sum += len, sum2 += len * len;
        }
        double mean = sum / trials, sd = std::sqrt(sum2 / trials - mean * mean);
        CHECK(std::fabs(mean - predictedLength(s, n)) < 4 * sd / std::sqrt(double(trials)));
    }
}

static void testChoiceFrequencies() {
    LSystem s;
    s.rule('A', "x", 1).rule('A', "y", 2).rule('A', "z", 5);
    int count[3] = {};
    const int n = 80000;
    for (int i = 0; i < n; ++i) {
        const std::string* r = choose(s, 'A', 42, 3, uint64_t(i));
        ++count[(*r)[0] - 'x'];
    }
    double expect[3] = {n / 8.0, n * 2 / 8.0, n * 5 / 8.0};
    for (int k = 0; k < 3; ++k) CHECK(std::fabs(count[k] - expect[k]) < 4 * std::sqrt(expect[k]));
    // 不同的代、不同的种子要给出不同的选择
    int sameLevel = 0, sameSeed = 0;
    for (int i = 0; i < 1000; ++i) {
        sameLevel += choose(s, 'A', 42, 3, uint64_t(i)) == choose(s, 'A', 42, 4, uint64_t(i));
        sameSeed += choose(s, 'A', 42, 3, uint64_t(i)) == choose(s, 'A', 43, 3, uint64_t(i));
    }
    double pSame = (1 + 4 + 25) / 64.0;
    CHECK(std::fabs(sameLevel / 1000.0 - pSame) < 0.06);
    CHECK(std::fabs(sameSeed / 1000.0 - pSame) < 0.06);
    CHECK(expand(treeSystem(), 6, 1) != expand(treeSystem(), 6, 2));
    CHECK(expand(treeSystem(), 6, 1) == expand(treeSystem(), 6, 1));
}

// ---- 曲线的几何性质（在精确格点上） ----
static const Preset& preset(const char* name) {
    static std::vector<Preset> ps = presets();
    for (const Preset& p : ps)
        if (std::string(p.name) == name) return p;
    std::abort();
}

static void testHilbertVisitsEveryCell() {
    const Preset& p = preset("hilbert");
    for (int n = 1; n <= 7; ++n) {
        LatticeWalk w(4);
        std::set<std::pair<long long, long long>> seen{{0, 0}};
        long long minA = 0, minB = 0, maxA = 0, maxB = 0;
        stream(p.sys, n, 0, [&](char c) {
            w.feed(c);
            if (c == 'F') {
                seen.insert({w.a, w.b});
                minA = std::min(minA, w.a), maxA = std::max(maxA, w.a);
                minB = std::min(minB, w.b), maxB = std::max(maxB, w.b);
            }
        });
        long long side = 1LL << n;
        CHECK(w.steps == side * side - 1);
        CHECK(seen.size() == size_t(side * side));  // 每个格子恰好一次
        CHECK(maxA - minA == side - 1 && maxB - minB == side - 1);
    }
}

static void testDragonNeverRetracesAnEdge() {
    const Preset& p = preset("dragon");
    for (int n = 1; n <= 14; ++n) {
        LatticeWalk w(4);
        std::set<std::array<long long, 4>> edges;
        bool repeated = false;
        stream(p.sys, n, 0, [&](char c) {
            long long a0 = w.a, b0 = w.b;
            w.feed(c);
            if (c == 'F') {
                std::array<long long, 4> e = a0 < w.a || (a0 == w.a && b0 < w.b) ? std::array<long long, 4>{a0, b0, w.a, w.b} : std::array<long long, 4>{w.a, w.b, a0, b0};
                repeated |= !edges.insert(e).second;
            }
        });
        CHECK(!repeated);
        CHECK(w.steps == (1LL << n));
        CHECK(w.a * w.a + w.b * w.b == (1LL << n));  // 终点离起点 √2^n
    }
}

static void testKochAndSierpinski() {
    for (int n = 0; n <= 7; ++n) {
        LatticeWalk w(6);
        stream(preset("koch").sys, n, 0, [&](char c) { w.feed(c); });
        CHECK(w.steps == 3 * (1LL << (2 * n)));
        CHECK(w.a == 0 && w.b == 0);  // 雪花是闭合的
    }
    for (int n = 0; n <= 10; ++n) {
        LatticeWalk w(6);
        stream(preset("sierpinski").sys, n, 0, [&](char c) { w.feed(c); });
        long long pow3 = 1;
        for (int i = 0; i < n; ++i) pow3 *= 3;
        CHECK(w.steps == pow3);
        CHECK(w.a * w.a + w.a * w.b + w.b * w.b == (1LL << (2 * n)));  // 终点离起点 2^n
    }
}

// ---- 海龟 ----
static void testTurtleBasics() {
    TurtleOptions o;
    o.angle = 90;
    std::vector<Segment> segs;
    std::vector<Leaf> leaves;
    Turtle<double> t(o, &segs, &leaves);
    t.feed("F[+F]F[-fF]!F");
    t.finish();
    CHECK(segs.size() == 5);
    auto near = [](const Segment& s, float x0, float y0, float x1, float y1) {
        return std::fabs(s.x0 - x0) < 1e-6f && std::fabs(s.y0 - y0) < 1e-6f && std::fabs(s.x1 - x1) < 1e-6f && std::fabs(s.y1 - y1) < 1e-6f;
    };
    CHECK(near(segs[0], 0, 0, 1, 0));
    CHECK(near(segs[1], 1, 0, 1, 1) && segs[1].depth == 1 && segs[1].dist == 1);
    CHECK(near(segs[2], 1, 0, 2, 0) && segs[2].depth == 0);
    CHECK(near(segs[3], 2, -1, 2, -2) && segs[3].dist == 3);  // f 移动但不画
    CHECK(near(segs[4], 2, 0, 3, 0) && std::fabs(segs[4].width - 0.7f) < 1e-6f);
    CHECK(leaves.size() == 3);  // 两个分叉的末端 + 主干末端
    CHECK(t.maxStack() == 1);
    CHECK(t.minY() == -2 && t.maxX() == 3);

    // 步长缩放和掉头
    TurtleOptions o2;
    o2.lengthScale = 0.5;
    Turtle<double> t2(o2);
    t2.feed("F\"F");
    CHECK(std::fabs(t2.x() - 1.5) < 1e-12);
    t2.feed("|F");
    CHECK(std::fabs(t2.x() - 1.0) < 1e-12 && t2.drawn() == 3);
    // 没画线就出栈不算叶子；包围盒包括起点
    std::vector<Leaf> lv;
    Turtle<double> t4(o, nullptr, &lv);
    t4.feed("[+f][-]F");
    CHECK(lv.empty());
    CHECK(t4.minX() == 0 && t4.maxX() == 1);
    // 不配对的 ] 被忽略
    Turtle<double> t3(o2);
    t3.feed("]]F");
    CHECK(std::fabs(t3.x() - 1) < 1e-12);
}

// 两种航向表示、以及 double 海龟和精确格点在小规模下一致
static void testTurtleMatchesLattice() {
    for (const char* name : {"dragon", "koch", "sierpinski", "hilbert"}) {
        const Preset& p = preset(name);
        int n = std::min(p.maxGen, 9);
        TurtleOptions o = p.turtle;
        Turtle<double> ta(o);
        o.mode = Heading::Rotate;
        Turtle<double> tr(o);
        LatticeWalk w(p.lattice);
        stream(p.sys, n, 0, [&](char c) {
            ta.feed(c), tr.feed(c), w.feed(c);
        });
        CHECK(std::fabs(ta.x() - w.x()) < 1e-6 && std::fabs(ta.y() - w.y()) < 1e-6);
        CHECK(std::fabs(tr.x() - w.x()) < 1e-6 && std::fabs(tr.y() - w.y()) < 1e-6);
    }
}

// 带掉头 | 的随机串：格点和海龟一致
static void testLatticeWithReversal() {
    Pcg32 rng(5);
    for (int k : {4, 6}) {
        for (int trial = 0; trial < 200; ++trial) {
            std::string str;
            for (int i = 0; i < 40; ++i) str += "FF+-|"[rng.below(5)];
            TurtleOptions o;
            o.angle = 360.0 / k;
            Turtle<double> t(o);
            LatticeWalk w(k);
            for (char c : str) t.feed(c), w.feed(c);
            CHECK(std::fabs(t.x() - w.x()) < 1e-9 && std::fabs(t.y() - w.y()) < 1e-9);
        }
    }
}

// 向性：水平的枝条往下弯，竖直向上的不弯
static void testTropism() {
    TurtleOptions o;
    o.tropism = 0.2;
    Turtle<double> t(o);
    t.feed("FFFF");
    CHECK(t.y() < -0.1);
    o.heading = 90;
    Turtle<double> up(o);
    up.feed("FFFF");
    CHECK(std::fabs(up.x()) < 1e-9 && std::fabs(up.y() - 4) < 1e-9);
    o.mode = Heading::Rotate, o.heading = 0;
    Turtle<double> tr(o);
    tr.feed("FFFF");
    CHECK(std::fabs(tr.y() - t.y()) < 1e-9);
}

// 抖动只影响转向，种子相同则结果相同
static void testJitter() {
    TurtleOptions o;
    o.angle = 30, o.jitter = 10, o.seed = 5;
    Turtle<double> a(o), b(o);
    a.feed("F+F-F+F"), b.feed("F+F-F+F");
    CHECK(a.x() == b.x() && a.y() == b.y());
    o.seed = 6;
    Turtle<double> c(o);
    c.feed("F+F-F+F");
    CHECK(a.x() != c.x());
    o.jitter = 0;
    Turtle<double> d(o);
    d.feed("F+F-F+F");
    CHECK(std::fabs(a.x() - d.x()) < 0.5);
}

int main() {
    testExpandAgreesWithReference();
    testSmallExamples();
    testPredictedLength();
    testChoiceFrequencies();
    testHilbertVisitsEveryCell();
    testDragonNeverRetracesAnEdge();
    testKochAndSierpinski();
    testTurtleBasics();
    testTurtleMatchesLattice();
    testLatticeWithReversal();
    testTropism();
    testJitter();
    return report("lsystem_test");
}
