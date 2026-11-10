#include "wfc.h"

#include <cstdio>
#include <set>
#include <vector>

#include "check.h"
#include "samples.h"

using namespace ga;
using namespace ga::wfc;

// ---- 参照写法 ----
static Rules randomRules(Pcg32& rng, int T, double density) {
    Rules r;
    r.init(T);
    for (int t = 0; t < T; ++t) r.weight[t] = 0.2 + rng.nextDouble() * 3;
    for (int a = 0; a < T; ++a)
        for (int d = 0; d < 2; ++d)
            for (int b = 0; b < T; ++b)
                if (rng.nextDouble() < density) r.allow(a, d, b);
    return r;
}

static bool has(const std::vector<uint64_t>& dom, int words, int c, int t) { return (dom[size_t(c) * words + (t >> 6)] >> (t & 63)) & 1; }

// 弧相容的不动点：反复扫描所有格子，删掉在某个方向上找不到相容邻居的图块，直到不再变化
static std::vector<uint64_t> naiveFixpoint(const Rules& r, int w, int h, bool periodic, std::vector<uint64_t> dom) {
    for (bool changed = true; changed;) {
        changed = false;
        for (int c = 0; c < w * h; ++c)
            for (int t = 0; t < r.T; ++t) {
                if (!has(dom, r.words, c, t)) continue;
                for (int d = 0; d < 4; ++d) {
                    int x = c % w + kDx[d], y = c / w + kDy[d];
                    if (periodic) x = (x + w) % w, y = (y + h) % h;
                    else if (x < 0 || y < 0 || x >= w || y >= h) continue;
                    int n = y * w + x;
                    bool ok = false;
                    for (int b = 0; b < r.T && !ok; ++b) ok = has(dom, r.words, n, b) && r.allows(t, d, b);
                    if (!ok) {
                        dom[size_t(c) * r.words + (t >> 6)] &= ~(uint64_t(1) << (t & 63));
                        changed = true;
                        break;
                    }
                }
            }
    }
    return dom;
}

// 穷举所有合法拼法（格子按行优先，逐个试）
static void enumerate(const Rules& r, int w, int h, bool periodic, std::vector<int>& cur, int c, std::vector<std::vector<int>>& out, size_t cap) {
    if (out.size() >= cap) return;
    if (c == w * h) {
        if (validTiling(r, w, h, periodic, cur)) out.push_back(cur);
        return;
    }
    int x = c % w, y = c / w;
    for (int t = 0; t < r.T; ++t) {
        if (x > 0 && !r.allows(cur[c - 1], 0, t)) continue;
        if (y > 0 && !r.allows(cur[c - w], 1, t)) continue;
        cur[c] = t;
        enumerate(r, w, h, periodic, cur, c + 1, out, cap);
    }
}
static std::vector<std::vector<int>> allSolutions(const Rules& r, int w, int h, bool periodic, size_t cap = 1 << 20) {
    std::vector<std::vector<int>> out;
    std::vector<int> cur(size_t(w) * h, -1);
    enumerate(r, w, h, periodic, cur, 0, out, cap);
    return out;
}

static std::vector<uint64_t> domains(const Solver& s, int words) {
    std::vector<uint64_t> d;
    for (int c = 0; c < s.width() * s.height(); ++c) d.insert(d.end(), s.domain(c), s.domain(c) + words);
    return d;
}

static void testPropagation() {
    // 随机规则、随机外部约束：队列传播的结果和反复扫描的不动点完全一致（包括多字位集）
    int compared = 0, contradictions = 0;
    for (int s = 0; s < 300; ++s) {
        Pcg32 rng{uint64_t(10 + s)};
        int T = 2 + int(rng.below(s % 5 == 0 ? 120 : 10)), w = 2 + int(rng.below(6)), h = 2 + int(rng.below(6));
        bool periodic = s % 3 == 0;
        Rules r = randomRules(rng, T, 0.25 + 0.5 * rng.nextDouble());
        Solver sol(r, w, h, {Heuristic::Entropy, periodic}, uint64_t(s));
        std::vector<uint64_t> constraint(size_t(w) * h * r.words, ~uint64_t(0));
        for (int k = 0; k < 6 && sol.status() == Status::Running; ++k) {
            int c = int(rng.below(uint32_t(w * h)));
            std::vector<uint64_t> mask(r.words, 0);
            for (int t = 0; t < T; ++t)
                if (rng.nextDouble() < 0.5) mask[t >> 6] |= uint64_t(1) << (t & 63);
            for (int j = 0; j < r.words; ++j) constraint[size_t(c) * r.words + j] &= mask[j];
            sol.restrict(c, mask);
            std::vector<uint64_t> start(size_t(w) * h * r.words, 0);
            for (int cc = 0; cc < w * h; ++cc)
                for (int t = 0; t < T; ++t) start[size_t(cc) * r.words + (t >> 6)] |= uint64_t(1) << (t & 63);
            for (size_t i = 0; i < start.size(); ++i) start[i] &= constraint[i];
            std::vector<uint64_t> ref = naiveFixpoint(r, w, h, periodic, start);
            bool refEmpty = false;
            for (int cc = 0; cc < w * h; ++cc) {
                bool e = true;
                for (int j = 0; j < r.words; ++j) e &= ref[size_t(cc) * r.words + j] == 0;
                refEmpty |= e;
            }
            if (sol.status() == Status::Contradiction) {
                CHECK(refEmpty);
                ++contradictions;
                break;
            }
            CHECK(!refEmpty);
            CHECK(domains(sol, r.words) == ref);
            ++compared;
        }
    }
    CHECK(compared > 500);
    CHECK(contradictions > 20);
}

static void checkBookkeeping(const Rules& r, const Solver& s) {
    for (int c = 0; c < s.width() * s.height(); ++c) {
        int n = 0;
        double sw = 0, swl = 0;
        for (int t = 0; t < r.T; ++t)
            if ((s.domain(c)[t >> 6] >> (t & 63)) & 1) ++n, sw += r.weight[t], swl += r.weight[t] * std::log(r.weight[t]);
        CHECK(n == s.count(c));
        if (n > 1) CHECK(std::abs(s.entropy(c) - (std::log(sw) - swl / sw)) < 1e-9);
    }
}

static void testEntropyBookkeeping() {
    // 增量维护的候选数和熵，与按当前集合重新计算的一致
    Pcg32 rng{5};
    Rules r = fromSockets(terrainTiles());
    Solver s(r, 12, 9, {}, 7);
    for (int k = 0; k < 30 && s.status() == Status::Running; ++k) s.step();
    checkBookkeeping(r, s);
    // 回溯撤销之后也一致
    Rules roads = fromSockets(roadTiles(true));
    int checked = 0;
    for (int seed = 1; seed < 200 && checked < 20; ++seed) {
        Solver b(roads, 24, 24, {Heuristic::Random, false, true}, uint64_t(seed));
        while (b.status() == Status::Running && b.stats().backtracks == 0) b.step();
        if (b.status() != Status::Running) continue;
        checkBookkeeping(roads, b);
        ++checked;
    }
    CHECK(checked == 20);
    // 一开始所有格子的熵相同：第一个被观察的格子应该随机分布，而不是总在同一处
    std::set<int> first;
    for (int seed = 1; seed <= 200; ++seed) {
        Solver f(r, 12, 9, {}, uint64_t(seed));
        f.step();
        first.insert(f.lastObserved());
    }
    CHECK(first.size() > 60);
}

static void testBacktrackingVsBrute() {
    // 小网格：开启回溯（不限次数）时，有解就一定找到、无解就报告矛盾；找到的解在穷举的解集里
    int sat = 0, unsat = 0;
    for (int s = 0; s < 400; ++s) {
        Pcg32 rng{uint64_t(200 + s)};
        int T = 2 + int(rng.below(5)), w = 2 + int(rng.below(3)), h = 2 + int(rng.below(3));
        bool periodic = s % 2;
        Rules r = randomRules(rng, T, 0.3 + 0.4 * rng.nextDouble());
        const size_t cap = 200000;
        auto sols = allSolutions(r, w, h, periodic, cap);
        std::set<std::vector<int>> ss(sols.begin(), sols.end());
        for (Heuristic hh : {Heuristic::Entropy, Heuristic::Scanline, Heuristic::Random}) {
            Options o{hh, periodic, true};
            Solver sol(r, w, h, o, uint64_t(s));
            Status st = sol.run();
            CHECK(sol.stats().backtracks < 1000);  // 回溯是穷尽的：每次撤销都禁掉一个选择，小网格上几步就证明无解
            if (sols.empty()) {
                CHECK(st == Status::Contradiction);
                ++unsat;
            } else {
                CHECK(st == Status::Done);
                CHECK(validTiling(r, w, h, periodic, sol.result()));
                if (sols.size() < cap) CHECK(ss.count(sol.result()) == 1);  // 解集穷举完整时才能比较
                ++sat;
            }
        }
    }
    CHECK(sat > 300 && unsat > 100);
    // 更大的网格、扫描线 + 回溯：需要连续回退好几层，结果仍然合法、完整
    int deep = 0;
    for (int s = 0; s < 300; ++s) {
        Pcg32 rng{uint64_t(900 + s)};
        Rules r = randomRules(rng, 8, 0.35);
        Solver sol(r, 6, 6, {Heuristic::Scanline, false, true, 20000}, uint64_t(s));
        if (sol.run() == Status::Done) {
            CHECK(validTiling(r, 6, 6, false, sol.result()));
            deep += sol.stats().backtracks >= 3;
        }
    }
    CHECK(deep > 10);
    // 经典的无解例子：两种颜色，相邻必须不同；周期边界、宽度为奇数时无解
    Rules chk;
    chk.init(2);
    chk.allow(0, 0, 1), chk.allow(1, 0, 0), chk.allow(0, 1, 1), chk.allow(1, 1, 0);
    for (int w : {3, 4, 5, 6}) {
        Solver sol(chk, w, 4, {Heuristic::Entropy, true, true}, 1);
        CHECK((sol.run() == Status::Done) == (w % 2 == 0));
        CHECK(allSolutions(chk, w, 4, true).empty() == (w % 2 == 1));
    }
}

static void testSupport() {
    // 所有权重为正时，每个合法拼法都可能出现，且只出现合法拼法（权重统一为 1，免得某些拼法概率太小抽不到）
    Rules r = fromSockets(roadTiles(true));
    Rules flat = r;
    std::fill(flat.weight.begin(), flat.weight.end(), 1.0);
    auto sols = allSolutions(flat, 2, 2, false);
    std::set<std::vector<int>> ss(sols.begin(), sols.end()), seen;
    for (int k = 0; k < 300000; ++k) {
        Solver s(flat, 2, 2, {}, uint64_t(k + 1));
        CHECK(s.run() == Status::Done);
        CHECK(ss.count(s.result()) == 1);
        seen.insert(s.result());
    }
    CHECK(seen == ss);
    // 无断头路的道路：随机挑格子时会矛盾（按熵挑不会），成功的结果都合法
    int contra = 0, ok = 0;
    for (int k = 0; k < 3000; ++k) {
        Solver s(r, 8, 8, {Heuristic::Random}, uint64_t(k + 1));
        Status st = s.run();
        if (st == Status::Done) {
            CHECK(validTiling(r, 8, 8, false, s.result()));
            ++ok;
        } else {
            CHECK(s.contradictionCell() >= 0);
            ++contra;
        }
    }
    CHECK(contra > 10 && ok > 1000);
}

static void testWeights() {
    // 1×1 的网格：观察的结果按权重分布
    Rules r = fromSockets(roadTiles(false));
    std::vector<int> hits(r.T, 0);
    const int n = 400000;
    for (int k = 0; k < n; ++k) {
        Solver s(r, 1, 1, {}, uint64_t(k + 1));
        s.run();
        ++hits[s.tileAt(0)];
    }
    double tot = 0;
    for (double w : r.weight) tot += w;
    for (int t = 0; t < r.T; ++t) {
        double p = r.weight[t] / tot, sd = std::sqrt(p * (1 - p) / n);
        CHECK(std::abs(double(hits[t]) / n - p) < 5 * sd);
    }
}

static void testTileSets() {
    // 道路：16 个图块，断头路版本 12 个；接口对称
    CHECK(roadTiles(false).size() == 16 && roadTiles(true).size() == 12);
    for (bool nd : {false, true}) {
        Rules r = fromSockets(roadTiles(nd));
        for (int a = 0; a < r.T; ++a)
            for (int d = 0; d < 4; ++d)
                for (int b = 0; b < r.T; ++b) CHECK(r.allows(a, d, b) == r.allows(b, opposite(d), a));
    }
    // 地形：81 种角的组合里，没有水草相邻边的；所有结果合法，且从不矛盾
    auto corners = terrainCorners();
    int brute = 0;
    for (int m = 0; m < 81; ++m) {
        int c[4] = {m % 3, m / 3 % 3, m / 9 % 3, m / 27};
        bool ok = true;
        for (int k = 0; k < 4; ++k) ok &= std::abs(c[k] - c[(k + 1) % 4]) < 2;
        brute += ok;
    }
    CHECK(int(corners.size()) == brute);
    Rules t = fromSockets(terrainTiles());
    for (int k = 0; k < 100; ++k) {
        Solver s(t, 16, 12, {Heuristic(k % 4), k % 2 == 0}, uint64_t(k + 1));
        CHECK(s.run() == Status::Done);
        CHECK(validTiling(t, 16, 12, k % 2 == 0, s.result()));
        // 相邻图块共享的角颜色一致
        std::vector<int> res = s.result();
        for (int y = 0; y < 12; ++y)
            for (int x = 0; x + 1 < 16; ++x) {
                const CornerTile &a = corners[res[y * 16 + x]], &b = corners[res[y * 16 + x + 1]];
                CHECK(a.tr == b.tl && a.br == b.bl);
            }
    }
}

// 重叠模型的输出画成图：非周期时图像比图案网格大 N − 1
static std::vector<uint8_t> render(const std::vector<std::vector<uint8_t>>& pats, int N, int w, int h, const std::vector<int>& res, int* iw, int* ih) {
    *iw = w + N - 1, *ih = h + N - 1;
    std::vector<uint8_t> img(size_t(*iw) * *ih);
    for (int y = 0; y < *ih; ++y)
        for (int x = 0; x < *iw; ++x) {
            int cx = std::min(x, w - 1), cy = std::min(y, h - 1);
            img[size_t(y) * *iw + x] = pats[res[cy * w + cx]][size_t(y - cy) * N + (x - cx)];
        }
    return img;
}

static void testOverlap() {
    // 图案数：全是同一种颜色 → 1 个；2×2 棋盘（周期）N = 2 → 2 个；权重之和 = 位置数 × 变换数
    {
        std::string al;
        Sample s = sampleFromText({"aaaa", "aaaa", "aaaa"}, &al);
        Rules r = fromSample(s, 3, true, false);
        CHECK(r.T == 1 && r.weight[0] == 2 * 8);
        Sample c = sampleFromText({"ab", "ba"}, &al);
        Rules rc = fromSample(c, 2, false, true);
        CHECK(rc.T == 2 && rc.weight[0] + rc.weight[1] == 4);
        CHECK(!rc.allows(0, 0, 0) && rc.allows(0, 0, 1));
    }
    // 输出图的每个 N×N 窗口都是样图里出现过的图案（这是重叠模型的全部承诺）
    int checked = 0;
    for (int which = 0; which < 3; ++which)
        for (int N : {2, 3}) {
            std::string al;
            Sample s = sampleFromText(which == 0 ? roomsText() : which == 1 ? lakesText() : mazeText(), &al);
            std::vector<std::vector<uint8_t>> pats;
            Rules r = fromSample(s, N, which != 0, which != 0, &pats);
            std::set<std::vector<uint8_t>> ps(pats.begin(), pats.end());
            for (int k = 0; k < 4; ++k) {
                Solver sol(r, 20, 14, {Heuristic::Entropy, false, true, 5000}, uint64_t(k + 1));
                if (sol.run() != Status::Done) continue;
                int iw, ih;
                std::vector<uint8_t> img = render(pats, N, 20, 14, sol.result(), &iw, &ih);
                for (int y = 0; y + N <= ih; ++y)
                    for (int x = 0; x + N <= iw; ++x) {
                        std::vector<uint8_t> win(size_t(N) * N);
                        for (int j = 0; j < N; ++j)
                            for (int i = 0; i < N; ++i) win[size_t(j) * N + i] = img[size_t(y + j) * iw + x + i];
                        CHECK(ps.count(win) == 1);
                    }
                ++checked;
            }
        }
    CHECK(checked >= 18);
    // 对称：旋转、镜像后的图案也在集合里
    std::string al;
    Sample s = sampleFromText(roomsText(), &al);
    std::vector<std::vector<uint8_t>> p1, p8;
    fromSample(s, 3, false, false, &p1);
    fromSample(s, 3, true, false, &p8);
    std::set<std::vector<uint8_t>> s8(p8.begin(), p8.end());
    for (const auto& p : p1) {
        // 8 种变换：4 个旋转 × 是否镜像
        std::vector<uint8_t> q = p;
        for (int m = 0; m < 2; ++m) {
            for (int k = 0; k < 4; ++k) {
                CHECK(s8.count(q) == 1);
                std::vector<uint8_t> rot(9);
                for (int j = 0; j < 3; ++j)
                    for (int i = 0; i < 3; ++i) rot[j * 3 + i] = q[(2 - i) * 3 + j];
                q = rot;
            }
            std::vector<uint8_t> mir(9);
            for (int j = 0; j < 3; ++j)
                for (int i = 0; i < 3; ++i) mir[j * 3 + i] = q[j * 3 + 2 - i];
            q = mir;
        }
    }
    CHECK(p8.size() > p1.size());
}

static void testMaze() {
    // 迷宫样图：12×12，通道 = 36 个格子 + 深度优先生成树的边
    auto m = mazeText();
    CHECK(m.size() == 12 && m[0].size() == 12);
    int open = 0;
    for (auto& r : m)
        for (char c : r) open += c == '.';
    CHECK(open == 36 + 35);  // 36 个格子 + 生成树的 35 条边
}

int main() {
    testPropagation();
    testEntropyBookkeeping();
    testBacktrackingVsBrute();
    testSupport();
    testWeights();
    testTileSets();
    testOverlap();
    testMaze();
    return report("wfc_test");
}
