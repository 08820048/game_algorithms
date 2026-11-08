// 蒙特卡洛树搜索：迭代次数与探索常数、速度、与 Alpha-Beta 对局、浅层陷阱（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/mcts/mcts_bench.cpp -o mcts_bench && ./mcts_bench [section]
//   section：ttt / speed / match / rollout / c / trap，省略时全部运行
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unordered_set>

#include "../minimax/games.h"
#include "../minimax/minimax.h"
#include "mcts.h"

using namespace ga;
using namespace ga::mcts;
using gametree::ConnectFour;
using gametree::TicTacToe;
using Clock = std::chrono::steady_clock;

static double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
static int sgn(int v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }

// 井字棋随机对局的精确结果概率
static void exactRandom(const TicTacToe& g, double p, double* out) {  // out：先手胜、后手胜、和
    int m[9];
    int n = g.listMoves(m, false);
    for (int i = 0; i < n; ++i) {
        TicTacToe c = g;
        c.play(m[i]);
        if (c.lastMoverWon()) out[c.moves % 2 ? 0 : 1] += p / n;
        else if (c.full()) out[2] += p / n;
        else exactRandom(c, p / n, out);
    }
}

static void sectionTtt() {
    double pr[3] = {0, 0, 0};
    exactRandom(TicTacToe{}, 1, pr);
    std::printf("\n== tic-tac-toe ==\nuniform random play: first wins %.4f, second wins %.4f, draw %.4f\n", pr[0], pr[1], pr[2]);
    std::vector<TicTacToe> all;
    std::unordered_set<uint64_t> seen;
    std::vector<TicTacToe> stack{TicTacToe{}};
    while (!stack.empty()) {
        TicTacToe g = stack.back();
        stack.pop_back();
        if (!seen.insert(g.key()).second || g.lastMoverWon() || g.full()) continue;
        all.push_back(g);
        int m[9];
        int n = g.listMoves(m, false);
        for (int i = 0; i < n; ++i) {
            TicTacToe c = g;
            c.play(m[i]);
            stack.push_back(c);
        }
    }
    // 每个局面的正确结果，以及每个着法的结果
    gametree::Stats st;
    std::vector<int> best(all.size());
    for (size_t i = 0; i < all.size(); ++i) best[i] = sgn(gametree::negamax(all[i], 9, st));
    std::printf("share of the 4520 positions where the chosen move keeps the result (win / draw / loss):\n%8s", "iters");
    const double cs[] = {0.25, 0.5, 1.0, 1.4, 2.0, 4.0};
    for (double c : cs) std::printf("  c=%-5.2f", c);
    std::printf("\n");
    Search<TicTacToe> s;
    for (int it : {10, 30, 100, 300, 1000, 3000}) {
        std::printf("%8d", it);
        for (double c : cs) {
            Pcg32 rng(it * 7 + int(c * 100));
            int keep = 0;
            for (size_t i = 0; i < all.size(); ++i) {
                int m = think(all[i], it, Params{c, false}, rng, &s);
                TicTacToe ch = all[i];
                ch.play(m);
                keep += sgn(-gametree::negamax(ch, 9, st)) == best[i];
            }
            std::printf("  %6.2f%%", 100.0 * keep / all.size());
        }
        std::printf("\n");
    }
}

static void sectionSpeed() {
    std::printf("\n== speed: connect four, empty board ==\n");
    for (bool smart : {false, true}) {
        Pcg32 rng(1);
        Search<ConnectFour> s(Params{1.4, smart});
        for (int it : {1000, 10000, 100000}) {
            double best = 1e30;
            for (int r = 0; r < 3; ++r) {
                auto t0 = Clock::now();
                think(ConnectFour{}, it, Params{1.4, smart}, rng, &s);
                best = std::min(best, msSince(t0));
            }
            int maxDepth = 0;
            for (size_t n = 0; n < s.nodes().size(); ++n) maxDepth = std::max(maxDepth, s.depthOf(int(n)));
            std::printf("%-7s %7d iterations: %8.2f ms (%5.2f us/iteration)  nodes %8zu  max depth %d\n", smart ? "smart" : "random", it, best,
                        best * 1000 / it, s.nodes().size(), maxDepth);
        }
    }
}

// 一个对局者：MCTS（iterations > 0）或 Alpha-Beta（depth > 0）
struct Player {
    const char* name;
    int iterations = 0;
    Params p{};
    int depth = 0;
};

struct MatchResult {
    int win = 0, draw = 0, loss = 0;
    double msA = 0, msB = 0;
    double score() const { return (win + 0.5 * draw) / (win + draw + loss); }
};

static int choose(const Player& pl, const ConnectFour& g, Pcg32& rng, Search<ConnectFour>& s, gametree::Table& table) {
    if (pl.iterations > 0) return think(g, pl.iterations, pl.p, rng, &s);
    gametree::Options o;
    o.table = &table;
    table.clear();
    gametree::Stats st;
    return gametree::iterativeDeepening(g, pl.depth, o, st).best;
}

// openings：开局随机摆好的步数（49 种两步开局，各执先后手）；repeat：重复几轮（MCTS 每轮的随机数不同）
static MatchResult match(const Player& A, const Player& B, int repeat, uint64_t seed) {
    MatchResult r;
    Pcg32 rng(seed);
    Search<ConnectFour> s;
    gametree::Table table(18);
    int movesA = 0, movesB = 0;
    for (int rep = 0; rep < repeat; ++rep)
        for (int open = 0; open < 49; ++open)
            for (int aFirst = 0; aFirst < 2; ++aFirst) {
                ConnectFour g;
                g.play(open / 7), g.play(open % 7);
                while (true) {
                    bool aToMove = (g.moves % 2 == 0) == bool(aFirst);
                    auto t0 = Clock::now();
                    int m = choose(aToMove ? A : B, g, rng, s, table);
                    (aToMove ? r.msA : r.msB) += msSince(t0);
                    ++(aToMove ? movesA : movesB);
                    g.play(m);
                    if (g.lastMoverWon()) {
                        ++(aToMove ? r.win : r.loss);
                        break;
                    }
                    if (g.full()) {
                        ++r.draw;
                        break;
                    }
                }
            }
    r.msA /= movesA, r.msB /= movesB;
    return r;
}

static void print(const char* a, const char* b, const MatchResult& r) {
    std::printf("%-22s vs %-22s  A %3d / %3d / %3d  A scores %5.1f%%  (%.2f / %.2f ms per move)\n", a, b, r.win, r.draw, r.loss,
                100 * r.score(), r.msA, r.msB);
}

static void sectionMatch() {
    std::printf("\n== MCTS (random rollouts, c = 1.4) vs alpha-beta (full search with heuristic), 196 games each ==\n");
    for (int it : {1000, 10000, 100000})
        for (int d : {2, 4, 6, 8}) {
            char a[32], b[32];
            std::snprintf(a, sizeof a, "mcts %d", it);
            std::snprintf(b, sizeof b, "alpha-beta d%d", d);
            print(a, b, match({a, it, {1.4, false}}, {b, 0, {}, d}, 2, 11 + it + d));
        }
}

// 补充：用时大致相当的对局
static void sectionMatch2() {
    std::printf("\n== roughly equal time per move ==\n");
    print("mcts 10000", "alpha-beta d10", match({"", 10000, {1.4, false}}, {"", 0, {}, 10}, 2, 61));
    print("mcts 100000", "alpha-beta d10", match({"", 100000, {1.4, false}}, {"", 0, {}, 10}, 2, 62));
    print("mcts 100000", "alpha-beta d12", match({"", 100000, {1.4, false}}, {"", 0, {}, 12}, 2, 63));
    print("smart 30000 c0.5", "alpha-beta d12", match({"", 30000, {0.5, true}}, {"", 0, {}, 12}, 2, 64));
}

static void sectionRollout() {
    std::printf("\n== smart rollouts vs random rollouts, 196 games each ==\n");
    for (int it : {1000, 10000, 100000}) {
        char a[32], b[32];
        std::snprintf(a, sizeof a, "smart %d", it);
        std::snprintf(b, sizeof b, "random %d", it);
        print(a, b, match({a, it, {1.4, true}}, {b, it, {1.4, false}}, 2, 21 + it));
    }
    // 大致相同的用时：smart 每次迭代更慢
    print("smart 3000", "random 10000", match({"", 3000, {1.4, true}}, {"", 10000, {1.4, false}}, 2, 31));
    print("smart 30000", "random 100000", match({"", 30000, {1.4, true}}, {"", 100000, {1.4, false}}, 2, 32));
    for (int d : {4, 6, 8}) {
        char b[32];
        std::snprintf(b, sizeof b, "alpha-beta d%d", d);
        print("smart 10000", b, match({"", 10000, {1.4, true}}, {b, 0, {}, d}, 2, 41 + d));
    }
}

static void sectionC() {
    std::printf("\n== exploration constant: mcts 10000 (random) with different c vs mcts 10000 c=1.4, 196 games each ==\n");
    for (double c : {0.1, 0.25, 0.5, 0.7, 1.0, 2.0, 4.0}) {
        char a[32];
        std::snprintf(a, sizeof a, "c = %.2f", c);
        print(a, "c = 1.40", match({a, 10000, {c, false}}, {"", 10000, {1.4, false}}, 2, 51 + int(c * 100)));
    }
}

// 浅层陷阱：某个着法会让对方在 k 步内强制获胜，而局面本身还有不输的着法。
// Alpha-Beta 搜 k+1 层一定能避开；MCTS 呢？
static void sectionTrap() {
    std::printf("\n== shallow traps ==\n");
    Pcg32 rng(77);
    gametree::Table table(18);
    struct Case {
        ConnectFour g;
        std::vector<int> trapMoves;  // 走了会在 depth 步内输掉的着法
        int depth;                   // 对方获胜需要的步数（含对方的最后一步）
    };
    std::vector<Case> cases[3];  // 对方 1 步、3 步、5 步内获胜
    int need = 200;
    while (cases[0].size() < size_t(need) || cases[1].size() < size_t(need) || cases[2].size() < size_t(need)) {
        ConnectFour g;
        bool ok = true;
        int plies = 4 + rng.below(20);
        for (int k = 0; k < plies && ok; ++k) {
            int m[7];
            int n = g.listMoves(m, false);
            g.play(m[rng.below(n)]);
            ok = !g.lastMoverWon() && !g.full();
        }
        if (!ok) continue;
        gametree::Options o;
        o.table = &table;
        gametree::Stats st;
        // 局面本身在 7 层内不输，并且自己也没有 7 层内的必胜（否则谈不上"陷阱"）
        table.clear();
        int rootScore = gametree::alphabeta(g, 7, -gametree::kInf, gametree::kInf, o, st);
        if (gametree::isWinScore(rootScore)) continue;
        int m[7];
        int n = g.listMoves(m, false);
        // 每个着法：对方最快几步强制获胜（1、3、5），0 表示 6 层内没有
        int fastest = 0;
        std::vector<int> moveLoss(n, 0);
        for (int i = 0; i < n; ++i) {
            ConnectFour c = g;
            c.play(m[i]);
            for (int k : {1, 3, 5}) {
                table.clear();
                int v = gametree::alphabeta(c, k, -gametree::kInf, gametree::kInf, o, st);
                if (v >= gametree::kWin) {
                    moveLoss[i] = k;
                    break;
                }
            }
        }
        // 只看"最浅的陷阱是 k 步"的局面：有 k 步陷阱，没有更浅的
        for (int i = 0; i < n; ++i)
            if (moveLoss[i] && (!fastest || moveLoss[i] < fastest)) fastest = moveLoss[i];
        if (!fastest) continue;
        bool same = true;  // 只保留所有陷阱都是同一深度的局面，这样"掉进陷阱"的深度是明确的
        for (int i = 0; i < n; ++i) same &= !moveLoss[i] || moveLoss[i] == fastest;
        if (!same) continue;
        Case cs{g, {}, fastest};
        for (int i = 0; i < n; ++i)
            if (moveLoss[i]) cs.trapMoves.push_back(m[i]);
        auto& bucket = cases[fastest / 2];
        if (bucket.size() < size_t(need)) bucket.push_back(cs);
    }
    std::printf("positions: no forced result within 7 plies; every losing move loses within exactly k plies\n");
    for (int k = 0; k < 3; ++k) {
        double traps = 0, moves = 0;
        for (const Case& cs : cases[k]) {
            int m[7];
            traps += cs.trapMoves.size(), moves += cs.g.listMoves(m, false);
        }
        std::printf("k = %d: %zu positions, %.1f of %.1f moves are traps on average\n", 2 * k + 1, cases[k].size(), traps / cases[k].size(), moves / cases[k].size());
    }
    std::printf("%-26s %12s %12s %12s\n", "player", "lose in 1", "lose in 3", "lose in 5");
    auto rate = [&](const char* name, auto pick) {
        std::printf("%-26s", name);
        for (auto& bucket : cases) {
            int fell = 0, trapsTotal = 0;
            for (const Case& cs : bucket) {
                int mv = pick(cs.g);
                for (int t : cs.trapMoves) fell += t == mv;
                trapsTotal += int(cs.trapMoves.size());
            }
            std::printf(" %11.1f%%", 100.0 * fell / bucket.size());
        }
        std::printf("\n");
    };
    rate("random move", [&](const ConnectFour& g) {
        int m[7];
        int n = g.listMoves(m, false);
        return m[rng.below(n)];
    });
    Search<ConnectFour> s;
    for (bool smart : {false, true})
        for (int it : {1000, 10000, 100000}) {
            char name[48];
            std::snprintf(name, sizeof name, "mcts %s %d", smart ? "smart" : "random", it);
            rate(name, [&](const ConnectFour& g) { return think(g, it, Params{1.4, smart}, rng, &s); });
        }
    for (int d : {2, 4, 6}) {
        char name[32];
        std::snprintf(name, sizeof name, "alpha-beta d%d", d);
        rate(name, [&](const ConnectFour& g) {
            gametree::Options o;
            o.table = &table;
            table.clear();
            gametree::Stats st;
            return gametree::iterativeDeepening(g, d, o, st).best;
        });
    }
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const char* s = argc > 1 ? argv[1] : "";
    auto want = [&](const char* name) { return !*s || !std::strcmp(s, name); };
    if (want("ttt")) sectionTtt();
    if (want("speed")) sectionSpeed();
    if (want("match")) sectionMatch();
    if (want("match2")) sectionMatch2();
    if (want("rollout")) sectionRollout();
    if (want("c")) sectionC();
    if (want("trap")) sectionTrap();
}
