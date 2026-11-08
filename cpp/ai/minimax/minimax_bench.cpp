// 博弈树搜索：剪枝、排序、置换表各省多少节点，搜索深度与估值对棋力的影响，小棋盘解到终局（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/minimax/minimax_bench.cpp -o minimax_bench && ./minimax_bench [section]
//   section：ttt / depth / mid / parity / match / solve，省略时全部运行
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../../foundations/random/random.h"
#include "games.h"
#include "minimax.h"

using namespace ga;
using namespace ga::gametree;
using Clock = std::chrono::steady_clock;

static double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

struct NoEval : ConnectFour {  // 只认胜负、不估值的四子棋
    int evaluate() const { return 0; }
};

struct Tempo : ConnectFour {  // 估值加上"轮到我走"的先手分，抵消奇偶层交替偏高偏低
    int evaluate() const { return ConnectFour::evaluate() + 8; }
};

struct Variant {
    const char* name;
    bool prune, order, table, deepening;
};
static const Variant kVariants[] = {
    {"minimax", false, false, false, false},
    {"ab", true, false, false, false},
    {"ab+order", true, true, false, false},
    {"ab+order+tt", true, true, true, false},
    {"ab+tt+id", true, true, true, true},
};

template <class G>
static RootResult run(const G& g, int depth, const Variant& v, Table& table, Stats& st) {
    Options o;
    o.prune = v.prune, o.order = v.order, o.table = v.table ? &table : nullptr;
    return v.deepening ? iterativeDeepening(g, depth, o, st) : searchRoot(g, depth, o, st);
}

static void sectionTtt() {
    std::printf("\n== tic-tac-toe, full tree from the empty board ==\n");
    Table table(16);
    for (const Variant& v : kVariants) {
        Stats st;
        table.clear();
        auto t0 = Clock::now();
        RootResult r = run(TicTacToe{}, 9, v, table, st);
        std::printf("%-12s nodes %9llu  cutoffs %7llu  tt %6llu  score %d  %.2f ms\n", v.name, (unsigned long long)st.nodes,
                    (unsigned long long)st.cutoffs, (unsigned long long)st.ttHits, r.score, msSince(t0));
    }
}

// 完美排序时 alpha-beta 在第 k 层访问的节点数（Knuth & Moore 1975）：b^ceil(k/2) + b^floor(k/2) - 1。
// 这里对 k = 0..d 求和，得到整棵"最小树"的节点数，与搜索统计的节点数口径相同
static double minimalNodes(double b, int d) {
    double sum = 0;
    for (int k = 0; k <= d; ++k) sum += std::pow(b, (k + 1) / 2) + std::pow(b, k / 2) - 1;
    return sum;
}

static void sectionDepth() {
    std::printf("\n== connect four, empty board, nodes by depth ==\n");
    Table table(22);
    std::printf("depth");
    for (const Variant& v : kVariants) std::printf(" %14s", v.name);
    std::printf(" %12s %10s\n", "minimal", "ms(tt+id)");
    for (int d = 1; d <= 14; ++d) {
        std::printf("%5d", d);
        double ms = 0;
        for (const Variant& v : kVariants) {
            if (!v.prune && d > 9) {
                std::printf(" %14s", "-");
                continue;
            }
            if (!v.order && d > 12) {
                std::printf(" %14s", "-");
                continue;
            }
            Stats st;
            table.clear();
            auto t0 = Clock::now();
            run(ConnectFour{}, d, v, table, st);
            ms = msSince(t0);
            std::printf(" %14llu", (unsigned long long)st.nodes);
        }
        std::printf(" %12.0f %10.1f\n", minimalNodes(7, d), ms);
    }
}

// 随机的中局局面（随机走 8~16 步），深度 8：平均节点数、时间、有效分支因子
static void sectionMid() {
    std::printf("\n== connect four, 100 random midgame positions, depth 8 ==\n");
    Pcg32 rng(2026);
    std::vector<ConnectFour> pos;
    while (pos.size() < 100) {
        ConnectFour g;
        bool ok = true;
        int plies = 8 + rng.below(9);
        for (int k = 0; k < plies && ok; ++k) {
            int m[7];
            int n = g.listMoves(m, false);
            g.play(m[rng.below(n)]);
            ok = !g.lastMoverWon();
        }
        if (ok) pos.push_back(g);
    }
    Table table(22);
    const int depth = 8;
    double base = 0;
    for (const Variant& v : kVariants) {
        double nodes = 0, ms = 0;
        for (const ConnectFour& g : pos) {
            Stats st;
            table.clear();
            auto t0 = Clock::now();
            run(g, depth, v, table, st);
            ms += msSince(t0);
            nodes += double(st.nodes);
        }
        nodes /= pos.size(), ms /= pos.size();
        if (!v.prune) base = nodes;
        std::printf("%-12s avg nodes %10.0f  (%5.2f%% of minimax)  eff. branching %.2f  %.2f ms  %.0f ns/node\n", v.name, nodes,
                    100 * nodes / base, std::pow(nodes, 1.0 / depth), ms, ms * 1e6 / nodes);
    }
}

// 根节点的估值随深度的变化：奇数层和偶数层是否交替偏高
static void sectionParity() {
    std::printf("\n== root score by depth (empty board and 5 random openings) ==\n");
    Table table(22);
    Pcg32 rng(5);
    for (int t = 0; t < 6; ++t) {
        ConnectFour g;
        for (int k = 0; t > 0 && k < 4; ++k) g.play(rng.below(7));
        std::printf("opening %d:", t);
        Tempo tg;
        tg.current = g.current, tg.mask = g.mask, tg.moves = g.moves;
        for (int d = 1; d <= 12; ++d) {
            Stats st;
            Options o;
            o.table = &table;
            table.clear();
            RootResult r = iterativeDeepening(g, d, o, st);
            std::printf(" %4d", r.score);
        }
        std::printf("\n   +tempo:");
        for (int d = 1; d <= 12; ++d) {
            Stats st;
            Options o;
            o.table = &table;
            table.clear();
            std::printf(" %4d", iterativeDeepening(tg, d, o, st).score);
        }
        std::printf("\n");
    }
}

// 对局：两个 AI 从 343 种开局（先走 3 步的所有组合）开始，各执先后手一次
template <class A, class B>
static void match(const char* label, int depthA, int depthB, int* wdl) {
    Table ta(20), tb(20);
    wdl[0] = wdl[1] = wdl[2] = 0;
    double msA = 0, msB = 0;
    int movesA = 0, movesB = 0;
    for (int open = 0; open < 343; ++open)
        for (int aFirst = 0; aFirst < 2; ++aFirst) {
            ConnectFour g;
            g.play(open / 49), g.play(open / 7 % 7), g.play(open % 7);
            int result = 0;  // 1：A 胜，-1：B 胜
            while (true) {
                bool aToMove = (g.moves % 2 == 0) == bool(aFirst);  // aFirst：A 执先手
                Stats st;
                Options o;
                auto t0 = Clock::now();
                int m;
                if (aToMove) {
                    o.table = &ta, ta.clear();
                    A a;
                    a.current = g.current, a.mask = g.mask, a.moves = g.moves;
                    m = iterativeDeepening(a, depthA, o, st).best;
                    msA += msSince(t0), ++movesA;
                } else {
                    o.table = &tb, tb.clear();
                    B b;
                    b.current = g.current, b.mask = g.mask, b.moves = g.moves;
                    m = iterativeDeepening(b, depthB, o, st).best;
                    msB += msSince(t0), ++movesB;
                }
                g.play(m);
                if (g.lastMoverWon()) {
                    result = aToMove ? 1 : -1;
                    break;
                }
                if (g.full()) break;
            }
            ++wdl[result == 1 ? 0 : result == 0 ? 1 : 2];
        }
    std::printf("%-34s A wins %3d  draws %3d  B wins %3d   (%.2f / %.2f ms per move)\n", label, wdl[0], wdl[1], wdl[2], msA / movesA,
                msB / movesB);
}

static void sectionMatch() {
    std::printf("\n== matches: 686 games each (343 openings x both colours) ==\n");
    int wdl[3];
    char label[64];
    const int pairs[][2] = {{1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}, {8, 9}, {9, 10}, {2, 4}, {4, 6}, {6, 8}, {8, 10}, {3, 5}, {5, 7}, {7, 9}, {2, 8}};
    for (auto& p : pairs) {
        std::snprintf(label, sizeof label, "eval d%d (A) vs eval d%d (B)", p[0], p[1]);
        match<ConnectFour, ConnectFour>(label, p[0], p[1], wdl);
    }
    for (int d : {4, 6, 8}) {
        std::snprintf(label, sizeof label, "no-eval d%d (A) vs eval d%d (B)", d, d);
        match<NoEval, ConnectFour>(label, d, d, wdl);
    }
    for (int d : {5, 6}) {
        std::snprintf(label, sizeof label, "tempo d%d (A) vs eval d%d (B)", d, d);
        match<Tempo, ConnectFour>(label, d, d, wdl);
    }
    match<NoEval, ConnectFour>("no-eval d10 (A) vs eval d4 (B)", 10, 4, wdl);
    match<NoEval, ConnectFour>("no-eval d12 (A) vs eval d6 (B)", 12, 6, wdl);
}

template <int W, int H>
static void solve(Table& table, bool useTable) {
    Stats st;
    Options o;
    o.table = useTable ? &table : nullptr;
    table.clear();
    auto t0 = Clock::now();
    Connect<W, H> g;
    int s = alphabeta(g, W * H, -kInf, kInf, o, st);
    const char* who = s > 0 ? "first player wins" : s < 0 ? "second player wins" : "draw";
    int empties = s > 0 ? s - kWin : s < 0 ? -s - kWin : 0;
    std::printf("%dx%d %s: %-18s (%d empty cells left)  nodes %12llu  %9.1f ms\n", W, H, useTable ? "tt   " : "no-tt", who, empties,
                (unsigned long long)st.nodes, msSince(t0));
}

static void sectionSolve() {
    std::printf("\n== solving small boards to the end (alpha-beta + ordering) ==\n");
    Table table(24);
    for (bool tt : {false, true}) {
        solve<4, 4>(table, tt);
        solve<5, 4>(table, tt);
        solve<4, 5>(table, tt);
        solve<5, 5>(table, tt);
        solve<6, 4>(table, tt);
    }
    solve<6, 5>(table, true);
    solve<5, 6>(table, true);
}

int main(int argc, char** argv) {
    const char* s = argc > 1 ? argv[1] : "";
    auto want = [&](const char* name) { return !*s || !std::strcmp(s, name); };
    if (want("ttt")) sectionTtt();
    if (want("depth")) sectionDepth();
    if (want("mid")) sectionMid();
    if (want("parity")) sectionParity();
    if (want("match")) sectionMatch();
    if (want("solve")) sectionSolve();
}
