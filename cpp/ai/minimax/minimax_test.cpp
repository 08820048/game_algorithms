#include "minimax.h"

#include <cstdio>
#include <unordered_set>

#include "../../foundations/random/random.h"
#include "check.h"
#include "games.h"

using namespace ga;
using namespace ga::gametree;

// 随机走若干步得到一个还没分出胜负的局面
template <class G>
static bool randomPosition(Pcg32& rng, int plies, G& g) {
    g = G{};
    for (int k = 0; k < plies; ++k) {
        int moves[G::kMaxMoves];
        int n = g.listMoves(moves, false);
        g.play(moves[rng.below(n)]);
        if (g.lastMoverWon() || g.full()) return false;
    }
    return true;
}

// 暴力检查连线：逐个窗口看是否 4 子都在
template <int W, int H>
static bool alignedBrute(uint64_t p) {
    for (uint64_t w : Connect<W, H>::windows())
        if ((p & w) == w) return true;
    return false;
}

// 对一个局面检查所有搜索变体都与 negamax 一致
template <class G>
static void checkAllVariants(const G& g, int depth, Table& table, long long& compared) {
    Stats st;
    const int ref = negamax(g, depth, st);
    CHECK(minimax(g, depth, true, st) == ref);  // 让轮到走的一方当 MAX，两种写法相同
    for (bool order : {false, true}) {
        Options o;
        o.order = order;
        CHECK(alphabeta(g, depth, -kInf, kInf, o, st) == ref);
        table.clear();
        o.table = &table;
        CHECK(alphabeta(g, depth, -kInf, kInf, o, st) == ref);
        o.prune = false;
        table.clear();
        CHECK(alphabeta(g, depth, -kInf, kInf, o, st) == ref);
    }
    Options o;
    o.table = &table;
    table.clear();
    RootResult id = iterativeDeepening(g, depth, o, st);
    CHECK(id.score == ref);
    // 根节点：最好着法的子局面确实是这个分数；被标为上界的着法确实不比它好
    for (size_t i = 0; i < id.moves.size(); ++i) {
        G child = g;
        child.play(id.moves[i]);
        int exact = -negamax(child, id.depth - 1, st);
        if (id.bounds[i] == Bound::Exact) CHECK(exact == id.scores[i]);
        else CHECK(exact <= id.scores[i]);
        if (id.moves[i] == id.best) CHECK(exact == ref || id.depth < depth);
    }
    compared += 1;
}

int main() {
    // ---- 位棋盘：连线判断与逐个窗口的暴力检查一致（包括列与列之间不会"绕过去"） ----
    {
        Pcg32 rng(7);
        int positions = 0;
        for (int t = 0; t < 20000; ++t) {
            ConnectFour g;
            int plies = rng.below(42);
            for (int k = 0; k < plies && !g.full(); ++k) {
                int moves[7];
                int n = g.listMoves(moves, false);
                g.play(moves[rng.below(n)]);
                CHECK(g.lastMoverWon() == (alignedBrute<7, 6>(g.current ^ g.mask)));
                CHECK(ConnectFour::aligned(g.current) == (alignedBrute<7, 6>(g.current)));
                ++positions;
            }
        }
        std::printf("bitboard: aligned() matches brute force on %d positions\n", positions);
        ConnectFour g;
        CHECK(g.playSequence("1111122"));  // 列 1 放 5 子、列 2 放 2 子：列 1 顶上和列 2 底部相邻，不是竖线
        CHECK(!ConnectFour::aligned(g.current) && !ConnectFour::aligned(g.current ^ g.mask));
        ConnectFour h;
        CHECK(h.playSequence("1213141"));  // 先手在列 1 竖着连成 4 子
        CHECK(h.lastMoverWon());
        ConnectFour d;
        CHECK(d.playSequence("12233434464"));  // 先手沿斜线 (1,1) (2,2) (3,3) (4,4)
        CHECK(d.lastMoverWon());
        using C76 = Connect<7, 6>;
        CHECK(C76::centerOrder(0) == 3 && C76::centerOrder(1) == 2 && C76::centerOrder(6) == 6);
        CHECK((Connect<6, 5>::centerOrder(5)) == 0);
        CHECK(C76::windows().size() == 69);
    }

    // ---- 估值：交换双方的棋子，估值取反 ----
    {
        Pcg32 rng(8);
        for (int t = 0; t < 2000; ++t) {
            ConnectFour g;
            if (!randomPosition(rng, rng.below(30), g)) continue;
            ConnectFour swapped = g;
            swapped.current = g.current ^ g.mask;
            CHECK(swapped.evaluate() == -g.evaluate());
        }
        ConnectFour g;
        CHECK(g.playSequence("152537"));  // 先手在底行连了 3 子，后手的棋子零散：先手的估值应为正
        CHECK(g.evaluate() > 0);
    }

    // ---- 井字棋：整棵博弈树 ----
    {
        Stats st;
        TicTacToe empty;
        CHECK(minimax(empty, 9, true, st) == 0);  // 双方都不犯错是和棋
        std::printf("tic-tac-toe: full minimax tree has %llu nodes\n", (unsigned long long)st.nodes);
        CHECK(st.nodes == 549946);

        // 枚举所有可能出现的局面，每个局面都比较所有变体
        std::vector<TicTacToe> all;
        std::unordered_set<uint64_t> seen;
        std::vector<TicTacToe> stack{empty};
        while (!stack.empty()) {
            TicTacToe g = stack.back();
            stack.pop_back();
            if (!seen.insert(g.key()).second) continue;
            all.push_back(g);
            int s;
            if (terminal(g, s)) continue;
            int moves[9];
            int n = g.listMoves(moves, false);
            for (int i = 0; i < n; ++i) {
                TicTacToe c = g;
                c.play(moves[i]);
                stack.push_back(c);
            }
        }
        CHECK(all.size() == 5478);

        // 剪枝的效率：上面的检查只保证"值正确"，少剪枝（比如截断条件写成 alpha > beta）也能通过，
        // 所以再检查节点数。完美排序下界约 2000 个节点，不剪枝是 549946 个
        Stats ab;
        Options o;
        searchRoot(empty, 9, o, ab);
        std::printf("tic-tac-toe: alpha-beta with ordering visits %llu nodes\n", (unsigned long long)ab.nodes);
        CHECK(ab.nodes < 9000);
        Table table(12);
        long long compared = 0;
        for (const TicTacToe& g : all) {
            int s;
            if (!terminal(g, s)) checkAllVariants(g, 9, table, compared);
        }
        std::printf("tic-tac-toe: %zu reachable positions, all variants agree on %lld non-terminal ones\n", all.size(), compared);
    }

    // ---- 重力四子棋：随机局面、固定深度，所有变体与 negamax 的值相同 ----
    {
        Pcg32 rng(9);
        Table table(16);
        long long compared = 0;
        for (int t = 0; t < 400; ++t) {
            ConnectFour g;
            if (!randomPosition(rng, rng.below(36), g)) continue;
            int depth = 1 + rng.below(t % 4 == 0 ? 6 : 5);
            checkAllVariants(g, depth, table, compared);
        }
        std::printf("connect four: all variants agree with negamax on %lld random positions (depth 1-6)\n", compared);
    }

    // ---- 胜负分：能立刻赢就立刻赢，分数 = kWin + 获胜后剩下的空格 ----
    {
        ConnectFour g;
        CHECK(g.playSequence("121314"));  // 先手在列 1 有 3 子，再走列 1 立即获胜（走别处也许稍后也能赢）
        Table table(14);
        Options o;
        o.table = &table;
        Stats st;
        for (int d = 1; d <= 7; ++d) {
            RootResult r = searchRoot(g, d, o, st);
            CHECK(r.best == 0);
            CHECK(r.score == kWin + g.empties() - 1);
        }
        ConnectFour h;
        CHECK(h.playSequence("22334"));  // 先手在底行连了 3 子、两头都空：后手只能堵一头，无论怎么走都会输
        RootResult r = searchRoot(h, 3, o, st);
        CHECK(r.score == -(kWin + h.empties() - 2));  // 后手走一步，先手再走一步获胜
    }

    // ---- fail-soft 的含义：任意窗口下，返回值落在窗口外时是正确方向的界 ----
    {
        Pcg32 rng(10);
        Table table(16);
        int low = 0, high = 0, exact = 0;
        for (int t = 0; t < 1500; ++t) {
            ConnectFour g;
            if (!randomPosition(rng, rng.below(36), g)) continue;
            int depth = 1 + rng.below(5);
            Stats st;
            int ref = negamax(g, depth, st);
            int a = int(rng.below(81)) - 40, b = a + 1 + int(rng.below(40));
            if (rng.below(6) == 0) a = -kWin - 50, b = -kWin + 50;  // 窗口落在胜负分附近
            Options o;
            o.order = rng.below(2);
            if (rng.below(2)) table.clear(), o.table = &table;
            int v = alphabeta(g, depth, a, b, o, st);
            if (v <= a) {
                CHECK(ref <= v);
                ++low;
            } else if (v >= b) {
                CHECK(ref >= v);
                ++high;
            } else {
                CHECK(ref == v);
                ++exact;
            }
        }
        std::printf("fail-soft windows: %d fail-low, %d fail-high, %d exact, all bounds correct\n", low, high, exact);
    }

    // ---- 小棋盘解到终局：置换表 + 排序的结果与不剪枝的完整搜索相同 ----
    {
        Pcg32 rng(11);
        Table table(18);
        int solved = 0;
        for (int t = 0; t < 60; ++t) {
            Connect<4, 4> g;
            if (!randomPosition(rng, 2 + rng.below(6), g)) continue;
            Stats st;
            int ref = negamax(g, 16, st);
            Options o;
            o.table = &table;
            table.clear();
            CHECK(alphabeta(g, 16, -kInf, kInf, o, st) == ref);
            CHECK(iterativeDeepening(g, 16, o, st).score == ref);
            ++solved;
        }
        std::printf("4x4 board: %d positions solved to the end, identical to full negamax\n", solved);
    }

    return report("minimax_test");
}
