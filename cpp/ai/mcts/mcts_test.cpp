#include "mcts.h"

#include <cmath>
#include <cstdio>
#include <unordered_set>

#include "../minimax/games.h"
#include "../minimax/minimax.h"
#include "check.h"

using namespace ga;
using namespace ga::mcts;
using gametree::ConnectFour;
using gametree::TicTacToe;

// 井字棋双方都均匀随机地走：后手获胜的概率 + 0.5 × 和棋的概率（精确值，逐个分支加权求和）
static double randomPlayoutValue(const TicTacToe& g, bool secondToMove, int depth = 0) {
    int moves[9];
    int n = g.listMoves(moves, false);
    double sum = 0;
    for (int i = 0; i < n; ++i) {
        TicTacToe c = g;
        c.play(moves[i]);
        bool moverIsSecond = secondToMove;
        if (c.lastMoverWon()) sum += moverIsSecond ? 1.0 : 0.0;
        else if (c.full()) sum += 0.5;
        else sum += randomPlayoutValue(c, !secondToMove, depth + 1);
    }
    return sum / n;
}

// 结果的符号：+1 胜、0 和、-1 负
static int sign(int v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }

template <class G>
static void checkInvariants(const Search<G>& s, int iterations, long long& nodesChecked) {
    const auto& nodes = s.nodes();
    CHECK(int(nodes[0].visits) == iterations);
    for (size_t n = 0; n < nodes.size(); ++n) {
        const Node& nd = nodes[n];
        CHECK(nd.reward >= 0 && nd.reward <= nd.visits);
        if (nd.terminal) {
            CHECK(nd.first < 0);
            CHECK(nd.reward == nd.visits * double(nd.terminalReward));
            continue;
        }
        if (nd.first < 0) {  // 还没展开的节点最多被模拟过一次
            CHECK(nd.visits <= 1);
            continue;
        }
        // 经过这个节点的每一次访问都来自某个子节点，再加上它自己第一次被访问时的那次模拟（根节点没有）
        uint32_t childVisits = 0;
        double fromChildren = 0;
        for (int c = nd.first; c < nd.first + nd.count; ++c) {
            CHECK(nodes[c].parent == int(n));
            childVisits += nodes[c].visits;
            fromChildren += nodes[c].visits - nodes[c].reward;  // 子节点的收益换成这个节点的视角
        }
        uint32_t own = n == 0 ? 0 : 1;
        CHECK(nd.visits == childVisits + own);
        double r0 = nd.reward - fromChildren;  // 第一次模拟的收益：0、0.5 或 1（都是 0.5 的倍数，double 精确）
        CHECK(n == 0 ? r0 == 0 : (r0 == 0 || r0 == 0.5 || r0 == 1));
        ++nodesChecked;
    }
}

int main() {
    // ---- 模拟：随机对局的平均收益与精确概率一致 ----
    {
        TicTacToe empty;
        double exact = randomPlayoutValue(empty, false);
        Search<TicTacToe> s;
        s.reset(empty);
        Pcg32 rng(1);
        const int samples = 400000;
        double sum = 0;
        for (int i = 0; i < samples; ++i) sum += s.simulate({0, empty}, rng);
        double mean = sum / samples, sigma = std::sqrt(0.25 / samples);  // 收益在 [0, 1]，方差不超过 1/4
        std::printf("random playout from the empty board: second player scores %.5f (exact %.5f, %.1f sigma)\n", mean, exact,
                    std::abs(mean - exact) / sigma);
        CHECK(std::abs(mean - exact) < 4 * sigma);
    }

    // ---- 启发式模拟：能一步赢就一定赢，否则对方能一步赢就一定堵 ----
    {
        Search<ConnectFour> s({1.4, true});
        Pcg32 rng(2);
        int wins = 0, blocks = 0;
        for (int t = 0; t < 20000; ++t) {
            ConnectFour g;
            bool ok = true;
            int plies = rng.below(30);
            for (int k = 0; k < plies && ok; ++k) {
                int m[7];
                int n = g.listMoves(m, false);
                g.play(m[rng.below(n)]);
                ok = !g.lastMoverWon() && !g.full();
            }
            if (!ok) continue;
            int m[7];
            int n = g.listMoves(m, false);
            bool canWin = false, mustBlock = false;
            for (int i = 0; i < n; ++i) {
                ConnectFour c = g;
                c.play(m[i]);
                canWin |= c.lastMoverWon();
                ConnectFour o = g;
                o.current ^= o.mask;
                o.play(m[i]);
                mustBlock |= o.lastMoverWon();
            }
            int pick = s.rolloutMove(g, rng);
            ConnectFour after = g;
            after.play(pick);
            if (canWin) {
                CHECK(after.lastMoverWon());
                ++wins;
            } else if (mustBlock) {
                ConnectFour o = g;
                o.current ^= o.mask;
                o.play(pick);
                CHECK(o.lastMoverWon());  // 堵的正是对方的获胜点
                ++blocks;
            }
        }
        std::printf("smart rollout: took the win in %d positions, blocked in %d\n", wins, blocks);
    }

    // ---- 树的不变量：访问次数与收益在父子之间精确守恒；UCB 选择与暴力计算一致 ----
    {
        Pcg32 rng(3);
        long long checked = 0, ucbChecked = 0;
        for (int t = 0; t < 60; ++t) {
            Params p{0.3 + rng.nextDouble() * 2, bool(rng.below(2))};
            Search<ConnectFour> s(p);
            ConnectFour g;
            for (int k = 0; k < int(rng.below(12)); ++k) {
                int m[7];
                int n = g.listMoves(m, false);
                ConnectFour c = g;
                c.play(m[rng.below(n)]);
                if (c.lastMoverWon()) break;
                g = c;
            }
            int iterations = 1 + rng.below(3000);
            s.reset(g);
            for (int i = 0; i < iterations; ++i) s.iterate(rng);
            checkInvariants(s, iterations, checked);
            // 最终选择：访问次数最多的子节点
            const Node& root = s.nodes()[0];
            uint32_t most = 0;
            for (int c = root.first; c < root.first + root.count; ++c) most = std::max(most, s.nodes()[c].visits);
            CHECK(s.nodes()[s.bestChild()].visits == most);
            // 暴力 UCB：对每个已展开的节点，重新算一遍所有子节点的分数取最大
            const auto& nodes = s.nodes();
            for (size_t n = 0; n < nodes.size(); ++n) {
                const Node& nd = nodes[n];
                if (nd.first < 0 || nd.terminal) continue;
                int expect = -1;
                double best = -1e300;
                for (int c = nd.first; c < nd.first + nd.count; ++c) {
                    double score = nodes[c].visits == 0 ? 1e300
                                 : nodes[c].reward / nodes[c].visits + p.c * std::sqrt(std::log(double(nd.visits)) / nodes[c].visits);
                    if (score > best) best = score, expect = c;
                }
                CHECK(s.selectChild(int(n)) == expect);
                ++ucbChecked;
            }
        }
        std::printf("tree invariants hold on %lld expanded nodes; UCB matches brute force on %lld nodes\n", checked, ucbChecked);
    }

    // ---- 井字棋：迭代足够多时，选出的着法保持局面的博弈论结果（胜 / 和 / 负）----
    {
        std::vector<TicTacToe> all;
        std::unordered_set<uint64_t> seen;
        std::vector<TicTacToe> stack{TicTacToe{}};
        while (!stack.empty()) {
            TicTacToe g = stack.back();
            stack.pop_back();
            if (!seen.insert(g.key()).second) continue;
            if (g.lastMoverWon() || g.full()) continue;
            all.push_back(g);
            int m[9];
            int n = g.listMoves(m, false);
            for (int i = 0; i < n; ++i) {
                TicTacToe c = g;
                c.play(m[i]);
                stack.push_back(c);
            }
        }
        CHECK(all.size() == 4520);
        Pcg32 rng(4);
        int keeps = 0;
        Search<TicTacToe> s;
        gametree::Stats st;
        for (const TicTacToe& g : all) {
            int move = think(g, 3000, Params{}, rng, &s);
            int best = gametree::negamax(g, 9, st);
            TicTacToe c = g;
            c.play(move);
            keeps += sign(-gametree::negamax(c, 9, st)) == sign(best);
        }
        std::printf("tic-tac-toe, 3000 iterations: the chosen move keeps the game-theoretic result in %d / %zu positions\n", keeps,
                    all.size());
        CHECK(keeps == int(all.size()));
    }

    return report("mcts_test");
}
