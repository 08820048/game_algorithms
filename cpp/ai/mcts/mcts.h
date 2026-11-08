#pragma once

// 蒙特卡洛树搜索（MCTS），使用 UCT（Upper Confidence bounds applied to Trees）选择子节点。
// 模板参数 G 是双人轮流、完全信息的游戏，接口与 ai/minimax/games.h 相同：
//   play / lastMoverWon / full / listMoves(out, ordered) / kMaxMoves
// 启发式模拟（smart rollout）还要求局面用 current（轮到走的一方的棋子）和 mask（所有棋子）表示。
//
// 每次迭代分四步：
//   1. 选择：从根出发，按 UCB 公式一路选子节点，直到一个还没被访问过的节点（或终局）
//   2. 扩展：经过一个已被访问、但还没有子节点的节点时，为它创建全部子节点
//   3. 模拟：从新节点开始随机走到终局，得到胜（1）、和（0.5）、负（0）
//   4. 回传：沿路径往回，每个节点访问次数 +1，累加收益；每往上一层，收益换成对方的视角（1 - r）

#include <cmath>
#include <cstdint>
#include <vector>

#include "../../foundations/random/random.h"

namespace ga::mcts {

// #region node
// 节点的收益站在"走到这个节点的那一方"（也就是父节点轮到走的一方）：
// 父节点挑选子节点时，直接比较子节点的平均收益即可
struct Node {
    int parent = -1;
    int first = -1;      // 子节点在数组中连续存放：[first, first + count)
    int8_t count = 0;
    int8_t move = -1;    // 从父节点走到这里的着法
    bool terminal = false;
    float terminalReward = 0;  // 终局节点：走到这里的一方得到的收益
    uint32_t visits = 0;
    double reward = 0;   // 收益之和；平均收益 = reward / visits
};
// #endregion

struct Params {
    double c = 1.4;             // 探索常数：越大越愿意尝试访问次数少的着法
    bool smartRollout = false;  // 模拟时能赢就赢、必须堵就堵，其余随机
};

template <class G>
class Search {
public:
    explicit Search(Params p = {}) : params_(p) {}

    void reset(const G& root) {
        root_ = root;
        nodes_.assign(1, Node{});
    }

    struct Leaf {
        int node;
        G state;
        int expanded = -1;  // 这次迭代为哪个节点创建了子节点（-1：没有扩展）
    };

    // #region select
    // 选择 + 扩展：返回这次迭代要模拟的节点
    Leaf select() {
        int n = 0;
        G g = root_;
        int expanded = -1;
        while (!nodes_[n].terminal) {
            if (n != 0 && nodes_[n].visits == 0) break;  // 新节点：先模拟它自己
            if (nodes_[n].first < 0) {  // 第二次经过（根节点是第一次）：创建全部子节点
                expand(n, g);
                expanded = n;
            }
            n = selectChild(n);
            g.play(nodes_[n].move);
        }
        return {n, g, expanded};
    }

    // UCB1：平均收益 + c * sqrt(ln(父节点访问次数) / 子节点访问次数)。
    // 没访问过的子节点优先（按着法顺序取第一个），相当于 UCB 为无穷大
    int selectChild(int n) const {
        const Node& p = nodes_[n];
        double logN = std::log(double(p.visits));
        int best = -1;
        double bestScore = -1;
        for (int i = p.first; i < p.first + p.count; ++i) {
            const Node& c = nodes_[i];
            if (c.visits == 0) return i;
            double score = c.reward / c.visits + params_.c * std::sqrt(logN / c.visits);
            if (score > bestScore) bestScore = score, best = i;
        }
        return best;
    }
    // #endregion

    // #region simulate
    // 模拟：返回"走到这个节点的那一方"的收益
    double simulate(const Leaf& leaf, Pcg32& rng) const {
        const Node& nd = nodes_[leaf.node];
        if (nd.terminal) return nd.terminalReward;
        G g = leaf.state;
        for (int ply = 1;; ++ply) {
            g.play(rolloutMove(g, rng));
            // 第 1、3、5……步是对方走的：对方连成线，我输
            if (g.lastMoverWon()) return ply % 2 ? 0.0 : 1.0;
            if (g.full()) return 0.5;
        }
    }

    int rolloutMove(const G& g, Pcg32& rng) const {
        int moves[G::kMaxMoves];
        int n = g.listMoves(moves, false);
        if (params_.smartRollout) {
            for (int i = 0; i < n; ++i) {  // 能一步赢就赢
                G t = g;
                t.play(moves[i]);
                if (t.lastMoverWon()) return moves[i];
            }
            G opp = g;
            opp.current ^= opp.mask;  // 假装轮到对方：对方哪一步能赢，我就堵哪一步
            for (int i = 0; i < n; ++i) {
                G t = opp;
                t.play(moves[i]);
                if (t.lastMoverWon()) return moves[i];
            }
        }
        return moves[rng.below(n)];
    }
    // #endregion

    // #region backprop
    void backpropagate(int n, double reward) {
        for (; n >= 0; n = nodes_[n].parent) {
            nodes_[n].visits += 1;
            nodes_[n].reward += reward;
            reward = 1 - reward;  // 父节点是另一方走到的
        }
    }

    void iterate(Pcg32& rng) {
        Leaf leaf = select();
        backpropagate(leaf.node, simulate(leaf, rng));
    }

    // 最终选择访问次数最多的着法（比"平均收益最高"更稳：访问少的着法平均收益波动很大）
    int bestChild() const {
        const Node& r = nodes_[0];
        int best = -1;
        for (int i = r.first; i < r.first + r.count; ++i)
            if (best < 0 || nodes_[i].visits > nodes_[best].visits ||
                (nodes_[i].visits == nodes_[best].visits && nodes_[i].reward > nodes_[best].reward))
                best = i;
        return best;
    }
    int bestMove() const { return nodes_[bestChild()].move; }
    // #endregion

    const std::vector<Node>& nodes() const { return nodes_; }
    const G& root() const { return root_; }
    Params& params() { return params_; }

    int depthOf(int n) const {
        int d = 0;
        for (; nodes_[n].parent >= 0; n = nodes_[n].parent) ++d;
        return d;
    }

private:
    // #region expand
    void expand(int n, const G& g) {
        int moves[G::kMaxMoves];
        int k = g.listMoves(moves, true);  // 中间优先：没访问过的子节点按这个顺序先试
        int first = int(nodes_.size());
        for (int i = 0; i < k; ++i) {
            Node c;
            c.parent = n, c.move = int8_t(moves[i]);
            G t = g;
            t.play(moves[i]);
            if (t.lastMoverWon()) c.terminal = true, c.terminalReward = 1;
            else if (t.full()) c.terminal = true, c.terminalReward = 0.5f;
            nodes_.push_back(c);
        }
        nodes_[n].first = first, nodes_[n].count = int8_t(k);
    }
    // #endregion

    Params params_;
    G root_{};
    std::vector<Node> nodes_;
};

// 跑 iterations 次迭代，返回选中的着法
template <class G>
int think(const G& root, int iterations, const Params& p, Pcg32& rng, Search<G>* out = nullptr) {
    Search<G> local(p);
    Search<G>& s = out ? *out : local;
    s.params() = p;
    s.reset(root);
    for (int i = 0; i < iterations; ++i) s.iterate(rng);
    return s.bestMove();
}

}  // namespace ga::mcts
