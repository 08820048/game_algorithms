#pragma once

// 博弈树搜索：Minimax、Negamax、Alpha-Beta 剪枝、着法排序、置换表、迭代加深。
// 模板参数 G 是一个双人轮流、零和、完全信息的游戏，接口见 games.h。
//
// 分数约定：
//   - 胜负：kWin + 获胜时棋盘上剩余的空格数。赢得越早，剩下的空格越多，分数越高；
//     这个值只取决于局面本身（与从哪里开始搜索无关），所以可以放心地存进置换表
//   - 和棋 0；搜索深度用完时用 G::evaluate() 估值，它的绝对值远小于 kWin

#include <algorithm>
#include <cstdint>
#include <vector>

namespace ga::gametree {

constexpr int kWin = 10000;
constexpr int kInf = 1 << 20;

inline bool isWinScore(int s) { return s >= kWin || s <= -kWin; }

struct Stats {
    uint64_t nodes = 0;    // 访问的局面数（每调用一次搜索函数算一个）
    uint64_t cutoffs = 0;  // 发生 beta 截断的次数
    uint64_t ttHits = 0;   // 置换表直接给出结果（或缩小窗口后截断）的次数
};

// #region terminal
// 终局判断：刚走完的一方连成了线，说明轮到走的一方已经输了
template <class G>
bool terminal(const G& g, int& score) {
    if (g.lastMoverWon()) return score = -(kWin + g.empties()), true;
    if (g.full()) return score = 0, true;
    return false;
}
// #endregion

// #region minimax
// 经典写法：分数始终站在先手（MAX）一方。轮到 MAX 时取子节点的最大值，轮到 MIN 时取最小值
template <class G>
int minimax(const G& g, int depth, bool maxToMove, Stats& st) {
    ++st.nodes;
    int s;
    bool over = terminal(g, s);
    if (over || depth == 0) {
        if (!over) s = g.evaluate();  // terminal 和 evaluate 都站在轮到走的一方
        return maxToMove ? s : -s;
    }
    int moves[G::kMaxMoves];
    int n = g.listMoves(moves, false);
    int best = maxToMove ? -kInf : kInf;
    for (int i = 0; i < n; ++i) {
        G child = g;
        child.play(moves[i]);
        int v = minimax(child, depth - 1, !maxToMove, st);
        best = maxToMove ? std::max(best, v) : std::min(best, v);
    }
    return best;
}
// #endregion

// #region negamax
// Negamax：零和博弈中"对方的最好 = 我的最坏"，max(a, b) = -min(-a, -b)。
// 分数始终站在轮到走的一方，子节点的分数取负号，MAX 和 MIN 合并成同一段代码
template <class G>
int negamax(const G& g, int depth, Stats& st) {
    ++st.nodes;
    int s;
    if (terminal(g, s)) return s;
    if (depth == 0) return g.evaluate();
    int moves[G::kMaxMoves];
    int n = g.listMoves(moves, false);
    int best = -kInf;
    for (int i = 0; i < n; ++i) {
        G child = g;
        child.play(moves[i]);
        best = std::max(best, -negamax(child, depth - 1, st));
    }
    return best;
}
// #endregion

// #region table
// 置换表：同一个局面可以由不同的着法顺序到达（先走 A 再走 B，或先走 B 再走 A）。
// 把搜过的局面记下来，再次遇到时直接使用。
// 因为有剪枝，存下的值不一定是精确值，要同时记下它是哪种界
enum class Bound : uint8_t { Exact, Lower, Upper };

struct Entry {
    uint64_t key = ~0ull;  // 完整的键：不会把两个不同的局面当成同一个
    int32_t value = 0;
    int8_t depth = -1;     // 这个值是搜索多少层得到的
    Bound bound = Bound::Exact;
    int8_t best = -1;      // 最好的着法，下次搜索时先试它
};

class Table {
public:
    explicit Table(int bits = 20) : entries_(size_t(1) << bits), shift_(64 - bits) {}
    Entry* find(uint64_t key) {
        Entry& e = slot(key);
        return e.key == key ? &e : nullptr;
    }
    void store(uint64_t key, int value, int depth, Bound bound, int best) {
        slot(key) = {key, value, int8_t(depth), bound, int8_t(best)};  // 总是覆盖旧的
    }
    void clear() { std::fill(entries_.begin(), entries_.end(), Entry{}); }

private:
    Entry& slot(uint64_t key) { return entries_[(key * 0x9E3779B97F4A7C15ull) >> shift_]; }
    std::vector<Entry> entries_;
    int shift_;
};
// #endregion

struct Options {
    bool prune = true;       // false：不剪枝，等价于 negamax（窗口参数被忽略）
    bool order = true;       // 着法按"中间优先"的静态顺序；关闭时从左到右
    Table* table = nullptr;  // 置换表；同时提供上一轮搜索的最好着法，排在最前面
};

// #region alphabeta
// Alpha-Beta：在 (alpha, beta) 窗口内搜索。
//   alpha：我已经有把握拿到的分数（更差的着法不用细看）
//   beta：对方已经有办法把我限制在这个分数以下（我找到 >= beta 的着法，对方根本不会让局面走到这里）
// 返回值 v（fail-soft：允许落在窗口外）的含义：
//   alpha < v < beta  精确值
//   v <= alpha        真实值 <= v（上界）
//   v >= beta         真实值 >= v（下界）
template <class G>
int alphabeta(const G& g, int depth, int alpha, int beta, const Options& o, Stats& st) {
    ++st.nodes;
    int s;
    if (terminal(g, s)) return s;
    if (depth == 0) return g.evaluate();

    int ttMove = -1;
    if (o.table)
        if (const Entry* e = o.table->find(g.key())) {
            ttMove = e->best;
            if (e->depth >= depth) {
                if (e->bound == Bound::Exact) return ++st.ttHits, e->value;
                if (e->bound == Bound::Lower) alpha = std::max(alpha, e->value);
                else beta = std::min(beta, e->value);
                if (alpha >= beta) return ++st.ttHits, e->value;
            }
        }
    const int alpha0 = alpha;  // 判断结果是哪种界，要用置换表收窄之后的窗口

    int moves[G::kMaxMoves];
    int n = g.listMoves(moves, o.order);
    if (ttMove >= 0) std::rotate(moves, std::find(moves, moves + n, ttMove), std::find(moves, moves + n, ttMove) + 1);

    int best = -kInf, bestMove = moves[0];
    for (int i = 0; i < n; ++i) {
        G child = g;
        child.play(moves[i]);
        int v = o.prune ? -alphabeta(child, depth - 1, -beta, -alpha, o, st) : -alphabeta(child, depth - 1, -kInf, kInf, o, st);
        if (v > best) best = v, bestMove = moves[i];
        if (best > alpha) alpha = best;
        if (o.prune && alpha >= beta) {
            ++st.cutoffs;  // 对方不会让局面走到这里，剩下的着法不用看了
            break;
        }
    }
    if (o.table) {
        Bound b = best <= alpha0 ? Bound::Upper : best >= beta ? Bound::Lower : Bound::Exact;
        o.table->store(g.key(), best, depth, b, bestMove);
    }
    return best;
}
// #endregion

// #region root
// 根节点：逐个着法搜索，记下每个着法的分数。
// 有剪枝时，不如当前最好着法的分数只是上界（"至多这么多"），bound[i] 记为 Upper
struct RootResult {
    int score = -kInf, best = -1, depth = 0;
    std::vector<int> moves, scores;
    std::vector<Bound> bounds;
};

template <class G>
RootResult searchRoot(const G& g, int depth, const Options& o, Stats& st, int firstMove = -1) {
    RootResult r;
    r.depth = depth;
    int moves[G::kMaxMoves];
    int n = g.listMoves(moves, o.order);
    if (firstMove >= 0 && std::find(moves, moves + n, firstMove) != moves + n)
        std::rotate(moves, std::find(moves, moves + n, firstMove), std::find(moves, moves + n, firstMove) + 1);
    ++st.nodes;
    int alpha = -kInf;
    for (int i = 0; i < n; ++i) {
        G child = g;
        child.play(moves[i]);
        int v = o.prune ? -alphabeta(child, depth - 1, -kInf, -alpha, o, st) : -alphabeta(child, depth - 1, -kInf, kInf, o, st);
        r.moves.push_back(moves[i]);
        r.scores.push_back(v);
        r.bounds.push_back(o.prune && v <= alpha ? Bound::Upper : Bound::Exact);
        if (v > r.score) r.score = v, r.best = moves[i];
        alpha = std::max(alpha, v);
    }
    return r;
}
// #endregion

// #region deepening
// 迭代加深：依次搜索 1、2、3……层。浅层搜索很便宜，它找到的最好着法
// 和存进置换表的最好着法，让下一层的着法排序好得多，往往总代价反而更低。
// 一旦发现必胜或必败（分数是胜负分），更深的搜索不会改变结论，提前停止
template <class G>
RootResult iterativeDeepening(const G& g, int maxDepth, const Options& o, Stats& st) {
    RootResult r;
    for (int d = 1; d <= maxDepth; ++d) {
        r = searchRoot(g, d, o, st, r.best);
        if (isWinScore(r.score)) break;
    }
    return r;
}
// #endregion

}  // namespace ga::gametree
