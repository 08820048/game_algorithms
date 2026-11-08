#pragma once

// 两个用来演示博弈树搜索的小游戏，接口相同，供 minimax.h 中的搜索模板使用：
//   - TicTacToe：井字棋，小到可以完整搜索整棵博弈树
//   - Connect<W, H>：W 列 H 行的重力四子棋，棋子落到所在列的最低空位，横、竖、斜连成 4 子获胜
//
// 共同的接口（搜索只用到这些）：
//   kMaxMoves               一个局面最多有几个着法
//   play(m)                 当前一方走 m，然后轮到对方（棋盘很小，搜索时直接复制局面，不需要悔棋）
//   lastMoverWon()          刚走完的一方是否已经获胜
//   full() / empties()      棋盘是否下满 / 还剩几个空位
//   listMoves(out, ordered) 列出合法着法；ordered 为 true 时按"中间优先"的顺序
//   evaluate()              非终局的估值，站在轮到走的一方
//   key()                   局面的唯一编号，用作置换表的键

#include <array>
#include <bit>
#include <cstdint>
#include <string_view>
#include <vector>

namespace ga::gametree {

// #region tictactoe
// 井字棋：格子编号 0..8，从左上角按行编号。
// current 是轮到走的一方的棋子，mask 是所有棋子；刚走完的一方的棋子 = current ^ mask
struct TicTacToe {
    static constexpr int kMaxMoves = 9;
    uint16_t current = 0, mask = 0;
    int moves = 0;

    void play(int cell) {
        current ^= mask;  // 换边：对方的棋子成为"轮到走的一方"
        mask |= uint16_t(1u << cell);
        ++moves;
    }
    bool lastMoverWon() const {
        static constexpr uint16_t lines[] = {0007, 0070, 0700, 0111, 0222, 0444, 0421, 0124};
        uint16_t p = current ^ mask;
        for (uint16_t l : lines)
            if ((p & l) == l) return true;
        return false;
    }
    bool full() const { return moves == 9; }
    int empties() const { return 9 - moves; }
    int listMoves(int* out, bool ordered) const {
        static constexpr int centerFirst[] = {4, 0, 2, 6, 8, 1, 3, 5, 7};
        int n = 0;
        for (int k = 0; k < 9; ++k) {
            int c = ordered ? centerFirst[k] : k;
            if (!(mask >> c & 1)) out[n++] = c;
        }
        return n;
    }
    int evaluate() const { return 0; }  // 井字棋总是搜到终局，用不到估值
    uint64_t key() const { return current | uint64_t(mask) << 9; }
};
// #endregion

// #region connect-board
// 重力四子棋的位棋盘（bitboard）：每列占 H+1 位，从下往上，最上面多出的一位始终为空，
// 用来隔开相邻两列，这样移位判断连线时不会从一列的顶端"绕"到下一列的底部。
//
//   列 0 占第 0..H 位，列 1 占第 H+1..2H+1 位，……
//   mask + bottom(c)：列 c 的最低空位正好是"该列已有棋子 + 1"进位的位置
template <int W, int H>
struct Connect {
    static_assert(W * (H + 1) <= 64, "board must fit in 64 bits");
    static constexpr int kWidth = W, kHeight = H, kMaxMoves = W;

    uint64_t current = 0, mask = 0;  // 轮到走的一方的棋子 / 所有棋子
    int moves = 0;

    static constexpr uint64_t bottom(int c) { return 1ull << (c * (H + 1)); }
    static constexpr uint64_t top(int c) { return 1ull << (c * (H + 1) + H - 1); }

    bool canPlay(int c) const { return !(mask & top(c)); }
    void play(int c) {
        current ^= mask;
        mask |= mask + bottom(c);
        ++moves;
    }
    bool lastMoverWon() const { return aligned(current ^ mask); }
    bool full() const { return moves == W * H; }
    int empties() const { return W * H - moves; }
    uint64_t key() const { return current + mask; }  // 在 mask 的每列顶上多加一位，与局面一一对应

    // 四个方向各做两次"与移位"：p & p>>s 标出连续 2 子，再与自身移位 2s 标出连续 4 子
    static bool aligned(uint64_t p) {
        for (int s : {1, H, H + 1, H + 2}) {  // 竖、斜（左上到右下）、横、斜（左下到右上）
            uint64_t m = p & (p >> s);
            if (m & (m >> (2 * s))) return true;
        }
        return false;
    }
    // #endregion

    // 中间优先的着法顺序：中间一列能参与的连线最多。W = 7 时为 3 2 4 1 5 0 6
    static constexpr int centerOrder(int k) { return W / 2 + (k % 2 ? -(k + 1) / 2 : k / 2); }

    int listMoves(int* out, bool ordered) const {
        int n = 0;
        for (int k = 0; k < W; ++k) {
            int c = ordered ? centerOrder(k) : k;
            if (canPlay(c)) out[n++] = c;
        }
        return n;
    }

    // 0 空，1 先手的棋子，2 后手的棋子
    int cell(int c, int r) const {
        uint64_t b = 1ull << (c * (H + 1) + r);
        if (!(mask & b)) return 0;
        uint64_t first = moves % 2 == 0 ? current : current ^ mask;
        return first & b ? 1 : 2;
    }
    int height(int c) const { return std::popcount(mask & (((1ull << H) - 1) << (c * (H + 1)))); }

    // 按列号（1 起）的字符串摆出局面，例如 "4453"。遇到非法着法或已分胜负时返回 false
    bool playSequence(std::string_view seq) {
        for (char ch : seq) {
            int c = ch - '1';
            if (c < 0 || c >= W || !canPlay(c) || lastMoverWon()) return false;
            play(c);
        }
        return true;
    }

    // #region connect-eval
    // 所有长度为 4 的"窗口"（横、竖、两条斜线），预先算成位掩码
    static const std::vector<uint64_t>& windows() {
        static const std::vector<uint64_t> w = [] {
            std::vector<uint64_t> out;
            const int dirs[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
            for (auto& d : dirs)
                for (int c = 0; c < W; ++c)
                    for (int r = 0; r < H; ++r) {
                        int ec = c + 3 * d[0], er = r + 3 * d[1];
                        if (ec < 0 || ec >= W || er < 0 || er >= H) continue;
                        uint64_t m = 0;
                        for (int k = 0; k < 4; ++k) m |= 1ull << ((c + k * d[0]) * (H + 1) + r + k * d[1]);
                        out.push_back(m);
                    }
            return out;
        }();
        return w;
    }

    // 估值：一个窗口里只有我的棋子（对方还能堵死前），按子数给分；只有对方的棋子则扣分。
    // 两边都有棋子的窗口已经不可能连成 4 子，不计分
    int evaluate() const {
        static constexpr int weight[5] = {0, 1, 4, 16, 64};  // 4 子只出现在终局，搜索不会走到这里
        uint64_t me = current, op = current ^ mask;
        int score = 0;
        for (uint64_t w : windows()) {
            int a = std::popcount(me & w), b = std::popcount(op & w);
            if (b == 0) score += weight[a];
            else if (a == 0) score -= weight[b];
        }
        return score;
    }
    // #endregion
};

using ConnectFour = Connect<7, 6>;

}  // namespace ga::gametree
