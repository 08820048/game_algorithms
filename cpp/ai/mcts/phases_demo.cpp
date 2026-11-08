// MCTS 的四个阶段逐步演示：井字棋，剩 4 个空格，轮到 O 走
//   - 右侧是这个局面的完整博弈树：灰点是还没进入搜索树的局面，圆圈是搜索树中的节点，圈里的数字是访问次数，
//     下方的百分比是"走到这个节点的一方"的平均收益。圆圈的颜色表示是谁走到这里的（黑 X，红 O）
//   - phases：每一步只执行一个阶段（选择、扩展、模拟、回传）；fast：每一步执行 25 次完整的迭代
//   - 鼠标悬停在节点上，左侧显示它的局面和 UCB 分数

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../minimax/games.h"
#include "demo.h"
#include "mcts.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::mcts;
using gametree::TicTacToe;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr float kTreeX0 = 170, kTreeX1 = 790;
constexpr float kLevelY0 = 84, kLevelDy = 70;

struct Preset {
    const char* name;
    int x[3], o[2];
};
// 三个局面，都是 X 走了 3 步、O 走了 2 步，轮到 O
const Preset kPresets[] = {
    {"O must block", {0, 4, 5}, {2, 8}},    // X 在 3-4-5 只差 3
    {"O can win now", {4, 8, 6}, {0, 1}},   // O 走 2 立即获胜；X 也有两个获胜点
    {"quiet", {1, 3, 8}, {0, 4}},
};

struct LayoutNode {
    int parent = -1, level = 0;
    std::vector<int> children;
    float x = 0, y = 0;
    TicTacToe g;
};

enum class Phase { Idle, Selected, Expanded, Simulated, Backed };

class PhasesDemo final : public Demo {
public:
    PhasesDemo() { load(0); }

    void reset() override {
        search_.params() = Params{c_, false};
        search_.reset(root_);
        phase_ = Phase::Idle, iterations_ = 0, rollout_.clear();
        rng_ = Pcg32(9);
    }

    bool step() override {
        if (fast_) {
            for (int k = 0; k < 25; ++k) search_.iterate(rng_), ++iterations_;
            phase_ = Phase::Idle;
            return true;
        }
        switch (phase_) {
            case Phase::Idle:
            case Phase::Backed:
                before_ = int(search_.nodes().size());
                leaf_ = search_.select();
                rollout_.clear();
                phase_ = Phase::Selected;
                break;
            case Phase::Selected:
                if (leaf_.expanded >= 0) {
                    phase_ = Phase::Expanded;
                    break;
                }
                [[fallthrough]];
            case Phase::Expanded: simulate(); phase_ = Phase::Simulated; break;
            case Phase::Simulated:
                search_.backpropagate(leaf_.node, reward_);
                ++iterations_;
                phase_ = Phase::Backed;
                break;
        }
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        hover_ = -1;
        if (ui::wantsMouse()) return;
        std::vector<int> map = mapping();
        for (size_t i = 0; i < map.size(); ++i) {
            const LayoutNode& l = layout_[map[i]];
            if (std::hypot(m.x - l.x, m.y - l.y) < 11) hover_ = int(i);
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        std::vector<int> map = mapping();
        const auto& nodes = search_.nodes();
        int visible = int(nodes.size());
        if (phase_ == Phase::Selected) visible = before_;  // 选择阶段：新创建的子节点还不显示

        // 路径：从叶子往上
        std::vector<char> onPath(nodes.size(), 0);
        if (phase_ != Phase::Idle)
            for (int n = (phase_ == Phase::Selected && leaf_.expanded >= 0) ? leaf_.expanded : leaf_.node; n >= 0; n = nodes[n].parent) onPath[n] = 1;

        // 完整博弈树的骨架
        for (size_t i = 1; i < layout_.size(); ++i) {
            const LayoutNode& l = layout_[i];
            const LayoutNode& p = layout_[l.parent];
            DrawLineEx({p.x, p.y}, {l.x, l.y}, hl, Fade(pal.gridLine, 0.7f));
        }
        for (const LayoutNode& l : layout_) DrawCircleV({l.x, l.y}, 2.5f, Fade(pal.textMuted, 0.4f));

        // 搜索树的边
        for (int i = 1; i < visible; ++i) {
            const LayoutNode& l = layout_[map[i]];
            const LayoutNode& p = layout_[map[nodes[i].parent]];
            bool path = onPath[i];
            DrawLineEx({p.x, p.y}, {l.x, l.y}, path ? 3.0f : 1.4f, path ? pal.path : pal.textMuted);
        }
        // 搜索树的节点
        for (int i = 0; i < visible; ++i) {
            const LayoutNode& l = layout_[map[i]];
            const Node& n = nodes[i];
            bool moverIsO = l.level % 2 == 1;  // 根节点轮到 O：第 1 层是 O 走到的
            Color ring = moverIsO ? pal.path : pal.text;
            bool isLeaf = phase_ != Phase::Idle && i == leaf_.node && phase_ != Phase::Selected;
            bool fresh = i >= before_ && phase_ == Phase::Expanded;
            float r = l.level <= 2 ? 11 : 9.5f;
            DrawCircleV({l.x, l.y}, r, isLeaf ? pal.frontier : fresh ? Fade(pal.frontier, 0.35f) : pal.background);
            DrawCircleLinesV({l.x, l.y}, r, ring);
            DrawCircleLinesV({l.x, l.y}, r - 0.8f, ring);
            if (n.terminal) DrawCircleLinesV({l.x, l.y}, r + 3, ring);  // 终局：双圈
            drawTextCentered(std::to_string(n.visits).c_str(), {l.x, l.y}, 10, pal.text);
            if (l.level <= 2 && n.visits > 0 && i > 0) {
                char buf[16];
                std::snprintf(buf, sizeof buf, "%.0f%%", 100 * n.reward / n.visits);
                drawTextCentered(buf, {l.x, l.y + r + 9}, 10, pal.textMuted);
            }
            if (i == hover_) DrawCircleLinesV({l.x, l.y}, r + 5, pal.frontier);
        }

        drawLeftPanel(map);
        drawBottom();
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 13, 72, 26}, "phases", !fast_)) fast_ = false, phase_ = Phase::Idle;
        if (ui::button({88, 13, 52, 26}, "fast", fast_)) {
            if (phase_ == Phase::Selected || phase_ == Phase::Expanded || phase_ == Phase::Simulated) finishIteration();
            fast_ = true;
        }
        const float xs[] = {170, 300, 430};
        const float ws[] = {126, 126, 70};
        for (int k = 0; k < 3; ++k)
            if (ui::button({xs[k], 13, ws[k], 26}, kPresets[k].name, preset_ == k)) load(k), notifyProblemChanged();
        if (ui::slider({530, 13, 258, 26}, "c", &c_, 0, 3, "%.2f", 0.05f)) reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "iteration %d · %zu nodes", iterations_, search_.nodes().size());
        return buf;
    }

private:
    void load(int k) {
        preset_ = k;
        root_ = TicTacToe{};
        for (int i = 0; i < 5; ++i) root_.play(i % 2 == 0 ? kPresets[k].x[i / 2] : kPresets[k].o[i / 2]);
        layout_.clear();
        build(root_, -1, 0);
        // 叶子（没有子节点的局面）从左到右排开，内部节点取子节点的平均位置
        int leaves = 0;
        for (const LayoutNode& l : layout_) leaves += l.children.empty();
        float w = (kTreeX1 - kTreeX0) / leaves;
        int k2 = 0;
        place(0, w, k2);
        reset();
    }
    void build(const TicTacToe& g, int parent, int level) {
        int me = int(layout_.size());
        layout_.push_back({parent, level, {}, 0, kLevelY0 + level * kLevelDy, g});
        if (parent >= 0) layout_[parent].children.push_back(me);
        if (g.lastMoverWon() || g.full()) return;
        int m[9];
        int n = g.listMoves(m, true);  // 与 MCTS 扩展时的顺序相同
        for (int i = 0; i < n; ++i) {
            TicTacToe c = g;
            c.play(m[i]);
            build(c, me, level + 1);
        }
    }
    void place(int n, float w, int& k) {
        LayoutNode& l = layout_[n];
        if (l.children.empty()) {
            l.x = kTreeX0 + (k++ + 0.5f) * w;
            return;
        }
        float sum = 0;
        for (int c : l.children) place(c, w, k), sum += layout_[c].x;
        layout_[n].x = sum / layout_[n].children.size();
    }

    // 搜索树节点 -> 完整博弈树中的位置（子节点顺序相同）
    std::vector<int> mapping() const {
        const auto& nodes = search_.nodes();
        std::vector<int> map(nodes.size(), 0);
        for (size_t i = 1; i < nodes.size(); ++i) {
            const Node& p = nodes[nodes[i].parent];
            map[i] = layout_[map[nodes[i].parent]].children[i - p.first];
        }
        return map;
    }

    void simulate() {
        const Node& n = search_.nodes()[leaf_.node];
        rollout_.clear();
        if (n.terminal) {
            reward_ = n.terminalReward;
            return;
        }
        TicTacToe g = leaf_.state;
        for (int ply = 1;; ++ply) {
            int m = search_.rolloutMove(g, rng_);
            g.play(m);
            rollout_.push_back(m);
            if (g.lastMoverWon()) {
                reward_ = ply % 2 ? 0.0 : 1.0;
                return;
            }
            if (g.full()) {
                reward_ = 0.5;
                return;
            }
        }
    }
    void finishIteration() {
        if (phase_ == Phase::Selected || phase_ == Phase::Expanded) simulate();
        search_.backpropagate(leaf_.node, reward_);
        ++iterations_;
        phase_ = Phase::Idle;
    }

    void drawBoard(const TicTacToe& g, float x, float y, float cell, const std::vector<int>& extra) const {
        const Palette& pal = palette();
        uint16_t xs = g.moves % 2 == 0 ? g.current : uint16_t(g.current ^ g.mask);  // X 是先手
        for (int i = 0; i < 9; ++i) {
            float cx = x + (i % 3 + 0.5f) * cell, cy = y + (i / 3 + 0.5f) * cell;
            DrawRectangleLinesEx({x + (i % 3) * cell, y + (i / 3) * cell, cell, cell}, 1, pal.gridLine);
            if (!(g.mask >> i & 1)) continue;
            bool isX = xs >> i & 1;
            float s = cell * 0.28f;
            if (isX) DrawLineEx({cx - s, cy - s}, {cx + s, cy + s}, 2.5f, pal.text), DrawLineEx({cx - s, cy + s}, {cx + s, cy - s}, 2.5f, pal.text);
            else DrawRing({cx, cy}, s - 1.5f, s + 1, 0, 360, 24, pal.path);
        }
        // 模拟中随机走的棋：淡色，带序号
        bool xToMove = g.moves % 2 == 0;
        for (size_t k = 0; k < extra.size(); ++k) {
            int i = extra[k];
            float cx = x + (i % 3 + 0.5f) * cell, cy = y + (i / 3 + 0.5f) * cell, s = cell * 0.28f;
            bool isX = (k % 2 == 0) == xToMove;
            Color c = Fade(isX ? pal.text : pal.path, 0.45f);
            if (isX) DrawLineEx({cx - s, cy - s}, {cx + s, cy + s}, 2, c), DrawLineEx({cx - s, cy + s}, {cx + s, cy - s}, 2, c);
            else DrawRing({cx, cy}, s - 1.5f, s + 1, 0, 360, 24, c);
            char buf[4];
            std::snprintf(buf, sizeof buf, "%zu", k + 1);
            drawText(buf, {cx + s - 2, cy + s - 6}, 9, pal.text);
        }
    }

    void drawLeftPanel(const std::vector<int>& map) const {
        const Palette& pal = palette();
        const auto& nodes = search_.nodes();
        drawText("position (O to move)", {12, 62}, 11, pal.textMuted);
        drawBoard(root_, 12, 78, 30, {});
        float y = 186;
        char buf[64];
        if (hover_ >= 0) {
            const Node& n = nodes[hover_];
            const LayoutNode& l = layout_[map[hover_]];
            drawText("hovered node", {12, y}, 11, pal.textMuted);
            drawBoard(l.g, 12, y + 16, 30, {});
            y += 116;
            const char* who = l.level % 2 == 1 ? "O" : "X";
            std::snprintf(buf, sizeof buf, "visits %u", n.visits);
            drawText(buf, {12, y}, 11, pal.text);
            if (hover_ > 0 && n.visits > 0) {
                std::snprintf(buf, sizeof buf, "%s's reward %.1f (%.0f%%)", who, n.reward, 100 * n.reward / n.visits);
                drawText(buf, {12, y + 15}, 11, pal.text);
                double ucb = n.reward / n.visits + c_ * std::sqrt(std::log(double(nodes[n.parent].visits)) / n.visits);
                std::snprintf(buf, sizeof buf, "UCB %.3f", ucb);
                drawText(buf, {12, y + 30}, 11, pal.text);
            }
            if (n.terminal) drawText(n.terminalReward == 1 ? "terminal: win" : "terminal: draw", {12, y + 45}, 11, pal.path);
        } else if (phase_ == Phase::Simulated) {
            drawText("simulation (random)", {12, y}, 11, pal.textMuted);
            drawBoard(leaf_.state, 12, y + 16, 30, rollout_);
            y += 116;
            const LayoutNode& l = layout_[map[leaf_.node]];
            const char* who = l.level % 2 == 1 ? "O" : "X";
            std::snprintf(buf, sizeof buf, "reward for %s: %s", who, reward_ == 1 ? "1 (win)" : reward_ == 0 ? "0 (loss)" : "0.5 (draw)");
            drawText(buf, {12, y}, 11, pal.text);
        } else {
            drawText("hover a node to see", {12, y}, 11, pal.textMuted);
            drawText("its position and UCB", {12, y + 15}, 11, pal.textMuted);
        }
    }

    void drawBottom() const {
        const Palette& pal = palette();
        DrawLineEx({12, 402}, {kWidth - 12.0f, 402}, hairline(), pal.gridLine);
        const char* text = "";
        switch (phase_) {
            case Phase::Idle: text = fast_ ? "fast: 25 full iterations per step" : "press step: each step runs one phase of an iteration"; break;
            case Phase::Selected: text = "1 selection: from the root, follow the child with the highest UCB until a new node"; break;
            case Phase::Expanded: text = "2 expansion: the node has been visited before, so create all its children; take the first"; break;
            case Phase::Simulated:
                text = search_.nodes()[leaf_.node].terminal ? "3 simulation: the node is a terminal position, its result is known"
                                                            : "3 simulation: play random moves to the end (left)";
                break;
            case Phase::Backed: text = "4 backpropagation: every node on the path gets +1 visit and the reward, flipped each level"; break;
        }
        drawText(text, {12, 412}, 12, pal.text);
        const auto& nodes = search_.nodes();
        char buf[128];
        if (nodes[0].first >= 0 && nodes[0].visits > 0) {
            int best = search_.bestChild();
            std::snprintf(buf, sizeof buf, "most visited move: cell %d (%u of %u visits)   cells are numbered 0-8 row by row", nodes[best].move,
                          nodes[best].visits, nodes[0].visits);
            drawText(buf, {12, 434}, 11, pal.textMuted);
        }
        drawText("ring colour: who moved into the node (ink X, red O)   number: visits   %: that side's average reward", {12, 452}, 11,
                 pal.textMuted);
    }

    TicTacToe root_;
    std::vector<LayoutNode> layout_;
    Search<TicTacToe> search_;
    Search<TicTacToe>::Leaf leaf_{0, {}};
    Pcg32 rng_{9};
    Phase phase_ = Phase::Idle;
    std::vector<int> rollout_;
    double reward_ = 0;
    int before_ = 0, iterations_ = 0, preset_ = 0, hover_ = -1;
    float c_ = 1.4f;
    bool fast_ = false;
};

}  // namespace

int main() {
    static PhasesDemo demo;
    ga::DemoConfig config;
    config.title = "MCTS phases";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 2.0f;
    return ga::runDemo(demo, config);
}
