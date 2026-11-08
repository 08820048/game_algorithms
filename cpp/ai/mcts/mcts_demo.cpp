// 重力四子棋：和蒙特卡洛树搜索对弈
//   - 点击一列落子。黑色（暗色主题下是白色）是你，红色是 AI
//   - 搜索分摊到每一帧进行，每列上方的柱子是根节点各着法的访问次数，数字是 AI 视角的平均收益（胜率）
//   - 右侧：每步的迭代次数、探索常数 c、启发式模拟（能赢就赢、该堵就堵）
//   - AI 最终选择访问次数最多的着法

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "../minimax/games.h"
#include "demo.h"
#include "mcts.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::mcts;
using gametree::ConnectFour;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr int kCols = 7, kRows = 6;
constexpr float kCell = 50;
constexpr float kBx = 24, kBy = kTop + 92;
constexpr float kPanelX = kBx + kCols * kCell + 34;

class MctsDemo final : public Demo {
public:
    MctsDemo() { newGame(); }

    void reset() override {}
    bool step() override { return true; }

    void update() override {
        Vector2 m = GetMousePosition();
        int c = int((m.x - kBx) / kCell);
        hoverCol_ = (m.x >= kBx && c >= 0 && c < kCols && m.y > kTop + 40 && m.y < kBy + kRows * kCell && !ui::wantsMouse()) ? c : -1;
        if (over_) return;
        bool human = humanTurn();
        if (human && hoverCol_ >= 0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && board_.canPlay(hoverCol_)) {
            play(hoverCol_);
            return;
        }
        // 搜索分摊到每一帧：每帧最多用 12 ms
        bool searching = !human || showHint_;
        if (searching && done_ < budget()) {
            auto t0 = std::chrono::steady_clock::now();
            while (done_ < budget()) {
                for (int k = 0; k < 64 && done_ < budget(); ++k) search_.iterate(rng_), ++done_;
                if (std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > 12) break;
            }
            ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            const auto& nodes = search_.nodes();  // 只检查新增的节点，记下树的最大深度
            for (; scanned_ < nodes.size(); ++scanned_) maxDepth_ = std::max(maxDepth_, search_.depthOf(int(scanned_)));
        }
        if (!human && done_ >= budget() && GetTime() - turnStart_ > 0.5) play(search_.bestMove());
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        bool human = humanTurn();
        bool show = !over_ && (!human || showHint_) && done_ > 0;

        // 根节点各着法：访问次数的柱子 + 平均收益
        if (show) {
            const auto& nodes = search_.nodes();
            const Node& root = nodes[0];
            uint32_t maxV = 1;
            for (int i = root.first; i < root.first + root.count; ++i) maxV = std::max(maxV, nodes[i].visits);
            int best = search_.bestChild();
            for (int i = root.first; i < root.first + root.count; ++i) {
                const Node& n = nodes[i];
                float x = kBx + n.move * kCell, h = 46.0f * n.visits / maxV;
                Color col = i == best ? pal.path : Fade(pal.textMuted, 0.6f);
                DrawRectangleRec({x + 12, kBy - 30 - h, kCell - 24, h}, col);
                char buf[24];
                if (n.terminal && n.terminalReward == 1) std::snprintf(buf, sizeof buf, "win");
                else if (n.visits) std::snprintf(buf, sizeof buf, "%.0f%%", 100 * n.reward / n.visits);
                else std::snprintf(buf, sizeof buf, "-");
                drawTextCentered(buf, {x + kCell / 2, kBy - 16}, 10, i == best ? pal.text : pal.textMuted);
            }
        }

        Rectangle board{kBx, kBy, kCols * kCell, kRows * kCell};
        DrawRectangleLinesEx(board, 1.5f, pal.textMuted);
        if (hoverCol_ >= 0 && human && !over_ && board_.canPlay(hoverCol_))
            DrawRectangleRec({kBx + hoverCol_ * kCell, kBy, kCell, kRows * kCell}, Fade(pal.frontier, 0.18f));
        for (int c = 0; c < kCols; ++c)
            for (int r = 0; r < kRows; ++r) {
                Vector2 p = center(c, r);
                int v = board_.cell(c, r);
                if (v == 0) DrawCircleLinesV(p, kCell * 0.38f, pal.gridLine);
                else DrawCircleV(p, kCell * 0.38f, ((v == 1) == humanFirst_) ? pal.text : pal.path);
            }
        if (lastCol_ >= 0) DrawCircleV(center(lastCol_, board_.height(lastCol_) - 1), 4, pal.background);
        if (winLine_[0] >= 0) DrawLineEx(center(winLine_[0], winLine_[1]), center(winLine_[2], winLine_[3]), 5, pal.frontier);

        const char* msg = over_ ? message_.c_str() : human ? "your move: click a column" : "AI is searching...";
        drawText(msg, {kBx, kHeight - 24.0f}, 13, over_ ? pal.path : pal.text);
        drawPanel();
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 13, 96, 26}, "new game")) newGame();
        if (ui::button({116, 13, 110, 26}, humanFirst_ ? "you move first" : "AI moves first")) humanFirst_ = !humanFirst_, newGame();
        if (ui::button({234, 13, 64, 26}, "undo")) undo();
        float x = kPanelX, w = kWidth - 12 - kPanelX;
        bool changed = false;
        changed |= ui::slider({x, kTop + 24, w, 24}, "log10 it.", &logIters_, 2, 5, "%.1f", 0.5f);
        changed |= ui::slider({x, kTop + 60, w, 24}, "c", &c_, 0, 3, "%.2f", 0.05f);
        changed |= ui::toggle({x, kTop + 96, w, 26}, "smart rollouts (win / block)", &smart_);
        ui::toggle({x, kTop + 130, w, 26}, "search on your turn too", &showHint_);
        if (changed) restartSearch();
    }

    std::string status() const override {
        char buf[128];
        std::snprintf(buf, sizeof buf, "iterations %d · nodes %zu · %.0f ms", done_, search_.nodes().size(), ms_);
        return buf;
    }

private:
    int budget() const { return int(std::lround(std::pow(10.0, logIters_))); }
    bool humanTurn() const { return (board_.moves % 2 == 0) == humanFirst_; }

    void restartSearch() {
        search_.params() = Params{c_, smart_};
        search_.reset(board_);
        done_ = 0, ms_ = 0, scanned_ = 0, maxDepth_ = 0;
    }

    void play(int c) {
        history_.push_back(board_);
        board_.play(c);
        lastCol_ = c, turnStart_ = GetTime();
        checkOver();
        restartSearch();
    }

    void undo() {
        while (!history_.empty()) {
            board_ = history_.back();
            history_.pop_back();
            if (humanTurn()) break;
        }
        if (!humanTurn()) board_ = ConnectFour{}, history_.clear();
        lastCol_ = -1, over_ = false, winLine_[0] = -1, turnStart_ = GetTime();
        restartSearch();
    }

    void newGame() {
        board_ = ConnectFour{};
        history_.clear();
        lastCol_ = -1, over_ = false, winLine_[0] = -1, turnStart_ = GetTime();
        restartSearch();
    }

    void checkOver() {
        if (board_.lastMoverWon()) {
            over_ = true;
            message_ = humanTurn() ? "AI wins. press undo or new game" : "you win! (try more iterations)";
            uint64_t stones = board_.current ^ board_.mask;
            for (uint64_t w : ConnectFour::windows())
                if ((stones & w) == w) {
                    int lo = std::countr_zero(w), hi = 63 - std::countl_zero(w);
                    winLine_[0] = lo / (kRows + 1), winLine_[1] = lo % (kRows + 1);
                    winLine_[2] = hi / (kRows + 1), winLine_[3] = hi % (kRows + 1);
                    break;
                }
        } else if (board_.full()) {
            over_ = true;
            message_ = "draw: the board is full";
        }
    }

    void drawPanel() const {
        const Palette& pal = palette();
        float x = kPanelX, y = kTop + 172;
        DrawLineEx({x, y - 8}, {kWidth - 12.0f, y - 8}, hairline(), pal.gridLine);
        char buf[96];
        std::snprintf(buf, sizeof buf, "budget       %d iterations", budget());
        drawText(buf, {x, y}, 12, pal.text);
        std::snprintf(buf, sizeof buf, "done         %d", done_);
        drawText(buf, {x, y + 18}, 12, pal.text);
        std::snprintf(buf, sizeof buf, "tree nodes   %zu", search_.nodes().size());
        drawText(buf, {x, y + 36}, 12, pal.text);
        std::snprintf(buf, sizeof buf, "deepest      %d plies", maxDepth_);
        drawText(buf, {x, y + 54}, 12, pal.text);
        std::snprintf(buf, sizeof buf, "time         %.0f ms", ms_);
        drawText(buf, {x, y + 72}, 12, pal.text);
        y += 104;
        DrawLineEx({x, y - 8}, {kWidth - 12.0f, y - 8}, hairline(), pal.gridLine);
        const char* help[] = {"bars: visits of each move at the root", "number: average reward for the", "side to move (win 1, draw 0.5)",
                              "the AI plays the most visited move", "c = 0: never explores, only exploits"};
        for (const char* h : help) drawText(h, {x, y}, 11, pal.textMuted), y += 16;
    }

    Vector2 center(int c, int r) const { return {kBx + (c + 0.5f) * kCell, kBy + (kRows - 1 - r + 0.5f) * kCell}; }

    ConnectFour board_;
    std::vector<ConnectFour> history_;
    Search<ConnectFour> search_;
    Pcg32 rng_{2026};
    float logIters_ = 4, c_ = 1.4f;
    bool smart_ = false, showHint_ = false, humanFirst_ = true, over_ = false;
    int done_ = 0, hoverCol_ = -1, lastCol_ = -1, winLine_[4] = {-1, 0, 0, 0};
    double ms_ = 0, turnStart_ = 0;
    size_t scanned_ = 0;
    int maxDepth_ = 0;
    std::string message_;
};

}  // namespace

int main() {
    static MctsDemo demo;
    ga::DemoConfig config;
    config.title = "Four in a row vs MCTS";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
