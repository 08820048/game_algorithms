// 重力四子棋（四子连线）：和 Alpha-Beta AI 对弈
//   - 点击一列落子。黑色（暗色主题下是白色）是你，红色是 AI
//   - 右侧：搜索深度；prune / order / table / deepen 分别开关剪枝、中间优先排序、置换表、迭代加深
//   - 每列上方是搜索给出的分数（站在轮到走的一方）。"<=" 表示被剪枝、只知道上界；win n / loss n 表示 n 步之内分出胜负
//   - 每次搜索都清空置换表，节点数只取决于局面和开关，方便对比

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstdio>
#include <string>

#include "demo.h"
#include "games.h"
#include "minimax.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::gametree;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr int kCols = 7, kRows = 6;
constexpr float kCell = 52;
constexpr float kBx = 24, kBy = kTop + 50;
constexpr float kPanelX = kBx + kCols * kCell + 32;

class Connect4Demo final : public Demo {
public:
    Connect4Demo() : table_(18) { newGame(); }

    void reset() override {}
    bool step() override { return true; }

    void update() override {
        Vector2 m = GetMousePosition();
        int c = int((m.x - kBx) / kCell);
        hoverCol_ = (m.x >= kBx && c >= 0 && c < kCols && m.y > kTop && m.y < kBy + kRows * kCell && !ui::wantsMouse()) ? c : -1;
        if (over_) return;
        bool humanTurn = (board_.moves % 2 == 0) == humanFirst_;
        if (humanTurn && hoverCol_ >= 0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && board_.canPlay(hoverCol_)) {
            play(hoverCol_);
            return;
        }
        // 落子后先画一帧再搜索（搜索会占住这一帧）；AI 再稍等片刻才落子，让玩家看清局面
        double since = GetTime() - turnStart_;
        if (!analysisValid_ && (showHint_ || !humanTurn) && since > 0.05) analyze();
        if (!humanTurn && analysisValid_ && since > 0.45) play(result_.best);
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        bool humanTurn = humanTurnNow();

        // 每列上方的分数
        if (analysisValid_ && !over_ && (showHint_ || !humanTurn))
            for (size_t i = 0; i < result_.moves.size(); ++i) {
                int c = result_.moves[i];
                bool best = c == result_.best;
                Vector2 p{kBx + (c + 0.5f) * kCell, kBy - 20};
                std::string s = scoreText(result_.scores[i], result_.bounds[i]);
                drawTextCentered(s.c_str(), p, 11, best ? pal.text : pal.textMuted);
                if (best) DrawLineEx({p.x - 16, p.y + 9}, {p.x + 16, p.y + 9}, 2, pal.path);
            }

        // 棋盘
        Rectangle board{kBx, kBy, kCols * kCell, kRows * kCell};
        DrawRectangleLinesEx(board, 1.5f, pal.textMuted);
        if (hoverCol_ >= 0 && humanTurn && !over_ && board_.canPlay(hoverCol_))
            DrawRectangleRec({kBx + hoverCol_ * kCell, kBy, kCell, kRows * kCell}, Fade(pal.frontier, 0.18f));
        for (int c = 0; c < kCols; ++c)
            for (int r = 0; r < kRows; ++r) {
                Vector2 p = center(c, r);
                int v = board_.cell(c, r);
                if (v == 0) {
                    DrawCircleLinesV(p, kCell * 0.38f, pal.gridLine);
                    continue;
                }
                bool human = (v == 1) == humanFirst_;
                DrawCircleV(p, kCell * 0.38f, human ? pal.text : pal.path);
            }
        // 最后一手
        if (lastCol_ >= 0) {
            Vector2 p = center(lastCol_, board_.height(lastCol_) - 1);
            DrawCircleV(p, 4, pal.background);
        }
        if (winLine_[0] >= 0) DrawLineEx(center(winLine_[0], winLine_[1]), center(winLine_[2], winLine_[3]), 5, pal.frontier);
        for (int c = 0; c < kCols; ++c) {
            char n[4];
            std::snprintf(n, sizeof n, "%d", c + 1);
            drawTextCentered(n, {kBx + (c + 0.5f) * kCell, kBy + kRows * kCell + 12}, 10, pal.textMuted);
        }

        // 提示行
        const char* msg = over_ ? message_.c_str()
                        : humanTurn ? "your move: click a column"
                                    : "AI is thinking...";
        drawText(msg, {kBx, kHeight - 26.0f}, 13, over_ ? pal.path : pal.text);

        drawPanel();
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 13, 96, 26}, "new game")) newGame();
        if (ui::button({116, 13, 110, 26}, humanFirst_ ? "you move first" : "AI moves first")) humanFirst_ = !humanFirst_, newGame();
        if (ui::button({234, 13, 64, 26}, "undo")) undo();
        bool changed = false;
        float x = kPanelX, w = kWidth - 12 - kPanelX;
        changed |= ui::slider({x, kBy - 2, w, 24}, "depth", &depth_, 1, 12, "%.0f", 1);
        float bw = (w - 12) / 4;
        changed |= ui::toggle({x, kBy + 40, bw, 26}, "prune", &prune_);
        changed |= ui::toggle({x + (bw + 4), kBy + 40, bw, 26}, "order", &order_);
        changed |= ui::toggle({x + 2 * (bw + 4), kBy + 40, bw, 26}, "table", &table_on_);
        changed |= ui::toggle({x + 3 * (bw + 4), kBy + 40, bw, 26}, "deepen", &deepen_);
        ui::toggle({x, kBy + 74, w, 26}, "show scores on your turn", &showHint_);
        if (changed) analysisValid_ = false;
    }

    std::string status() const override {
        if (!analysisValid_) return {};
        char buf[128];
        std::snprintf(buf, sizeof buf, "depth %d · nodes %llu · %.1f ms", result_.depth, (unsigned long long)stats_.nodes, ms_);
        return buf;
    }

private:
    bool humanTurnNow() const { return (board_.moves % 2 == 0) == humanFirst_; }

    // 不剪枝时节点数约为 7^depth，没有排序时剪枝效果也差很多：限制深度，免得网页卡住
    int effectiveDepth() const {
        int d = int(depth_);
        if (!prune_) d = std::min(d, 7);
        else if (!order_ && !table_on_) d = std::min(d, 10);
        return d;
    }

    void analyze() {
        Options o;
        o.prune = prune_, o.order = order_, o.table = table_on_ ? &table_ : nullptr;
        table_.clear();
        stats_ = {};
        auto t0 = std::chrono::steady_clock::now();
        result_ = deepen_ ? iterativeDeepening(board_, effectiveDepth(), o, stats_) : searchRoot(board_, effectiveDepth(), o, stats_);
        ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        analysisValid_ = true;
    }

    void play(int c) {
        history_.push_back(board_);
        board_.play(c);
        lastCol_ = c;
        analysisValid_ = false;
        turnStart_ = GetTime();
        checkOver();
    }

    void undo() {  // 退回到上一次轮到你走的局面
        while (!history_.empty()) {
            board_ = history_.back();
            history_.pop_back();
            if (humanTurnNow()) break;
        }
        if (!humanTurnNow()) board_ = ConnectFour{}, history_.clear();  // AI 先走时退到开局，AI 会重新走第一步
        lastCol_ = -1, over_ = false, winLine_[0] = -1, analysisValid_ = false, turnStart_ = GetTime();
    }

    void newGame() {
        board_ = ConnectFour{};
        history_.clear();
        lastCol_ = -1, over_ = false, winLine_[0] = -1, analysisValid_ = false, turnStart_ = GetTime();
    }

    void checkOver() {
        if (board_.lastMoverWon()) {
            over_ = true;
            bool humanWon = !humanTurnNow();
            message_ = humanWon ? "you win! (try a deeper AI)" : "AI wins. press undo or new game";
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

    // 胜负分换算成"几步之内"：分数 = kWin + 获胜时剩下的空格数
    std::string scoreText(int s, Bound b) const {
        char buf[24];
        const char* pre = b == Bound::Upper ? "<=" : "";
        if (s >= kWin) std::snprintf(buf, sizeof buf, "%swin %d", pre, board_.empties() - (s - kWin));
        else if (s <= -kWin) std::snprintf(buf, sizeof buf, "%sloss %d", pre, board_.empties() - (-s - kWin));
        else std::snprintf(buf, sizeof buf, "%s%+d", pre, s);
        return buf;
    }

    void drawPanel() const {
        const Palette& pal = palette();
        float x = kPanelX, y = kBy + 116;
        DrawLineEx({x, y - 8}, {kWidth - 12.0f, y - 8}, hairline(), pal.gridLine);
        char buf[96];
        if (analysisValid_) {
            const char* side = humanTurnNow() ? "your position" : "AI's position";
            std::snprintf(buf, sizeof buf, "last search (%s)", side);
            drawText(buf, {x, y}, 11, pal.textMuted);
            std::snprintf(buf, sizeof buf, "depth   %d%s", result_.depth, result_.depth < effectiveDepth() ? "  (stopped: result is forced)" : "");
            drawText(buf, {x, y + 20}, 12, pal.text);
            std::snprintf(buf, sizeof buf, "nodes   %llu", (unsigned long long)stats_.nodes);
            drawText(buf, {x, y + 38}, 12, pal.text);
            std::snprintf(buf, sizeof buf, "cutoffs %llu   table hits %llu", (unsigned long long)stats_.cutoffs, (unsigned long long)stats_.ttHits);
            drawText(buf, {x, y + 56}, 12, pal.text);
            std::snprintf(buf, sizeof buf, "time    %.1f ms", ms_);
            drawText(buf, {x, y + 74}, 12, pal.text);
        } else {
            drawText("scores appear when the AI", {x, y}, 11, pal.textMuted);
            drawText("searches (or turn them on above)", {x, y + 16}, 11, pal.textMuted);
        }
        if (effectiveDepth() < int(depth_)) {
            std::snprintf(buf, sizeof buf, "depth capped at %d without %s", effectiveDepth(), prune_ ? "order / table" : "pruning");
            drawText(buf, {x, y + 96}, 11, pal.path);
        }
        y += 124;
        DrawLineEx({x, y - 8}, {kWidth - 12.0f, y - 8}, hairline(), pal.gridLine);
        const char* help[] = {"scores: for the side to move", "+n  heuristic (threat count)", "win n / loss n: forced within n plies",
                              "<=n  pruned: at most n", "toggle switches to compare nodes"};
        for (const char* h : help) drawText(h, {x, y}, 11, pal.textMuted), y += 16;
    }

    Vector2 center(int c, int r) const { return {kBx + (c + 0.5f) * kCell, kBy + (kRows - 1 - r + 0.5f) * kCell}; }

    ConnectFour board_;
    std::vector<ConnectFour> history_;
    Table table_;
    RootResult result_;
    Stats stats_;
    double ms_ = 0, turnStart_ = 0;
    float depth_ = 8;
    bool prune_ = true, order_ = true, table_on_ = true, deepen_ = true, showHint_ = false;
    bool humanFirst_ = true, over_ = false, analysisValid_ = false;
    int hoverCol_ = -1, lastCol_ = -1, winLine_[4] = {-1, 0, 0, 0};
    std::string message_;
};

}  // namespace

int main() {
    static Connect4Demo demo;
    ga::DemoConfig config;
    config.title = "Four in a row vs alpha-beta";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
