// 客户端预测与和解
//   - 左边是 0 号客户端（你）的画面，右边是服务器。朱红方块是你，墨色方块是其他玩家（机器人）
//   - 不预测：你的角色和别人一样从快照插值，按键之后要等一个往返加插值延迟才动
//   - 预测：按键立刻移动自己；但收到快照时直接跳到服务器给的位置，还在路上的输入被"抹掉"，角色一顿一顿地往回拉
//   - 和解：从快照里自己的位置出发，重算服务器还没处理的输入。没人挡路时和服务器逐位相同；
//     撞到别人时会猜错（你只知道别人过去的位置），这时会跳一下
//   - 平滑：把这一跳分摊到后面几帧
//   - 空心方块是最新快照里你的位置（服务器确认过的）。方向键或 WASD 可以自己控制

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>

#include "demo.h"
#include "predict.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::net;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kLx = 12, kRx = 420, kY0 = 98;

class PredictDemo final : public Demo {
public:
    PredictDemo() { reset(); }
    void reset() override {
        PredictConfig c;
        c.players = players_ ? 8 : 4, c.mode = Mode(mode_), c.link = {latency_, jitter_, loss_ / 100.0};
        s_ = std::make_unique<PredictSession>(c);
        steps_.clear();
    }
    void update() override {
        int dx = (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) - (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A));
        int dy = (IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_S)) - (IsKeyDown(KEY_UP) || IsKeyDown(KEY_W));
        if (dx || dy) lastKey_ = GetTime();
        bool manual = GetTime() - lastKey_ < 3;  // 松开键 3 秒后交还给机器人
        s_->setManual(manual, Input{int8_t(dx), int8_t(dy)});
        manual_ = manual;
    }
    bool step() override {
        s_->tick();
        const auto& h = s_->clients()[0].shownHistory;
        if (h.size() >= 2) steps_.push_back(std::hypot(h.back().first - h[h.size() - 2].first, h.back().second - h[h.size() - 2].second));
        if (steps_.size() > 180) steps_.pop_front();
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const Field& sv = s_->server();
        const auto& c = s_->clients()[0];
        auto walls = [&](float x0) {
            DrawRectangleLinesEx({x0, kY0, float(Field::kW), float(Field::kH)}, hl, pal.gridLine);
            for (auto& w : sv.walls)
                DrawRectangleRec({x0 + float(w.x0.toDouble()), kY0 + float(w.y0.toDouble()), float((w.x1 - w.x0).toDouble()), float((w.y1 - w.y0).toDouble())}, pal.wall);
        };
        auto square = [&](float x0, double x, double y, Color col, bool filled) {
            Rectangle r{x0 + float(x) - Field::kHalf, kY0 + float(y) - Field::kHalf, 2.0f * Field::kHalf, 2.0f * Field::kHalf};
            if (filled) DrawRectangleRec(r, col);
            else DrawRectangleLinesEx(r, 1.5f, col);
        };

        walls(kLx);
        for (size_t k = 1; k < c.shown.size(); ++k) square(kLx, c.shown[k].first, c.shown[k].second, pal.text, true);
        if (c.newest >= 0) {
            const Field::Player& p = c.snaps.at(uint32_t(c.newest)).players[0];
            square(kLx, p.x.toDouble(), p.y.toDouble(), pal.textMuted, false);
        }
        square(kLx, c.shownX, c.shownY, pal.path, true);
        drawText(manual_ ? "client 0: your screen (keyboard)" : "client 0: your screen (arrows / WASD to drive)", {kLx, kY0 + Field::kH + 6}, 10, pal.textMuted);

        walls(kRx);
        for (size_t k = 0; k < sv.players.size(); ++k) square(kRx, sv.players[k].x.toDouble(), sv.players[k].y.toDouble(), k == 0 ? pal.path : pal.text, true);
        drawText("server (authoritative, now)", {kRx, kY0 + Field::kH + 6}, 10, pal.textMuted);

        float y = kY0 + Field::kH + 24;
        char buf[200];
        const auto& cfg = s_->config();
        if (cfg.mode == Mode::None)
            std::snprintf(buf, sizeof buf, "you see yourself in the past: about %.0f ms (up %.0f + down %.0f + interpolation %.0f + waiting for a snapshot)",
                          2 * cfg.link.latencyMs + cfg.interpMs + 25, cfg.link.latencyMs, cfg.link.latencyMs, cfg.interpMs);
        else std::snprintf(buf, sizeof buf, "you see yourself now: inputs move you in the same frame; others are %.0f ms in the past", cfg.link.latencyMs + cfg.interpMs);
        drawText(buf, {kLx, y}, 11, pal.text), y += 17;
        double big = 0;
        for (double v : steps_) big = std::max(big, v);
        std::snprintf(buf, sizeof buf, "corrections %ld (avg %.1f px, max %.1f px)   biggest on-screen jump in the last 3 s: %.1f px", c.corrections,
                      c.correctionSum / std::max(1L, c.corrections), c.correctionMax, big);
        drawText(buf, {kLx, y}, 11, big > 4 ? pal.path : pal.textMuted), y += 17;
        drawText("outline = you in the newest snapshot (what the server has confirmed)", {kLx, y}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        const char* modes[4] = {"no prediction", "predict", "+ reconcile", "+ smooth"};
        const float w[4] = {120, 80, 104, 90};
        float x = 12;
        for (int i = 0; i < 4; ++i) {
            if (ui::button({x, 8, w[i], 26}, modes[i], mode_ == i) && mode_ != i) mode_ = i, ch = true;
            x += w[i] + 4;
        }
        if (ui::toggle({x + 16, 8, 110, 26}, "8 players", &players_)) ch = true;
        ch |= ui::slider({12, 44, 200, 26}, "latency", &latency_, 0, 200, "%.0f ms", 5);
        ch |= ui::slider({220, 44, 200, 26}, "jitter", &jitter_, 0, 100, "%.0f ms", 5);
        ch |= ui::slider({428, 44, 180, 26}, "loss", &loss_, 0, 20, "%.0f%%", 1);
        if (ch) reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "tick %ld", s_->ticks());
        return buf;
    }

private:
    std::unique_ptr<PredictSession> s_;
    std::deque<double> steps_;
    int mode_ = 2;
    bool players_ = false, manual_ = false;
    double lastKey_ = -100;
    float latency_ = 60, jitter_ = 10, loss_ = 0;
};

}  // namespace

int main() {
    static PredictDemo demo;
    ga::DemoConfig config;
    config.title = "Client-side prediction and reconciliation";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
