// 回滚：不等别人的输入，先猜再改
//   - 左边是 0 号机器看到的竞技场。别人的输入还没到时，沿用他最后一个已知输入往前算（猜）
//   - 猜错了就读回那一帧的存档，用真实输入重算到现在：画面上别人会瞬移一下，朱红线段从旧位置指向新位置
//   - 右边是 4 台机器最近 4 秒的时间线，每格一个 tick：柱子高度 = 这个 tick 算了几帧，
//     墨色是正常往前算的一帧，朱红是回滚重算的帧；柱子下面的朱红短线 = 停顿（超出了窗口，只能等）
//   - D + 窗口 要大于单程延迟才不停顿；D 越大，要猜的帧越少，回滚越少，但自己的操作也晚 D 帧生效

#include <algorithm>
#include <array>
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
constexpr float kTop = 80, kX0 = 12, kY0 = 98;
constexpr int kHistory = 240;

class RollbackDemo final : public Demo {
public:
    RollbackDemo() { reset(); }
    void reset() override {
        RollbackConfig c;
        c.units = 160, c.delay = int(delay_), c.window = int(window_), c.link = {latency_, jitter_, loss_ / 100.0};
        s_ = std::make_unique<RollbackSession>(c);
        for (auto& h : hist_) h.clear();
        trails_.clear();
    }
    bool step() override {
        s_->tick();
        for (int i = 0; i < 4; ++i) {
            const auto& p = s_->peers()[size_t(i)];
            bool stalled = p.lastSteps == 0 && int64_t(p.sim.frame) <= s_->ticks() - 1;
            hist_[size_t(i)].push_back({p.lastSteps, p.lastResim, stalled});
            if (hist_[size_t(i)].size() > kHistory) hist_[size_t(i)].pop_front();
        }
        for (auto& t : trails_) --t.life;
        trails_.erase(std::remove_if(trails_.begin(), trails_.end(), [](const Trail& t) { return t.life <= 0; }), trails_.end());
        for (auto& j : s_->peers()[0].lastJumps) trails_.push_back({j, 45});
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const Arena& a = s_->peers()[0].sim;
        DrawRectangleLinesEx({kX0, kY0, float(Arena::kW), float(Arena::kH)}, hl, pal.gridLine);
        for (auto& u : a.units) {
            bool moving = u.vx.raw || u.vy.raw;
            DrawCircleV({kX0 + float(u.x.toDouble()), kY0 + float(u.y.toDouble())}, 2.5f, moving ? pal.text : Fade(pal.textMuted, 0.6f));
        }
        for (auto& t : trails_) {
            Vector2 p0{kX0 + float(t.j.x0), kY0 + float(t.j.y0)}, p1{kX0 + float(t.j.x1), kY0 + float(t.j.y1)};
            Color c = Fade(pal.path, float(t.life) / 45);
            DrawLineEx(p0, p1, 2, c);
            DrawCircleLinesV(p0, 6, c);
        }
        for (size_t i = 0; i < a.players.size(); ++i) {
            Vector2 p{kX0 + float(a.players[i].x.toDouble()), kY0 + float(a.players[i].y.toDouble())};
            DrawCircleV(p, 6, i == 0 ? pal.path : pal.text);
            char b[4];
            std::snprintf(b, sizeof b, "%zu", i);
            drawTextCentered(b, {p.x, p.y - 14}, 10, pal.text);
        }
        drawText("peer 0's view (red lines: rollback corrections)", {kX0, kY0 + Arena::kH + 6}, 10, pal.textMuted);

        // 时间线
        const float tx = 400, tw = 388, rowH = 34;
        drawText("last 4 s: frames computed per tick (red = re-simulated)", {tx, kY0 - 2}, 10, pal.textMuted);
        const float cw = tw / kHistory, unit = 2.6f;
        for (int i = 0; i < 4; ++i) {
            float y = kY0 + 14 + i * (rowH + 8);
            char b[64];
            const auto& p = s_->peers()[size_t(i)];
            std::snprintf(b, sizeof b, "peer %d  frame %u  confirmed %lld", i, p.sim.frame, (long long)RollbackSession::confirmed(p));
            drawText(b, {tx, y}, 10, pal.text);
            float base = y + rowH;
            DrawLineEx({tx, base}, {tx + tw, base}, hl, pal.gridLine);
            const auto& h = hist_[size_t(i)];
            for (size_t k = 0; k < h.size(); ++k) {
                float x = tx + float(k) * cw, w = std::max(cw - 0.3f, 0.6f);
                if (h[k].stalled) DrawRectangleRec({x, base + 1, w, 3}, pal.path);
                float hn = h[k].steps * unit, hr = h[k].resim * unit;
                DrawRectangleRec({x, base - hn, w, hn}, pal.text);
                DrawRectangleRec({x, base - hn - hr, w, hr}, pal.path);
            }
        }
        // 统计
        long ticks = std::max(1L, s_->ticks()), guessed = 0, wrong = 0, rb = 0, resim = 0, maxSteps = 0, stalls = 0;
        double jsum = 0, jmax = 0;
        long jumps = 0;
        for (auto& p : s_->peers()) {
            guessed += p.guessed, wrong += p.wrongGuesses, rb += p.rollbacks, resim += p.resimFrames;
            maxSteps = std::max(maxSteps, p.maxStepsPerTick), stalls = std::max(stalls, p.stallTicks);
            jumps += p.jumps, jsum += p.jumpSum, jmax = std::max(jmax, p.jumpMax);
        }
        float y = kY0 + 14 + 4 * (rowH + 8) + 4;
        char buf[160];
        std::snprintf(buf, sizeof buf, "game speed %.1f%%   stalled ticks %.1f%%   own input delay %d frames", 100.0 * s_->peers()[0].sim.frame / double(ticks),
                      100.0 * stalls / double(ticks), int(delay_));
        drawText(buf, {tx, y}, 11, stalls ? pal.path : pal.text), y += 16;
        std::snprintf(buf, sizeof buf, "guessed inputs wrong %.1f%%   rollbacks %.1f/s per peer", 100.0 * wrong / std::max(1L, guessed), rb / 4.0 / (double(ticks) / 60));
        drawText(buf, {tx, y}, 11, pal.text), y += 16;
        std::snprintf(buf, sizeof buf, "re-simulated %.2f frames/tick (max %ld frames in one tick)", resim / 4.0 / double(ticks), maxSteps);
        drawText(buf, {tx, y}, 11, pal.textMuted), y += 16;
        std::snprintf(buf, sizeof buf, "players jumped %.1f px on average, %.1f px at most", jsum / std::max(1L, jumps), jmax);
        drawText(buf, {tx, y}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::slider({12, 8, 180, 26}, "latency", &latency_, 0, 200, "%.0f ms", 5);
        ch |= ui::slider({200, 8, 180, 26}, "jitter", &jitter_, 0, 100, "%.0f ms", 5);
        ch |= ui::slider({388, 8, 160, 26}, "loss", &loss_, 0, 20, "%.0f%%", 1);
        ch |= ui::slider({12, 44, 180, 26}, "D", &delay_, 0, 10, "%.0f", 1);
        ch |= ui::slider({200, 44, 180, 26}, "window", &window_, 0, 15, "%.0f", 1);
        if (ch) reset(), notifyProblemChanged();
        if (ui::button({388, 44, 80, 26}, "restart")) reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "frame %u · tick %ld", s_->peers()[0].sim.frame, s_->ticks());
        return buf;
    }

private:
    struct Cell {
        int steps, resim;
        bool stalled;
    };
    struct Trail {
        RollbackPeer::Jump j;
        int life;
    };
    std::unique_ptr<RollbackSession> s_;
    std::array<std::deque<Cell>, 4> hist_;
    std::vector<Trail> trails_;
    float latency_ = 60, jitter_ = 20, loss_ = 2, delay_ = 0, window_ = 8;
};

}  // namespace

int main() {
    static RollbackDemo demo;
    ga::DemoConfig config;
    config.title = "Rollback";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
