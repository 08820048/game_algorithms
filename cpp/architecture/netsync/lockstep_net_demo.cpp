// 帧同步：4 台机器只交换输入
//   - 左边是 0 号机器看到的竞技场：4 个玩家（机器人）推开周围的单位。所有机器算出的画面逐位相同
//   - 右边是 4 台机器最近 4 秒的时间线，每格一个 tick：墨色 = 算了一帧，朱红 = 停顿（还缺某个玩家这一帧的输入），
//     灰色加深 = 一个 tick 里算了好几帧（追赶）
//   - 调高延迟和抖动，或把输入延迟 D 调到比单程延迟小，就开始停顿；停顿多了整个游戏变慢
//   - resend 关掉：每个输入只发一次，丢一个包所有机器永远卡住
//   - corrupt：让 2 号机器的一个单位差一个最小单位，下一个哈希检查点（每秒一次）就会发现

#include <algorithm>
#include <array>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>

#include "demo.h"
#include "net.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::net;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kX0 = 12, kY0 = 98;
constexpr int kHistory = 240;

class LockstepNetDemo final : public Demo {
public:
    LockstepNetDemo() { reset(); }
    void reset() override {
        LockstepConfig c;
        c.units = 160, c.delay = int(delay_), c.resend = resend_, c.link = {latency_, jitter_, loss_ / 100.0};
        s_ = std::make_unique<LockstepSession>(c);
        for (auto& h : hist_) h.clear();
        prevFrame_.fill(0);
    }
    bool step() override {
        s_->tick();
        for (int i = 0; i < 4; ++i) {
            uint32_t f = s_->peers()[size_t(i)].sim.frame;
            hist_[size_t(i)].push_back(int(f - prevFrame_[size_t(i)]));
            prevFrame_[size_t(i)] = f;
            if (hist_[size_t(i)].size() > kHistory) hist_[size_t(i)].pop_front();
        }
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
        for (size_t i = 0; i < a.players.size(); ++i) {
            Vector2 p{kX0 + float(a.players[i].x.toDouble()), kY0 + float(a.players[i].y.toDouble())};
            DrawCircleLinesV(p, 36, Fade(pal.path, 0.35f));
            DrawCircleV(p, 6, pal.path);
            char b[4];
            std::snprintf(b, sizeof b, "%zu", i);
            drawTextCentered(b, {p.x, p.y - 14}, 10, pal.text);
        }
        drawText("peer 0's view", {kX0, kY0 + Arena::kH + 6}, 10, pal.textMuted);

        // 时间线
        const float tx = 400, tw = 388, rowH = 22;
        drawText("last 4 s, one cell per tick", {tx, kY0 - 2}, 10, pal.textMuted);
        const float cw = tw / kHistory;
        for (int i = 0; i < 4; ++i) {
            float y = kY0 + 16 + i * (rowH + 8);
            char b[48];
            const auto& p = s_->peers()[size_t(i)];
            std::snprintf(b, sizeof b, "peer %d  frame %u", i, p.sim.frame);
            drawText(b, {tx, y}, 10, pal.text);
            const auto& h = hist_[size_t(i)];
            for (size_t k = 0; k < h.size(); ++k) {
                Color c = h[k] == 0 ? pal.path : h[k] == 1 ? pal.text : pal.textMuted;
                DrawRectangleRec({tx + float(k) * cw, y + 13, std::max(cw - 0.3f, 0.6f), 8}, c);
            }
        }
        // 统计
        const auto& p0 = s_->peers()[0];
        long ticks = std::max(1L, s_->ticks());
        float y = kY0 + 16 + 4 * (rowH + 8) + 6;
        char buf[160];
        std::snprintf(buf, sizeof buf, "game speed %.1f%%   stalled ticks %.1f%%", 100.0 * p0.sim.frame / double(ticks), 100.0 * p0.stallTicks / double(ticks));
        drawText(buf, {tx, y}, 11, p0.stallTicks ? pal.path : pal.text), y += 17;
        std::snprintf(buf, sizeof buf, "input delay D = %d frames = %.0f ms", int(delay_), delay_ * 1000 / 60);
        drawText(buf, {tx, y}, 11, pal.text), y += 17;
        std::snprintf(buf, sizeof buf, "worst one-way %.0f ms = %.1f ticks", latency_ + jitter_, (latency_ + jitter_) * 60 / 1000);
        drawText(buf, {tx, y}, 11, pal.textMuted), y += 17;
        std::snprintf(buf, sizeof buf, "upload per peer %.1f KB/s (inputs + acks + hash)", double(s_->bytesSentBy(0)) / 1024 / (double(ticks) / 60));
        drawText(buf, {tx, y}, 11, pal.textMuted), y += 17;
        int64_t desync = -1;
        for (auto& p : s_->peers()) desync = std::max(desync, p.desyncAt);
        if (desync >= 0) std::snprintf(buf, sizeof buf, "hash mismatch detected at frame %lld", (long long)desync);
        else if (!p0.myHashes.empty()) std::snprintf(buf, sizeof buf, "hashes agree (last check: frame %u)", p0.myHashes.rbegin()->first);
        else std::snprintf(buf, sizeof buf, "hashes: first check at frame 60");
        drawText(buf, {tx, y}, 11, desync >= 0 ? pal.path : pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::slider({12, 8, 180, 26}, "latency", &latency_, 0, 200, "%.0f ms", 5);
        ch |= ui::slider({200, 8, 180, 26}, "jitter", &jitter_, 0, 100, "%.0f ms", 5);
        ch |= ui::slider({388, 8, 160, 26}, "loss", &loss_, 0, 20, "%.0f%%", 1);
        ch |= ui::slider({556, 8, 160, 26}, "D", &delay_, 1, 15, "%.0f", 1);
        ch |= ui::toggle({12, 44, 80, 26}, "resend", &resend_);
        if (ch) reset(), notifyProblemChanged();
        if (ui::button({100, 44, 150, 26}, "corrupt peer 2")) s_->corrupt(2);
        if (ui::button({258, 44, 80, 26}, "restart")) reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "frame %u · tick %ld", s_->peers()[0].sim.frame, s_->ticks());
        return buf;
    }

private:
    std::unique_ptr<LockstepSession> s_;
    std::array<std::deque<int>, 4> hist_;
    std::array<uint32_t, 4> prevFrame_{};
    float latency_ = 40, jitter_ = 20, loss_ = 2, delay_ = 6;
    bool resend_ = true;
};

}  // namespace

int main() {
    static LockstepNetDemo demo;
    ga::DemoConfig config;
    config.title = "Lockstep over a lossy network";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
