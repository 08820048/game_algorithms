// 延迟补偿
//   - 左边是射手的画面：目标（朱红方块）是插值出来的过去的位置。目标走到墙的缺口里时射手就开枪，瞄的是画面上的目标
//   - 右边是服务器：实心方块是目标此刻的位置；收到开枪消息时，空心方块是目标退回到射手看到的时刻的位置，
//     × 是准星。不补偿时用此刻的位置判定，补偿时用退回去的位置判定
//   - 补偿后射手"看到打中就是打中"，代价是：目标可能已经躲到墙后了还被打中（服务器上标成"behind cover"）
//   - rewind cap：最多回退多久。延迟太高的射手超过上限，又打不中了

#include <algorithm>
#include <cmath>
#include <cstdio>
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

class LagCompDemo final : public Demo {
public:
    LagCompDemo() { reset(); }
    void reset() override {
        LagCompConfig c;
        c.link = {latency_, 0, 0}, c.interpMs = kInterp[interp_], c.maxRewindMs = cap_ >= 500 ? 1e9 : cap_;
        s_ = std::make_unique<LagCompSession>(c);
    }
    bool step() override {
        s_->tick();
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        auto scene = [&](float x0) {
            DrawRectangleLinesEx({x0, kY0, float(Range::kW), float(Range::kH)}, hl, pal.gridLine);
            DrawRectangleRec({x0, kY0 + float(Range::kWallY0), float(Range::kGap0), float(Range::kWallY1 - Range::kWallY0)}, pal.wall);
            DrawRectangleRec({x0 + float(Range::kGap1), kY0 + float(Range::kWallY0), float(Range::kW - Range::kGap1), float(Range::kWallY1 - Range::kWallY0)}, pal.wall);
            DrawCircleV({x0 + float(Range::kShooterX), kY0 + float(Range::kShooterY)}, 7, pal.text);
            // 射手看得见的范围
            for (double gx : {Range::kGap0, Range::kGap1}) {
                double t = (Range::kShooterY - Range::kTargetY) / (Range::kShooterY - Range::kWallY1);
                Vector2 a{x0 + float(Range::kShooterX), kY0 + float(Range::kShooterY)};
                Vector2 b{x0 + float(Range::kShooterX + (gx - Range::kShooterX) * t), kY0 + float(Range::kTargetY)};
                drawDashedLine(a, b, hl, pal.gridLine);
            }
        };
        auto box = [&](float x0, double x, Color c, bool filled) {
            Rectangle r{x0 + float(x) - Range::kHalf, kY0 + float(Range::kTargetY) - Range::kHalf, 2.0f * Range::kHalf, 2.0f * Range::kHalf};
            if (filled) DrawRectangleRec(r, c);
            else DrawRectangleLinesEx(r, 1.5f, c);
        };
        auto cross = [&](float x0, double x, double y, Color c) {
            Vector2 p{x0 + float(x), kY0 + float(y)};
            DrawLineEx({p.x - 4, p.y - 4}, {p.x + 4, p.y + 4}, 1.5f, c);
            DrawLineEx({p.x - 4, p.y + 4}, {p.x + 4, p.y - 4}, 1.5f, c);
        };
        const auto& res = s_->results();
        bool comp = compensate_;

        scene(kLx);
        box(kLx, s_->seenX(), pal.path, true);
        for (size_t k = res.size() > 6 ? res.size() - 6 : 0; k < res.size(); ++k) cross(kLx, res[k].shot.x, res[k].shot.y, pal.textMuted);
        drawText("shooter's screen (target is interpolated, in the past)", {kLx, kY0 + Range::kH + 6}, 10, pal.textMuted);

        scene(kRx);
        box(kRx, s_->targetX(), pal.path, true);
        if (!res.empty()) {
            const ShotResult& r = res.back();
            long age = s_->ticks() - long(r.serverFrame);
            if (age < 50) {
                bool hit = comp ? r.hitBack : r.hitNow;
                if (comp) box(kRx, r.backX, pal.text, false);
                cross(kRx, r.shot.x, r.shot.y, hit ? pal.path : pal.text);
                char b[64];
                std::snprintf(b, sizeof b, "%s%s", hit ? "HIT" : "miss", comp && r.behindCover ? " (target already behind cover)" : "");
                drawText(b, {kRx + 8, kY0 + 8}, 11, hit ? pal.path : pal.text);
            }
        }
        drawText(comp ? "server: outline = target rewound to what the shooter saw" : "server: judged against where the target is now",
                 {kRx, kY0 + Range::kH + 6}, 10, pal.textMuted);

        long n = long(res.size()), hn = 0, hb = 0, cover = 0;
        double rew = 0;
        for (auto& r : res) hn += r.hitNow, hb += r.hitBack, cover += r.behindCover, rew += (r.serverFrame - r.shot.renderFrame) * 1000 / 60;
        float y = kY0 + Range::kH + 24;
        char buf[200];
        const auto& c = s_->config();
        std::snprintf(buf, sizeof buf, "shooter sees the target %.0f ms late; the shot reaches the server %.0f ms later: rewind %.0f ms on average",
                      c.link.latencyMs + c.interpMs, c.link.latencyMs, n ? rew / double(n) : 0.0);
        drawText(buf, {kLx, y}, 11, pal.text), y += 17;
        std::snprintf(buf, sizeof buf, "%ld shots   hit rate without compensation %.0f%%   with compensation %.0f%%", n, n ? 100.0 * hn / double(n) : 0.0,
                      n ? 100.0 * hb / double(n) : 0.0);
        drawText(buf, {kLx, y}, 11, pal.text), y += 17;
        std::snprintf(buf, sizeof buf, "compensated hits where the target was already behind cover on the server: %.0f%%", hb ? 100.0 * cover / double(hb) : 0.0);
        drawText(buf, {kLx, y}, 11, cover ? pal.path : pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ui::toggle({12, 8, 130, 26}, "compensate", &compensate_);
        ch |= ui::slider({160, 8, 200, 26}, "latency", &latency_, 0, 200, "%.0f ms", 5);
        const char* interps[3] = {"interp 50", "100 ms", "200 ms"};
        for (int i = 0; i < 3; ++i)
            if (ui::button({380.0f + i * 82, 8, 78, 26}, interps[i], interp_ == i) && interp_ != i) interp_ = i, ch = true;
        ch |= ui::slider({12, 44, 260, 26}, "rewind cap", &cap_, 50, 500, cap_ >= 500 ? "none" : "%.0f ms", 25);
        if (ch) reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "tick %ld · %zu shots", s_->ticks(), s_->results().size());
        return buf;
    }

private:
    static constexpr double kInterp[3] = {50, 100, 200};
    std::unique_ptr<LagCompSession> s_;
    bool compensate_ = true;
    int interp_ = 1;
    float latency_ = 60, cap_ = 500;
};

}  // namespace

int main() {
    static LagCompDemo demo;
    ga::DemoConfig config;
    config.title = "Lag compensation";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
