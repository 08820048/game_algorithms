// 状态同步：快照与插值
//   - 左边是服务器（唯一的权威模拟），右边是 0 号客户端看到的画面
//   - 客户端故意落后：渲染时刻 = 现在 − 单程延迟 − 插值延迟，在它两侧的两个快照之间插值
//   - 右边的空心圆是服务器此刻的真实位置：客户端看到的永远是过去
//   - 小点是 0 号玩家最近收到的快照位置：插值就是在这些离散的点之间连线；快照越稀，转弯处切角越多
//   - 插值延迟太小（或抖动、丢包让快照晚到）时，没有比渲染时刻更新的快照，只能外推（玩家画成空心）
//   - 底部：同样的快照用三种编码各要多少带宽

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include "demo.h"
#include "net.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::net;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kSx = 12, kCx = 420, kY0 = 98;

class SnapshotDemo final : public Demo {
public:
    SnapshotDemo() { reset(); }
    void reset() override {
        StateConfig c;
        c.units = 200, c.sendEvery = kEvery[rate_], c.interpMs = kInterp[interp_], c.link = {latency_, jitter_, loss_ / 100.0};
        s_ = std::make_unique<StateSession>(c);
        extraRecent_ = 0;
    }
    bool step() override {
        long before = s_->clients()[0].extrapolated;
        s_->tick();
        bool extra = s_->clients()[0].extrapolated > before;
        extraRecent_ = extra ? 30 : std::max(0, extraRecent_ - 1);
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const Arena& a = s_->server();
        auto frame = [&](float x0, const char* title) {
            DrawRectangleLinesEx({x0, kY0, float(Arena::kW), float(Arena::kH)}, hl, pal.gridLine);
            drawText(title, {x0, kY0 + Arena::kH + 6}, 10, pal.textMuted);
        };
        frame(kSx, "server (authoritative)");
        for (auto& u : a.units) DrawCircleV({kSx + float(u.x.toDouble()), kSy(u.y.toDouble())}, 2.5f, pal.text);
        for (auto& p : a.players) DrawCircleV({kSx + float(p.x.toDouble()), kSy(p.y.toDouble())}, 6, pal.path);

        frame(kCx, "client 0 (what the player sees)");
        double now = double(s_->ticks() - 1) * 1000 / 60;
        const auto& cl = s_->clients()[0];
        Sampled v;
        BeginScissorMode(int(kCx), int(kY0), Arena::kW, Arena::kH);
        if (sampleAt(cl.snaps, s_->renderFrame(now), &v, true) && s_->renderFrame(now) >= 0) {
            for (auto [x, y] : v.units) DrawCircleV({kCx + float(x), kSy(y)}, 2.5f, pal.text);
            // 0 号玩家最近收到的快照
            int k = 0;
            for (auto it = cl.snaps.rbegin(); it != cl.snaps.rend() && k < 24; ++it, ++k) {
                QPos q = it->second.players[0];
                DrawCircleV({kCx + q.x / 64.0f, kSy(q.y / 64.0)}, 1.8f, pal.textMuted);
            }
            for (auto [x, y] : v.players) {
                if (v.extrapolated) DrawCircleLinesV({kCx + float(x), kSy(y)}, 6, pal.path);
                else DrawCircleV({kCx + float(x), kSy(y)}, 6, pal.path);
            }
        }
        for (auto& p : a.players) DrawCircleLinesV({kCx + float(p.x.toDouble()), kSy(p.y.toDouble())}, 9, pal.textMuted);  // 服务器此刻
        EndScissorMode();

        // 统计
        float y = kY0 + Arena::kH + 24;
        char buf[200];
        const auto& c = s_->config();
        double sec = std::max(1.0, double(s_->ticks())) / 60;
        std::snprintf(buf, sizeof buf, "client sees the past: %.0f ms latency + %.0f ms interpolation = %.0f ms behind the server", c.link.latencyMs,
                      c.interpMs, c.link.latencyMs + c.interpMs);
        drawText(buf, {kSx, y}, 11, pal.text), y += 17;
        std::snprintf(buf, sizeof buf, "extrapolated %.1f%% of frames%s   avg error %.2f px", 100.0 * cl.extrapolated / std::max(1L, cl.rendered),
                      extraRecent_ ? "  (extrapolating now)" : "", cl.errorSum / std::max(1.0, double(cl.rendered * 4)));
        drawText(buf, {kSx, y}, 11, extraRecent_ ? pal.path : pal.textMuted), y += 17;
        std::snprintf(buf, sizeof buf, "download per client:  full %.1f KB/s   quantized %.1f KB/s   delta %.1f KB/s", s_->encodedBytes(Encoding::Full) / 4096.0 / sec,
                      s_->encodedBytes(Encoding::Quantized) / 4096.0 / sec, s_->encodedBytes(Encoding::Delta) / 4096.0 / sec);
        drawText(buf, {kSx, y}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        const char* rates[4] = {"10 Hz", "20 Hz", "30 Hz", "60 Hz"};
        for (int i = 0; i < 4; ++i)
            if (ui::button({12.0f + i * 62, 8, 58, 26}, rates[i], rate_ == i) && rate_ != i) rate_ = i, ch = true;
        const char* interps[4] = {"interp 0", "50 ms", "100 ms", "200 ms"};
        for (int i = 0; i < 4; ++i)
            if (ui::button({272.0f + i * 72, 8, 68, 26}, interps[i], interp_ == i) && interp_ != i) interp_ = i, ch = true;
        ch |= ui::slider({12, 44, 200, 26}, "latency", &latency_, 0, 200, "%.0f ms", 5);
        ch |= ui::slider({220, 44, 200, 26}, "jitter", &jitter_, 0, 100, "%.0f ms", 5);
        ch |= ui::slider({428, 44, 180, 26}, "loss", &loss_, 0, 20, "%.0f%%", 1);
        if (ch) reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "server frame %u", s_->server().frame);
        return buf;
    }

private:
    static float kSy(double y) { return kY0 + float(y); }
    static constexpr int kEvery[4] = {6, 3, 2, 1};
    static constexpr double kInterp[4] = {0, 50, 100, 200};
    std::unique_ptr<StateSession> s_;
    int rate_ = 0, interp_ = 2, extraRecent_ = 0;
    float latency_ = 40, jitter_ = 10, loss_ = 0;
};

}  // namespace

int main() {
    static SnapshotDemo demo;
    ga::DemoConfig config;
    config.title = "Snapshots and interpolation";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
