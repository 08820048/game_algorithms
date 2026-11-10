// 震屏
//   - hit 给 trauma 加一个量；trauma 随时间线性衰减，抖动幅度 = trauma^exponent
//   - random：每帧一个新的随机偏移；noise：沿时间采样连续的噪声
//   - simulated fps：震屏按这个帧率更新（画面仍按原帧率绘制）。random 的抖动快慢随它变，noise 不变
//   - 下方曲线：最近 2 秒水平偏移，以及 trauma

#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

#include "camera.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::camera;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr Rectangle kScene{20, kTop + 14, 480, 300};
constexpr Rectangle kPlot{20, kHeight - 62.0f, 760, 52};

class ShakeDemo final : public Demo {
public:
    ShakeDemo() { reset(); }
    void reset() override {
        shake_ = Shake(options());
        acc_ = 0, time_ = 0, nextHit_ = 0.3f;
        trace_.clear();
    }
    bool step() override {
        const float dt = 1 / 60.0f;
        time_ += dt;
        if (auto_ && time_ >= nextHit_) shake_.add(0.8f), nextHit_ = time_ + 2.4f;
        // 按模拟的帧率更新震屏
        acc_ += dt;
        float simDt = 1 / fps_;
        while (acc_ >= simDt) shake_.update(simDt), acc_ -= simDt;
        trace_.push_back({shake_.offset().x, shake_.trauma()});
        if (trace_.size() > 120) trace_.pop_front();
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        // 场景：偏移 + 旋转后画几个方块和准星
        Vec2 off = shake_.offset();
        float ang = shake_.angle();
        Vec2 c{kScene.x + kScene.width / 2, kScene.y + kScene.height / 2};
        auto xf = [&](Vec2 p) {
            Vec2 d = p - c;
            Vec2 r = rotate(d, ang);
            return Vector2{c.x + r.x + off.x, c.y + r.y + off.y};
        };
        DrawRectangleLinesEx(kScene, hl, pal.gridLine);
        const float boxes[5][4] = {{60, 220, 90, 60}, {190, 180, 60, 100}, {300, 240, 120, 40}, {120, 120, 40, 40}, {360, 140, 50, 50}};
        for (auto& b : boxes) {
            Vec2 p0{kScene.x + b[0], kScene.y + b[1]};
            Vector2 q[4] = {xf(p0), xf(p0 + Vec2{b[2], 0}), xf(p0 + Vec2{b[2], b[3]}), xf(p0 + Vec2{0, b[3]})};
            for (int k = 0; k < 4; ++k) DrawLineEx(q[k], q[(k + 1) % 4], 2, pal.text);
        }
        Vector2 g0 = xf({kScene.x, kScene.y + 280}), g1 = xf({kScene.x + kScene.width, kScene.y + 280});
        DrawLineEx(g0, g1, 2, pal.textMuted);
        Vector2 cc = xf(c);
        DrawCircleLinesV(cc, 14, pal.path);
        DrawLineEx({cc.x - 22, cc.y}, {cc.x + 22, cc.y}, 1.5f, pal.path);
        DrawLineEx({cc.x, cc.y - 22}, {cc.x, cc.y + 22}, 1.5f, pal.path);

        // 右侧数值
        char buf[96];
        float y = kTop + 20;
        auto line = [&](const char* s, Color col) {
            drawText(s, {kScene.x + kScene.width + 24, y}, 12, col);
            y += 20;
        };
        std::snprintf(buf, sizeof buf, "trauma    %.2f", shake_.trauma());
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "amplitude %.2f", std::pow(shake_.trauma(), exponent_));
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "offset    %+5.1f %+5.1f px", off.x, off.y);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "angle     %+5.2f deg", ang * 180 / float(M_PI));
        line(buf, pal.text);
        DrawRectangleRec({kScene.x + kScene.width + 24, y + 4, 200 * shake_.trauma(), 8}, pal.path);
        DrawRectangleLinesEx({kScene.x + kScene.width + 24, y + 4, 200, 8}, hl, pal.gridLine);

        // 曲线
        DrawRectangleLinesEx(kPlot, hl, pal.gridLine);
        float mid = kPlot.y + kPlot.height / 2;
        DrawLineEx({kPlot.x, mid}, {kPlot.x + kPlot.width, mid}, hl, Fade(pal.gridLine, 0.7f));
        for (size_t i = 1; i < trace_.size(); ++i) {
            float x0 = kPlot.x + kPlot.width * float(i - 1) / 119, x1 = kPlot.x + kPlot.width * float(i) / 119;
            float s = kPlot.height / 2 / 30;
            DrawLineEx({x0, mid - trace_[i - 1].x * s}, {x1, mid - trace_[i].x * s}, 1.5f, pal.text);
            DrawLineEx({x0, kPlot.y + kPlot.height * (1 - trace_[i - 1].t)}, {x1, kPlot.y + kPlot.height * (1 - trace_[i].t)}, 1.2f, Fade(pal.path, 0.7f));
        }
        drawText("offset x (black) and trauma (red), last 2 s", {kPlot.x + 4, kPlot.y - 15}, 10, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 10, 70, 26}, "hit .3")) shake_.add(0.3f);
        if (ui::button({86, 10, 70, 26}, "hit .6")) shake_.add(0.6f);
        if (ui::button({160, 10, 70, 26}, "hit 1")) shake_.add(1.0f);
        ui::toggle({236, 10, 70, 26}, "auto", &auto_);
        bool ch = false;
        if (ui::button({318, 10, 80, 26}, "random", random_)) random_ = true, ch = true;
        if (ui::button({402, 10, 80, 26}, "noise", !random_)) random_ = false, ch = true;
        ch |= ui::toggle({488, 10, 100, 26}, "stagger", &stagger_);
        ch |= ui::slider({12, 44, 220, 26}, "exponent", &exponent_, 1, 3, "%.1f", 0.5f);
        ui::slider({244, 44, 260, 26}, "simulated fps", &fps_, 10, 240, "%.0f", 1);
        if (ch) {
            float t = shake_.trauma();
            shake_ = Shake(options());
            shake_.add(t);
        }
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "trauma %.2f · %s · %.0f fps", shake_.trauma(), random_ ? "random" : "noise", fps_);
        return buf;
    }

private:
    ShakeOptions options() const {
        ShakeOptions o;
        o.source = random_ ? ShakeSource::Random : ShakeSource::Noise;
        o.exponent = exponent_;
        o.staggered = stagger_;
        return o;
    }
    struct P {
        float x, t;
    };
    Shake shake_;
    std::deque<P> trace_;
    float acc_ = 0, time_ = 0, nextHit_ = 0, exponent_ = 2, fps_ = 60;
    bool auto_ = true, random_ = false, stagger_ = true;
};

}  // namespace

int main() {
    static ShakeDemo demo;
    ga::DemoConfig config;
    config.title = "Screen shake";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
