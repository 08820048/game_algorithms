// 2D 相机：跟随、死区、平台吸附、前视
//   - 角色由自动驾驶控制：走、停、原地小碎步、跳到另一个高度的平台、偶尔往回走
//   - 虚线框是死区，十字是相机中心，空心圆是相机正在追的"期望位置"
//   - 右下角：最近 10 秒相机每秒移动的距离，以及水平方向来回摆动的次数

#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "camera.h"
#include "demo.h"
#include "trace.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::camera;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr float kZoom = 0.8f;
const Vec2 kCentre{kWidth / 2.0f, kTop + (kHeight - kTop) / 2.0f};

Vector2 rv(Vec2 v) { return {v.x, v.y}; }

class CameraDemo final : public Demo {
public:
    CameraDemo() { reset(); }
    void reset() override {
        TraceOptions to;
        to.seed = uint64_t(seed_);
        pilot_ = Autopilot(to);
        rig_ = Rig(options());
        rig_.reset(pilot_.sample().pos);
        trail_.clear(), ground_.clear(), history_.clear();
        frame_ = 0, lastSign_ = 0;
    }
    bool step() override {
        const float dt = 1 / 60.0f;
        Vec2 before = rig_.position();
        pilot_.step(dt);
        PlayerSample s = pilot_.sample();
        rig_.update(s, dt);
        ++frame_;
        trail_.push_back(s.pos);
        if (trail_.size() > 180) trail_.pop_front();
        if (s.grounded) ground_.push_back(s.pos);
        if (ground_.size() > 4000) ground_.erase(ground_.begin(), ground_.begin() + 1000);
        Vec2 d = rig_.position() - before;
        int sign = d.x > dt ? 1 : d.x < -dt ? -1 : 0;
        int rev = 0;
        if (sign) {
            rev = lastSign_ && sign != lastSign_;
            lastSign_ = sign;
        }
        history_.push_back({std::fabs(d.x), std::fabs(d.y), rev});
        if (history_.size() > 600) history_.pop_front();
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        Vec2 cam = rig_.position();
        auto toScreen = [&](Vec2 w) { return kCentre + (w - cam) * kZoom; };
        // 世界里固定的网格：相机一动，网格就跟着动
        float step = 80;
        float x0 = std::floor((cam.x - kWidth / kZoom) / step) * step, y0 = std::floor((cam.y - kHeight / kZoom) / step) * step;
        for (float x = x0; x < cam.x + kWidth / kZoom; x += step) {
            Vec2 a = toScreen({x, 0});
            if (a.x >= 0 && a.x <= kWidth) DrawLineEx({a.x, kTop}, {a.x, float(kHeight)}, hl, Fade(pal.gridLine, 0.8f));
        }
        for (float y = y0; y < cam.y + kHeight / kZoom; y += step) {
            Vec2 a = toScreen({0, y});
            if (a.y >= kTop && a.y <= kHeight) DrawLineEx({0, a.y}, {float(kWidth), a.y}, hl, Fade(pal.gridLine, 0.8f));
        }
        // 走过的地面：落地时脚下的位置
        for (size_t i = 1; i < ground_.size(); ++i) {
            Vec2 a = toScreen(ground_[i - 1] + Vec2{0, 16}), b = toScreen(ground_[i] + Vec2{0, 16});
            if (std::fabs(ground_[i].y - ground_[i - 1].y) < 1 && std::fabs(ground_[i].x - ground_[i - 1].x) < 20) DrawLineEx(rv(a), rv(b), 4, Fade(pal.text, 0.5f));
        }
        for (size_t i = 1; i < trail_.size(); ++i) DrawLineEx(rv(toScreen(trail_[i - 1])), rv(toScreen(trail_[i])), 1.2f, Fade(pal.path, 0.1f + 0.4f * float(i) / float(trail_.size())));
        PlayerSample s = pilot_.sample();
        Vec2 p = toScreen(s.pos);
        DrawRectangleRec({p.x - 9, p.y - 16, 18, 32}, pal.path);
        // 死区、期望位置、相机中心
        const RigOptions& o = rig_.options();
        if (o.deadZone.x > 0) {
            Vec2 g = toScreen(rig_.goal() - Vec2{o.lookAhead > 0 ? rig_.lookOffset() : 0, 0});
            Vec2 h = o.deadZone * kZoom;
            Vector2 c[4] = {{g.x - h.x, g.y - h.y}, {g.x + h.x, g.y - h.y}, {g.x + h.x, g.y + h.y}, {g.x - h.x, g.y + h.y}};
            for (int k = 0; k < 4; ++k) drawDashedLine(c[k], c[(k + 1) % 4], 1.2f, pal.text);
        }
        Vec2 g = toScreen(rig_.goal());
        DrawCircleLinesV(rv(g), 7, pal.text);
        DrawLineEx({kCentre.x - 10, kCentre.y}, {kCentre.x + 10, kCentre.y}, 1.2f, pal.textMuted);
        DrawLineEx({kCentre.x, kCentre.y - 10}, {kCentre.x, kCentre.y + 10}, 1.2f, pal.textMuted);
        if (o.platformSnap) {
            Vec2 gy = toScreen({cam.x, rig_.groundY() + 16});
            drawDashedLine({0, gy.y}, {float(kWidth), gy.y}, hl, Fade(pal.frontier, 0.9f));
        }

        double tx = 0, ty = 0;
        int rev = 0;
        for (const H& h : history_) tx += h.dx, ty += h.dy, rev += h.rev;
        double secs = std::max<size_t>(1, history_.size()) / 60.0;
        char buf[128];
        DrawRectangleRec({kWidth - 260.0f, kHeight - 58.0f, 252, 50}, Fade(pal.background, 0.85f));
        std::snprintf(buf, sizeof buf, "camera travel x %3.0f  y %3.0f px/s", tx / secs, ty / secs);
        drawText(buf, {kWidth - 252.0f, kHeight - 52.0f}, 11, pal.text);
        std::snprintf(buf, sizeof buf, "sway %2d in %.0f s", rev, secs);
        drawText(buf, {kWidth - 252.0f, kHeight - 34.0f}, 11, pal.text);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::toggle({12, 10, 90, 26}, "smooth", &smooth_);
        ch |= ui::toggle({108, 10, 110, 26}, "dead zone", &deadZone_);
        ch |= ui::toggle({224, 10, 130, 26}, "platform snap", &snap_);
        ch |= ui::toggle({360, 10, 110, 26}, "look-ahead", &look_);
        ch |= ui::toggle({476, 10, 120, 26}, "hysteresis", &hyst_);
        ch |= ui::slider({12, 44, 200, 26}, "seed", &seed_, 1, 20, "%.0f", 1);
        if (ui::button({224, 44, 70, 26}, "lock")) smooth_ = deadZone_ = snap_ = look_ = hyst_ = false, ch = true;
        if (ui::button({300, 44, 60, 26}, "all")) smooth_ = deadZone_ = snap_ = look_ = hyst_ = true, ch = true;
        if (ch) {
            // 只换相机的设置，不打断角色
            Vec2 c = rig_.position();
            rig_ = Rig(options());
            rig_.reset(c);
            history_.clear(), lastSign_ = 0;
        }
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "t %.0f s", frame_ / 60.0);
        return buf;
    }

private:
    struct H {
        float dx, dy;
        int rev;
    };
    RigOptions options() const {
        RigOptions o;
        o.smooth = smooth_;
        o.deadZone = deadZone_ ? Vec2{60, 40} : Vec2{0, 0};
        o.platformSnap = snap_;
        o.lookAhead = look_ ? 100 : 0;
        o.turnHysteresis = hyst_ ? 60 : 0;
        return o;
    }

    Autopilot pilot_;
    Rig rig_;
    std::deque<Vec2> trail_;
    std::vector<Vec2> ground_;
    std::deque<H> history_;
    long long frame_ = 0;
    int lastSign_ = 0;
    float seed_ = 1;
    bool smooth_ = false, deadZone_ = false, snap_ = false, look_ = false, hyst_ = false;
};

}  // namespace

int main() {
    static CameraDemo demo;
    ga::DemoConfig config;
    config.title = "2D camera";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
