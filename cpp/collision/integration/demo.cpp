// 数值积分演示：四种积分方法同时运行同一个问题
//   spring：弹簧振子的相图（横轴位置、纵轴速度）。精确解是一个圆
//   orbit ：行星绕恒星运动。精确解是一个闭合的椭圆
//   jump  ：抛体。每个点是一步；精确解是虚线抛物线
// 右侧：每种方法的能量误差（jump 场景显示最高点），下方是能量误差随时间的变化

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "integration.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Rectangle kView{16, kTop + 12, 504, kHeight - kTop - 24};
constexpr float kPanelX = 538;
constexpr int kMethods = 4;
constexpr int kTrail = 3000;
constexpr int kHistory = 400;

const double kPiD = std::acos(-1.0);
const char* kNames[kMethods] = {"Euler", "semi-implicit", "Verlet", "RK4"};

enum class Scene { Spring, Orbit, Jump };

Color methodColor(int m) {
    const Palette& p = palette();
    switch (m) {
        case 0: return p.path;
        case 1: return p.frontier;
        case 2: return p.text;
        default: return isDarkTheme() ? Color{120, 160, 220, 255} : Color{48, 98, 168, 255};
    }
}

class IntegrationDemo final : public Demo {
public:
    IntegrationDemo() { reset(); }

    void reset() override {
        StateT<Vec2d> s0 = initial();
        for (int m = 0; m < kMethods; ++m) state_[m] = s0, trail_[m].clear(), history_[m].clear(), apex_[m] = 0;
        verlet_ = makeVerlet(s0, Accel{this});
        time_ = 0;
        e0_ = energy(s0);
        for (int m = 0; m < kMethods; ++m) trail_[m].push_back(display(s0));
        // orbit 的参考轨道：RK4 以很小的步长跑一圈
        reference_.clear();
        if (scene_ == Scene::Orbit) {
            StateT<Vec2d> s = s0;
            double period = orbitPeriod(), h = period / 2000;
            for (int i = 0; i <= 2000; ++i) reference_.push_back(s.x), s = rk4(s, h, Accel{this});
        }
        setStepsPerSecond(fps_);
    }

    bool step() override {
        double dt = 1.0 / fps_;
        Accel acc{this};
        state_[0] = explicitEuler(state_[0], dt, acc);
        state_[1] = semiImplicitEuler(state_[1], dt, acc);
        verlet_ = velocityVerlet(verlet_, dt, acc);
        state_[2] = {verlet_.x, verlet_.v};
        state_[3] = rk4(state_[3], dt, acc);
        time_ += dt;
        for (int m = 0; m < kMethods; ++m) {
            trail_[m].push_back(display(state_[m]));
            if (trail_[m].size() > kTrail) trail_[m].erase(trail_[m].begin());
            history_[m].push_back(static_cast<float>(energy(state_[m]) / e0_ - 1));
            if (history_[m].size() > kHistory) history_[m].erase(history_[m].begin());
            if (scene_ == Scene::Jump) apex_[m] = std::max(apex_[m], state_[m].x.y);
        }
        // 跳跃：精确解落地后停一会儿，再重新起跳
        if (scene_ == Scene::Jump && time_ > jumpFlightTime() + 0.6) reset();
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        BeginScissorMode(int(kView.x), int(kView.y), int(kView.width), int(kView.height));
        drawReference();
        for (int m = kMethods - 1; m >= 0; --m) {
            if (!show_[m]) continue;
            Color c = methodColor(m);
            const auto& t = trail_[m];
            for (size_t i = 1; i < t.size(); ++i) DrawLineEx(toScreen(t[i - 1]), toScreen(t[i]), 1.6f, Fade(c, 0.85f));
            if (scene_ == Scene::Jump)
                for (const Vec2d& p : t) DrawCircleV(toScreen(p), 2.6f, c);
            DrawCircleV(toScreen(display(state_[m])), 5, c);
        }
        EndScissorMode();
        DrawRectangleLinesEx(kView, hl * 1.5f, pal.textMuted);
        drawPanel();
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* scenes[] = {"spring", "orbit", "jump"};
        for (int i = 0; i < 3; ++i)
            if (ui::button({16 + 66.0f * i, 13, 60, 26}, scenes[i], int(scene_) == i) && int(scene_) != i) {
                scene_ = Scene(i);
                fps_ = scene_ == Scene::Spring ? 20 : scene_ == Scene::Orbit ? 40 : 12;
                reset();
            }
        if (ui::slider({222, 13, 210, 26}, "steps/s", &fps_, 4, 120, "%.0f", 1)) reset();
        float y = kTop + 16;
        for (int m = 0; m < kMethods; ++m, y += 30) ui::toggle({kPanelX + 18, y, 118, 24}, kNames[m], &show_[m]);
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "dt = %.3f s · t = %.1f s", 1.0 / fps_, time_);
        return buf;
    }

private:
    struct Accel {
        const IntegrationDemo* d;
        Vec2d operator()(const Vec2d& x, const Vec2d& v) const { return d->accel(x, v); }
    };

    // ---- 三个问题 ----
    static constexpr double kOmega = 2 * 3.14159265358979323846;  // 弹簧：周期 1 秒
    static constexpr double kRp = 110, kE = 0.5;                  // 轨道：近日点距离、偏心率
    static constexpr double kG = 1400;                            // 跳跃：重力加速度（像素 / 秒²）
    static constexpr Vec2d kLaunch{520, 600};

    double orbitA() const { return kRp / (1 - kE); }
    double orbitPeriod() const { return 3.0; }
    double gm() const { return 4 * kPiD * kPiD * std::pow(orbitA(), 3) / (orbitPeriod() * orbitPeriod()); }
    double jumpFlightTime() const { return 2 * kLaunch.y / kG; }

    StateT<Vec2d> initial() const {
        switch (scene_) {
            case Scene::Spring: return {{1, 0}, {0, 0}};
            case Scene::Orbit: return {{kRp, 0}, {0, std::sqrt(gm() * (1 + kE) / kRp)}};
            default: return {{0, 0}, kLaunch};
        }
    }
    Vec2d accel(const Vec2d& x, const Vec2d&) const {
        switch (scene_) {
            case Scene::Spring: return x * -(kOmega * kOmega);
            case Scene::Orbit: {
                double r = length(x);
                return x * (-gm() / (r * r * r));
            }
            default: return {0, -kG};
        }
    }
    double energy(const StateT<Vec2d>& s) const {
        switch (scene_) {
            case Scene::Spring: return 0.5 * dot(s.v, s.v) + 0.5 * kOmega * kOmega * dot(s.x, s.x);
            case Scene::Orbit: return 0.5 * dot(s.v, s.v) - gm() / length(s.x);
            default: return 0.5 * dot(s.v, s.v) + kG * s.x.y;
        }
    }

    // 画在屏幕上的点。弹簧画相图：横轴位置，纵轴 v/ω（这样精确解是圆）
    Vec2d display(const StateT<Vec2d>& s) const {
        if (scene_ == Scene::Spring) return {s.x.x, s.v.x / kOmega};
        return s.x;
    }
    Vector2 toScreen(const Vec2d& p) const {
        switch (scene_) {
            case Scene::Spring: return {float(kView.x + kView.width / 2 + p.x * 150), float(kView.y + kView.height / 2 - p.y * 150)};
            case Scene::Orbit: return {float(kView.x + 380 + p.x), float(kView.y + kView.height / 2 - p.y)};
            default: return {float(kView.x + 30 + p.x), float(kView.y + kView.height - 40 - p.y)};
        }
    }

    void drawReference() const {
        const Palette& pal = palette();
        const float hl = hairline();
        if (scene_ == Scene::Spring) {
            Vector2 c{kView.x + kView.width / 2, kView.y + kView.height / 2};
            DrawLineEx({kView.x, c.y}, {kView.x + kView.width, c.y}, hl, pal.gridLine);
            DrawLineEx({c.x, kView.y}, {c.x, kView.y + kView.height}, hl, pal.gridLine);
            drawText("x", {kView.x + kView.width - 16, c.y + 4}, 12, pal.textMuted);
            drawText("v / w", {c.x + 6, kView.y + 6}, 12, pal.textMuted);
            for (int i = 0; i < 96; i += 2) {
                float a0 = i * 2 * kPi / 96, a1 = (i + 1) * 2 * kPi / 96;
                DrawLineEx({c.x + 150 * std::cos(a0), c.y + 150 * std::sin(a0)}, {c.x + 150 * std::cos(a1), c.y + 150 * std::sin(a1)}, 1.2f,
                           pal.textMuted);
            }
        } else if (scene_ == Scene::Orbit) {
            Vector2 sun = toScreen({0, 0});
            DrawCircleV(sun, 7, pal.frontier);
            for (size_t i = 2; i < reference_.size(); i += 2)
                DrawLineEx(toScreen(reference_[i - 2]), toScreen(reference_[i - 1]), 1.2f, pal.textMuted);
        } else {
            float ground = kView.y + kView.height - 40;
            DrawLineEx({kView.x, ground}, {kView.x + kView.width, ground}, 1.2f, pal.textMuted);
            Vec2d prev{0, 0};
            for (int i = 1; i <= 60; ++i) {
                double t = jumpFlightTime() * i / 60;
                Vec2d p{kLaunch.x * t, kLaunch.y * t - 0.5 * kG * t * t};
                if (i % 2) drawDashedLine(toScreen(prev), toScreen(p), 1.2f, pal.textMuted, 4, 3);
                else DrawLineEx(toScreen(prev), toScreen(p), 1.2f, pal.textMuted);
                prev = p;
            }
            double apex = kLaunch.y * kLaunch.y / (2 * kG);
            Vector2 a = toScreen({kLaunch.x * kLaunch.y / kG, apex});
            drawDashedLine({kView.x, a.y}, {kView.x + kView.width, a.y}, hl * 1.5f, pal.textMuted, 4, 4);
        }
    }

    void drawPanel() const {
        const Palette& pal = palette();
        DrawLineEx({kPanelX - 8, kTop}, {kPanelX - 8, float(kHeight)}, hairline(), pal.gridLine);
        float y = kTop + 16;
        char buf[32];
        for (int m = 0; m < kMethods; ++m, y += 30) {
            DrawRectangleRec({kPanelX + 2, y + 9, 10, 6}, methodColor(m));
            if (scene_ == Scene::Jump) {
                double exact = kLaunch.y * kLaunch.y / (2 * kG);
                // 过了最高点才有意义
                if (time_ > jumpFlightTime() / 2 + 1.0 / fps_) std::snprintf(buf, sizeof buf, "%+.1f px", apex_[m] - exact);
                else std::snprintf(buf, sizeof buf, "-");
            } else {
                double e = energy(state_[m]) / e0_ - 1;
                if (scene_ == Scene::Orbit) e = -e;  // 轨道能量为负：取反后"正 = 能量增加"
                if (std::abs(e) >= 10) std::snprintf(buf, sizeof buf, "x%.2g", 1 + e);
                else std::snprintf(buf, sizeof buf, std::abs(e) < 1e-3 ? "%+.4f%%" : "%+.1f%%", e * 100);
            }
            drawText(buf, {kWidth - 16 - measureText(buf, 12).x, y + 5}, 12, methodColor(m));
        }
        drawText(scene_ == Scene::Jump ? "apex error" : "energy error", {kPanelX + 2, y + 2}, 12, pal.textMuted);
        y += 26;

        // 能量误差随时间的变化
        Rectangle g{kPanelX + 2, y, kWidth - 18 - kPanelX, kHeight - 16 - y};
        DrawRectangleLinesEx(g, hairline(), pal.gridLine);
        // 纵轴范围：只看当前误差没超过 100% 的方法，超出的（显式欧拉）画到边缘截断
        float maxAbs = 1e-6f;
        for (int m = 0; m < kMethods; ++m)
            if (show_[m] && !history_[m].empty() && std::abs(history_[m].back()) < 1)
                for (float v : history_[m]) maxAbs = std::max(maxAbs, std::min(std::abs(v), 1.0f));
        float mid = g.y + g.height / 2;
        DrawLineEx({g.x, mid}, {g.x + g.width, mid}, hairline(), pal.textMuted);
        for (int m = 0; m < kMethods; ++m) {
            if (!show_[m]) continue;
            const auto& h = history_[m];
            for (size_t i = 1; i < h.size(); ++i) {
                auto pt = [&](size_t k) {
                    float v = std::clamp(h[k] / maxAbs, -1.0f, 1.0f) * (scene_ == Scene::Orbit ? -1 : 1);
                    return Vector2{g.x + g.width * k / kHistory, mid - v * (g.height / 2 - 4)};
                };
                DrawLineEx(pt(i - 1), pt(i), 1.4f, methodColor(m));
            }
        }
        std::snprintf(buf, sizeof buf, "+-%.2g%%", maxAbs * 100);
        drawText(buf, {g.x + 4, g.y + 3}, 11, pal.textMuted);
        drawText("energy over time", {g.x + 4, g.y + g.height - 15}, 11, pal.textMuted);
    }

    Scene scene_ = Scene::Spring;
    float fps_ = 20;
    StateT<Vec2d> state_[kMethods];
    VerletStateT<Vec2d> verlet_;
    std::vector<Vec2d> trail_[kMethods];
    std::vector<float> history_[kMethods];
    double apex_[kMethods] = {};
    std::vector<Vec2d> reference_;
    double time_ = 0, e0_ = 1;
    bool show_[kMethods] = {true, true, true, true};
};

}  // namespace

int main() {
    static IntegrationDemo demo;
    ga::DemoConfig config;
    config.title = "Numerical Integration";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 20.0f;
    return ga::runDemo(demo, config);
}
