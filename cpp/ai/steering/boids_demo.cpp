// Boids 演示
//   - 三个滑块调整分离、对齐、聚合的权重；radius 是感知半径，view 是视野角度
//   - 点击一只鸟选中它：高亮它的邻居、视野扇形，以及三条规则各自的力
//   - grid：显示网格，选中鸟周围要检查的 3×3 个格子
//   - predator：鼠标变成捕食者，附近的鸟会逃开

#include <cmath>
#include <cstdio>
#include <string>

#include "boids.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::boids;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 84;
constexpr Vec2 kOrigin{16, kTop + 8};
constexpr float kArenaW = kWidth - 32, kArenaH = kHeight - kTop - 24;
constexpr float kDt = 1.0f / 60;

inline Vector2 rl(Vec2 v) { return {v.x + kOrigin.x, v.y + kOrigin.y}; }

class BoidsDemo final : public Demo {
public:
    BoidsDemo() { reset(); }

    void reset() override {
        Params p = params();
        Pcg32 rng(21);
        flock_ = randomFlock(static_cast<int>(count_), p, rng);
        frame_ = 0;
    }

    bool step() override {
        Params p = params();
        if (static_cast<int>(flock_.size()) != static_cast<int>(count_)) resize(p);
        grid_.build(flock_, p);
        grid_.forces(flock_, p, forces_);
        if (predator_ && mouseInArena_) {  // 捕食者：在原来的力之上叠加一个逃离的力
            for (size_t i = 0; i < flock_.size(); ++i) {
                Vec2 d = flock_[i].pos - mouse_;
                float dist = length(d);
                if (dist < 110 && dist > 1e-3f) forces_[i] += d / dist * (p.maxForce * 3 * (1 - dist / 110));
            }
        }
        integrate(flock_, forces_, p, kDt);
        ++frame_;
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        mouse_ = {m.x - kOrigin.x, m.y - kOrigin.y};
        mouseInArena_ = mouse_.x >= 0 && mouse_.y >= 0 && mouse_.x <= kArenaW && mouse_.y <= kArenaH && !ui::wantsMouse();
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && mouseInArena_) {
            float best = 18 * 18;
            for (const Boid& b : flock_)
                if (lengthSq(b.pos - mouse_) < best) best = lengthSq(b.pos - mouse_), selected_ = b.id;
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        Params p = params();
        const Boid* sel = selectedBoid();
        if (showGrid_) {
            float c = grid_.cellSize();
            for (float x = c; x < kArenaW; x += c) DrawLineEx(rl({x, 0}), rl({x, kArenaH}), hl, pal.gridLine);
            for (float y = c; y < kArenaH; y += c) DrawLineEx(rl({0, y}), rl({kArenaW, y}), hl, pal.gridLine);
            if (sel) {  // 选中鸟要检查的 3×3 个格子
                int cx = grid_.cellX(sel->pos.x), cy = grid_.cellY(sel->pos.y);
                float x0 = std::max(cx - 1, 0) * c, y0 = std::max(cy - 1, 0) * c;
                float x1 = std::min((cx + 2) * c, kArenaW), y1 = std::min((cy + 2) * c, kArenaH);
                DrawRectangleRec({rl({x0, y0}).x, rl({x0, y0}).y, x1 - x0, y1 - y0}, Fade(pal.frontier, 0.12f));
            }
        }
        DrawRectangleLinesEx({kOrigin.x, kOrigin.y, kArenaW, kArenaH}, hl * 1.5f, pal.textMuted);

        // 选中的鸟：视野扇形与邻居
        Accum acc;
        if (sel) {
            Vec2 fwd = forward(*sel);
            float half = std::acos(std::clamp(p.fovCos, -1.0f, 1.0f)) * 180 / kPi, heading = angleOf(fwd) * 180 / kPi;
            DrawCircleSector(rl(sel->pos), p.radius, heading - half, heading + half, 48, Fade(pal.frontier, 0.18f));
            DrawCircleLinesV(rl(sel->pos), p.separation, Fade(pal.path, 0.45f));
            for (const Boid& o : flock_) {
                if (o.id == sel->id) continue;
                Accum one;
                addNeighbor(one, *sel, fwd, o, p);
                if (one.count) {
                    addNeighbor(acc, *sel, fwd, o, p);
                    DrawLineEx(rl(sel->pos), rl(o.pos), hl, Fade(pal.textMuted, 0.5f));
                }
            }
        }
        for (const Boid& b : flock_) drawBoid(b, sel && b.id == sel->id ? pal.path : pal.text);
        if (sel) drawRuleForces(*sel, acc, p);
        if (predator_ && mouseInArena_) {
            DrawCircleLinesV(rl(mouse_), 110, Fade(pal.path, 0.35f));
            DrawCircleV(rl(mouse_), 7, pal.path);
        }
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        ui::slider({16, 13, 250, 26}, "separation", &wSep_, 0, 4, "%.1f", 0.1f);
        ui::slider({276, 13, 250, 26}, "alignment", &wAli_, 0, 4, "%.1f", 0.1f);
        ui::slider({536, 13, 248, 26}, "cohesion", &wCoh_, 0, 4, "%.1f", 0.1f);
        ui::slider({16, 47, 200, 26}, "radius", &radius_, 16, 120, "%.0f", 2);
        ui::slider({226, 47, 190, 26}, "view", &view_, 60, 360, "%.0f deg", 10);
        ui::slider({426, 47, 170, 26}, "boids", &count_, 20, 600, "%.0f", 10);
        ui::toggle({606, 47, 70, 26}, "grid", &showGrid_);
        ui::toggle({684, 47, 100, 26}, "predator", &predator_);
    }

    std::string status() const override {
        char buf[128];
        std::snprintf(buf, sizeof buf, "%zu boids · order %.2f · %d groups", flock_.size(),
                      polarization(flock_), frame_ % 30 == 0 || cachedGroups_ < 0 ? (cachedGroups_ = groups(flock_, params().radius)) : cachedGroups_);
        return buf;
    }

private:
    Params params() const {
        Params p;
        p.wSeparation = wSep_, p.wAlignment = wAli_, p.wCohesion = wCoh_;
        p.radius = radius_;
        p.separation = std::min(22.0f, radius_);
        p.fovCos = std::cos(view_ / 2 * kPi / 180);
        p.width = kArenaW, p.height = kArenaH;
        return p;
    }

    const Boid* selectedBoid() const {
        for (const Boid& b : flock_)
            if (b.id == selected_) return &b;
        return nullptr;
    }

    void resize(const Params& p) {
        int n = static_cast<int>(count_);
        if (n < static_cast<int>(flock_.size())) {  // 去掉 id 最大的那些
            std::erase_if(flock_, [&](const Boid& b) { return b.id >= n; });
            return;
        }
        Pcg32 rng(static_cast<uint64_t>(frame_) + 99);
        auto extra = randomFlock(n - static_cast<int>(flock_.size()), p, rng);
        int id = static_cast<int>(flock_.size());
        for (Boid& b : extra) b.id = id++, flock_.push_back(b);
    }

    void drawBoid(const Boid& b, Color c) const {
        Vec2 f = forward(b), s = perp(f);
        Vec2 p = b.pos;
        Vector2 a = rl(p + f * 7), l = rl(p - f * 5 + s * 4), r = rl(p - f * 5 - s * 4);
        DrawTriangle(a, r, l, c);
        DrawTriangle(a, l, r, c);
    }

    // 选中鸟的三条规则各自的力（与 boids.h 中 steer 的计算相同），按 0.3 秒的效果缩放
    void drawRuleForces(const Boid& b, const Accum& acc, const Params& p) const {
        if (acc.count == 0) return;
        const Palette& pal = palette();
        auto toward = [&](Vec2 dir) { return truncate((normalized(dir) * p.maxSpeed - b.vel) / p.reaction, p.maxForce); };
        Vec2 sep = truncate(acc.away, 1) * (p.maxForce * p.wSeparation);
        Vec2 ali = lengthSq(acc.heading) > 0 ? toward(acc.heading) * p.wAlignment : Vec2{};
        Vec2 coh = acc.offset / (acc.count * p.radius) * (p.maxForce * p.wCohesion);
        const float k = 0.3f;
        if (lengthSq(sep) > 1) drawArrow(rl(b.pos), rl(b.pos + sep * k), 2, pal.path, 7);
        if (lengthSq(ali) > 1) drawArrow(rl(b.pos), rl(b.pos + ali * k), 2, pal.frontier, 7);
        if (lengthSq(coh) > 1) drawArrow(rl(b.pos), rl(b.pos + coh * k), 2, pal.text, 7);
        float x = kOrigin.x + 10, y = kOrigin.y + kArenaH - 18;
        char buf[64];
        std::snprintf(buf, sizeof buf, "selected: %d neighbours", acc.count);
        drawText(buf, {x, y}, 10, pal.textMuted);
        x += 150;
        const Color colors[] = {pal.path, pal.frontier, pal.text};
        const char* names[] = {"separation", "alignment", "cohesion"};
        for (int i = 0; i < 3; ++i) {
            DrawLineEx({x, y + 6}, {x + 18, y + 6}, 2, colors[i]);
            drawText(names[i], {x + 24, y}, 10, pal.textMuted);
            x += 100;
        }
    }

    std::vector<Boid> flock_;
    std::vector<Vec2> forces_;
    FlockGrid grid_;
    float wSep_ = 1.6f, wAli_ = 1.0f, wCoh_ = 0.5f, radius_ = 48, view_ = 240, count_ = 150;
    bool showGrid_ = false, predator_ = false, mouseInArena_ = false;
    Vec2 mouse_;
    int selected_ = 0;
    long frame_ = 0;
    mutable int cachedGroups_ = -1;
};

}  // namespace

int main() {
    static BoidsDemo demo;
    ga::DemoConfig config;
    config.title = "Boids";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
