// 连续碰撞检测演示
//   bullets：左侧不断射出子弹（半径 3），飞向三面薄墙。每个点是子弹在某一帧的位置
//            discrete 只检查每帧的终点；4 substeps 把一帧分成 4 段检查；swept 求出这一帧内最早的接触时刻
//   rotate ：一根快速旋转、平移的长杆在一步之内扫过一面薄墙。每一步执行一次保守前进：
//            用 GJK 求出当前距离，除以靠近速度的上界，安全地前进这么多

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "../sat/sat.h"
#include "ccd.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr float kRadius = 3;

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }

struct Wall {
    Vec2 a, b;
};
const Wall kWalls[] = {{{330, 90}, {330, 470}}, {{490, 150}, {490, 410}}, {{650, 90}, {650, 470}}};

struct Bullet {
    Vec2 pos;
    std::vector<Vec2> trail;
    int stuckFrames = -1;  // ≥ 0：已经撞墙，停住了多少帧
};

enum class Mode { Discrete, Substeps, Swept };

class CcdDemo final : public Demo {
public:
    CcdDemo() { reset(); }

    void reset() override {
        bullets_.clear();
        fired_ = hit_ = tunneled_ = 0;
        frame_ = 0;
        rng_ = Pcg32(5);
        caT_ = 0;
        caDone_ = false;
        caHit_ = false;
        caLog_.clear();
        setStepsPerSecond(rotate_ ? 1.5f : 30.0f);
    }

    bool step() override {
        if (rotate_) return caStep();
        ++frame_;
        if (frame_ % 3 == 0) {
            Bullet b;
            // 起点 x 随机错开不到一帧的距离：否则所有子弹每帧落在同样的位置，要么全撞上、要么全穿过
            b.pos = {20 + rng_.nextFloat() * speed_, 100 + rng_.nextFloat() * 360};
            b.trail.push_back(b.pos);
            bullets_.push_back(b);
            ++fired_;
        }
        for (Bullet& b : bullets_) {
            if (b.stuckFrames >= 0) {
                ++b.stuckFrames;
                continue;
            }
            Vec2 d{speed_, 0};
            float t = 2;  // 这一帧内撞墙的时刻；> 1 表示没撞
            if (mode_ == Mode::Swept) {
                for (const Wall& w : kWalls)
                    if (auto toi = sweep(Circle{b.pos, kRadius}, d, w.a, w.b)) t = std::min(t, toi->t);
            } else {
                int sub = mode_ == Mode::Discrete ? 1 : 4;
                for (int k = 1; k <= sub && t > 1; ++k)
                    for (const Wall& w : kWalls)
                        if (touches(b.pos + d * (float(k) / sub), w)) t = float(k) / sub;
            }
            Vec2 next = b.pos + d * std::min(t, 1.0f);
            if (t <= 1) {
                b.stuckFrames = 0;
                ++hit_;
            } else {
                // 越过了某面墙所在的直线、又在墙的高度范围内，却没有被检测到：隧穿
                for (const Wall& w : kWalls)
                    if (b.pos.x < w.a.x && next.x >= w.a.x && b.pos.y > w.a.y && b.pos.y < w.b.y) ++tunneled_;
            }
            b.pos = next;
            b.trail.push_back(next);
        }
        std::erase_if(bullets_, [](const Bullet& b) { return b.pos.x > kWidth + 20 || b.stuckFrames > 45; });
        return true;
    }

    void draw() const override {
        if (rotate_) drawRotate();
        else drawBullets();
        const Palette& pal = palette();
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hairline(), pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({16, 13, 70, 26}, "bullets", !rotate_) && rotate_) rotate_ = false, reset();
        if (ui::button({92, 13, 70, 26}, "rotate", rotate_) && !rotate_) rotate_ = true, reset();
        if (!rotate_) {
            const char* names[] = {"discrete", "4 substeps", "swept"};
            float x = 178;
            for (int i = 0; i < 3; ++i) {
                float w = i == 1 ? 96.0f : 76.0f;
                if (ui::button({x, 13, w, 26}, names[i], int(mode_) == i) && int(mode_) != i) mode_ = Mode(i), reset();
                x += w + 6;
            }
            if (ui::slider({x + 10, 13, 200, 26}, "px/frame", &speed_, 2, 60, "%.0f", 1)) reset();
        } else {
            if (ui::slider({178, 13, 190, 26}, "spin", &spin_, 0, 12, "%.1f", 0.5f)) reset();
            if (ui::slider({380, 13, 190, 26}, "move", &move_, 100, 500, "%.0f", 10)) reset();
        }
    }

    std::string status() const override {
        char buf[96];
        if (rotate_) {
            if (!caDone_) std::snprintf(buf, sizeof buf, "iteration %zu · t = %.3f", caLog_.size(), caT_);
            else if (caHit_) std::snprintf(buf, sizeof buf, "time of impact t = %.3f after %zu iterations", caT_, caLog_.size());
            else std::snprintf(buf, sizeof buf, "no impact this step (%zu iterations)", caLog_.size());
        } else {
            std::snprintf(buf, sizeof buf, "fired %d · stopped %d · walls tunneled %d", fired_, hit_, tunneled_);
        }
        return buf;
    }

private:
    static bool touches(Vec2 p, const Wall& w) {
        Vec2 ab = w.b - w.a;
        float u = std::clamp(dot(p - w.a, ab) / dot(ab, ab), 0.0f, 1.0f);
        return lengthSq(p - (w.a + ab * u)) < kRadius * kRadius;
    }

    void drawBullets() const {
        const Palette& pal = palette();
        for (const Wall& w : kWalls) DrawLineEx(rl(w.a), rl(w.b), 4, pal.text);
        for (const Bullet& b : bullets_) {
            bool stuck = b.stuckFrames >= 0;
            Color c = stuck ? pal.path : pal.text;
            float fade = stuck ? 1 - b.stuckFrames / 50.0f : 1;
            for (size_t i = 0; i + 1 < b.trail.size(); ++i) DrawCircleV(rl(b.trail[i]), 1.6f, Fade(pal.textMuted, 0.7f * fade));
            // 当前这一帧扫过的区域（swept 模式检测的就是它）
            if (mode_ == Mode::Swept && !stuck && b.trail.size() >= 2) {
                Vec2 p0 = b.trail[b.trail.size() - 2];
                DrawLineEx(rl(p0), rl(b.pos), kRadius * 2, Fade(pal.frontier, 0.35f));
            }
            DrawCircleV(rl(b.pos), kRadius, Fade(c, fade));
        }
        char buf[64];
        std::snprintf(buf, sizeof buf, "wall gap a bullet can skip: speed > %.0f px/frame", 2 * kRadius);
        drawText(buf, {16, kHeight - 22.0f}, 11, pal.textMuted);
    }

    // ---- 保守前进 ----
    ConvexShape bar() const { return {makeBox({0, 0}, {70, 6}).v, 0}; }
    ConvexShape wallShape() const { return {makeBox({0, 0}, {5, 100}).v, 0}; }
    Motion barMotion() const { return {{140, 280}, 0.3f, {move_, 0}, spin_}; }
    Motion wallMotion() const { return {{440, 280}, 0, {0, 0}, 0}; }

    bool caStep() {
        if (caDone_) return false;
        Proximity p = query(poseAt(bar(), barMotion(), caT_), poseAt(wallShape(), wallMotion(), caT_));
        float bound = dot(barMotion().dpos, p.normal) + std::abs(spin_) * boundingRadius(bar());
        caLog_.push_back({caT_, p.distance, bound, p.pointA, p.pointB});
        if (p.intersecting || p.distance < kTolerance) {
            caDone_ = caHit_ = true;
            return false;
        }
        if (bound <= 0 || caT_ + p.distance / bound > 1 || caLog_.size() >= 64) {
            caDone_ = true;
            return false;
        }
        caT_ += p.distance / bound;
        return true;
    }

    void drawPolygon(const ConvexShape& s, float thick, Color c, bool dashed = false) const {
        int n = static_cast<int>(s.points.size());
        for (int i = 0; i < n; ++i) {
            Vector2 a = rl(s.points[i]), b = rl(s.points[(i + 1) % n]);
            if (dashed) drawDashedLine(a, b, thick, c, 5, 4);
            else DrawLineEx(a, b, thick, c);
        }
    }

    void drawRotate() const {
        const Palette& pal = palette();
        ConvexShape a = bar(), w = wallShape();
        Motion ma = barMotion(), mw = wallMotion();
        // 这一步内长杆扫过的姿态（淡）
        for (int k = 0; k <= 24; ++k) drawPolygon(poseAt(a, ma, k / 24.0f), 1, Fade(pal.textMuted, 0.25f));
        ConvexShape wall = poseAt(w, mw, 0);
        DrawTriangle(rl(wall.points[0]), rl(wall.points[2]), rl(wall.points[1]), Fade(pal.text, 0.15f));
        DrawTriangle(rl(wall.points[0]), rl(wall.points[3]), rl(wall.points[2]), Fade(pal.text, 0.15f));
        drawPolygon(wall, 2, pal.text);
        drawPolygon(poseAt(a, ma, 0), 1.5f, pal.textMuted);
        drawPolygon(poseAt(a, ma, 1), 1.5f, pal.textMuted, true);
        drawText("start", rl(ma.pos + Vec2{-20, 70}), 11, pal.textMuted);
        drawText("end (discrete check)", rl(ma.pos + ma.dpos + Vec2{-60, 70}), 11, pal.textMuted);
        // 每次迭代的姿态，以及当时的最近点
        for (size_t i = 0; i < caLog_.size(); ++i) {
            bool last = i + 1 == caLog_.size();
            ConvexShape s = poseAt(a, ma, caLog_[i].t);
            Color c = last ? (caDone_ && caHit_ ? pal.path : pal.text) : pal.frontier;
            drawPolygon(s, last ? 2.5f : 1.5f, c);
            if (last && !(caDone_ && caHit_)) {
                DrawLineEx(rl(caLog_[i].pa), rl(caLog_[i].pb), 1.5f, pal.path);
                DrawCircleV(rl(caLog_[i].pa), 3, pal.path);
                DrawCircleV(rl(caLog_[i].pb), 3, pal.path);
            }
        }
        // 右侧：迭代记录
        float x = 560, y = kTop + 16;
        drawText("iter     t      distance   bound    +t", {x, y}, 11, pal.textMuted);
        y += 18;
        char buf[80];
        for (size_t i = 0; i < caLog_.size() && i < 18; ++i, y += 17) {
            const auto& e = caLog_[i];
            float adv = e.bound > 0 ? e.distance / e.bound : 0;
            if (e.distance < kTolerance) std::snprintf(buf, sizeof buf, "%-4zu %6.3f %9.2f   contact", i + 1, e.t, e.distance);
            else std::snprintf(buf, sizeof buf, "%-4zu %6.3f %9.1f %7.0f %6.3f", i + 1, e.t, e.distance, e.bound, adv);
            drawText(buf, {x, y}, 11, i + 1 == caLog_.size() ? pal.text : pal.textMuted);
        }
        // 离散检测的结论
        bool endOverlaps = query(poseAt(a, ma, 1), poseAt(w, mw, 1)).intersecting;
        bool startOverlaps = query(poseAt(a, ma, 0), poseAt(w, mw, 0)).intersecting;
        std::snprintf(buf, sizeof buf, "discrete check at the end: %s", startOverlaps || endOverlaps ? "hit" : "no hit (tunnels)");
        drawText(buf, {16, kHeight - 22.0f}, 11, pal.textMuted);
    }

    struct CaEntry {
        float t, distance, bound;
        Vec2 pa, pb;
    };
    static constexpr float kTolerance = 0.5f;

    bool rotate_ = false;
    Mode mode_ = Mode::Discrete;
    float speed_ = 24;
    std::vector<Bullet> bullets_;
    int fired_ = 0, hit_ = 0, tunneled_ = 0, frame_ = 0;
    Pcg32 rng_{5};

    float spin_ = 4;
    float move_ = 420;
    float caT_ = 0;
    bool caDone_ = false, caHit_ = false;
    std::vector<CaEntry> caLog_;
};

}  // namespace

int main() {
    static CcdDemo demo;
    ga::DemoConfig config;
    config.title = "Continuous Collision Detection";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
