// 遮挡剔除：一维深度缓冲
//   - 视点沿街道走（walk）；关掉 walk 后拖动画布移动视点，heading 调方向
//   - 楼（树）光栅化进深度缓冲：每一列记下"完全被盖住时，遮挡物在这一列里最远的深度"，朱红折线就是这些深度
//   - 底部一条是"屏幕"：每一列的深度，越近越深；空白 = 没有被完全盖住
//   - 墨色实心：要画；灰色空心：被遮挡剔除；淡色：在视锥外。ideal 是逐条视线算出的真正看不见的数量
//   - fusion 关掉：每个遮挡物单独判断（物体必须被某一个完全挡住）。在 forest 里差别最明显

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "culling.h"
#include "demo.h"
#include "scene.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::culling;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kStrip = 22;
constexpr float kBottom = kHeight - kStrip - 6;

class OcclusionDemo final : public Demo {
public:
    OcclusionDemo() { build(); }
    void build() {
        Pcg32 r{scene_ == 0 ? 8u : 9u};
        Aabb world{{0, kTop + 6}, {float(kWidth), kBottom}};
        occluders_.clear(), solids_.clear(), items_.clear();
        if (scene_ == 0) {
            City c = makeCity(r, world, 6, 3, 26, 420);
            solids_ = c.buildings, occluders_ = c.occluders, items_ = c.props;
            route_.clear();
            // 沿内圈街道绕一圈
            float bw = float(kWidth) / 6, bh = (kBottom - world.min.y) / 3;
            for (Vec2 g : {Vec2{1, 1}, Vec2{5, 1}, Vec2{5, 2}, Vec2{3, 2}, Vec2{3, 1}, Vec2{1, 1}, Vec2{1, 2}, Vec2{5, 2}, Vec2{5, 1}})
                route_.push_back({world.min.x + g.x * bw, world.min.y + g.y * bh});
        } else {
            solids_ = randomBoxes(r, {world.min + Vec2{40, 0}, world.max}, 160, 4, 7);
            for (const Aabb& t : solids_) occluders_.push_back(corners(t));
            for (const Aabb& b : randomBoxes(r, world, 400, 9, 18)) {
                bool clash = b.min.x < 70;
                for (const Aabb& t : solids_) clash |= overlaps(t, b);
                if (!clash) items_.push_back(b);
                if (items_.size() >= 110) break;
            }
            route_ = {{30, world.min.y + 40}, {30, kBottom - 40}};
        }
        reset();
    }
    void reset() override {
        leg_ = 0, along_ = 0;
        if (walk_) eye_ = route_[0];
        look_ = 0;
    }
    bool step() override {
        if (!walk_) return true;
        time_ += 1 / 60.0f;
        Vec2 a = route_[leg_], b = route_[(leg_ + 1) % route_.size()];
        float len = length(b - a), speed = scene_ == 0 ? 60 : 25;
        along_ += speed / 60;
        if (along_ >= len) along_ -= len, leg_ = (leg_ + 1) % route_.size(), a = route_[leg_], b = route_[(leg_ + 1) % route_.size()];
        eye_ = a + normalized(b - a) * along_;
        float target = scene_ == 0 ? angleOf(b - a) + 0.5f * std::sin(time_ * 0.9f) : 0.6f * std::sin(time_ * 0.5f);
        look_ += std::remainder(target - look_, 2 * kPi) * 0.06f;
        return true;
    }
    void update() override {
        Vector2 m = GetMousePosition();
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop + 4 && m.y < kBottom) {
            eye_ = {m.x, m.y};
            if (walk_) walk_ = false, heading_ = look_ * 180 / kPi;
        }
    }

    View view() const {
        float a = walk_ ? look_ : heading_ * kPi / 180;
        return {eye_, fromAngle(a), kPi / 4, 4, 1200};
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        View v = view();
        Frustum f = makeFrustum(v);
        int bins = 1 << int(binsLog_);
        DepthBuffer db(bins);
        db.begin(v);
        std::vector<std::vector<Vec2>> active;  // 视锥内、不包含视点的遮挡物
        for (size_t i = 0; i < occluders_.size(); ++i)
            if (classify(f, solids_[i]) != Cull::Outside && !contains(solids_[i], eye_)) active.push_back(occluders_[i]);
        for (const auto& o : active) db.rasterize(o);
        db.buildHiZ();
        std::vector<char> indiv;
        if (!fusion_) {
            DepthBuffer scratch(bins);
            cullIndividually(scratch, v, active, items_, indiv);
        }

        for (const Aabb& b : solids_)
            DrawRectangleRec({b.min.x, b.min.y, b.max.x - b.min.x, b.max.y - b.min.y}, Fade(pal.wall, scene_ == 0 ? 0.16f : 0.5f));
        // 视锥
        Vector2 c[4];
        for (int i = 0; i < 4; ++i) c[i] = {f.corners[i].x, f.corners[i].y};
        BeginScissorMode(0, int(kTop), kWidth, int(kBottom - kTop));
        drawDashedLine({eye_.x, eye_.y}, c[3], hl, pal.textMuted);
        drawDashedLine({eye_.x, eye_.y}, c[2], hl, pal.textMuted);
        // 深度缓冲：每列在存下的深度处画一小段
        float tn = std::tan(v.halfFov);
        auto at = [&](float s, float z) { Vec2 p = v.eye + (v.dir + v.right() * (s * tn)) * z; return Vector2{p.x, p.y}; };
        for (int i = 0; i < bins; ++i) {
            float z = db.depth(i);
            if (z == kInf) continue;
            DrawLineEx(at(db.binEdge(i), z), at(db.binEdge(i + 1), z), 1.6f, pal.path);
        }
        int inF = 0, culled = 0, ideal = 0, wrong = 0;
        for (size_t i = 0; i < items_.size(); ++i) {
            const Aabb& b = items_[i];
            Rectangle r{b.min.x, b.min.y, b.max.x - b.min.x, b.max.y - b.min.y};
            if (!overlapsExact(f, b)) {
                DrawRectangleLinesEx(r, hl, Fade(pal.textMuted, 0.35f));
                continue;
            }
            ++inF;
            bool occ = fusion_ ? (hiz1_ ? db.occludedHiZ(b, false) : db.occluded(b)) : indiv[i] != 0;
            bool hidden = !visibleByRays(v, b, active, 24);
            culled += occ, ideal += hidden;
            if (occ && !hidden) ++wrong;
            if (occ) DrawRectangleLinesEx(r, 1.0f, Fade(pal.textMuted, 0.9f));
            else if (hidden) DrawRectangleLinesEx(r, 1.6f, pal.text);  // 其实看不见，但没剔除掉
            else DrawRectangleRec(r, pal.text);
        }
        EndScissorMode();
        DrawCircleV({eye_.x, eye_.y}, 5, pal.path);
        drawArrow({eye_.x, eye_.y}, {eye_.x + v.dir.x * 22, eye_.y + v.dir.y * 22}, 1.6f, pal.path, 7);

        // 屏幕条
        float x0 = 12, w = kWidth - 24, y = kHeight - kStrip - 2;
        DrawRectangleLinesEx({x0, y, w, kStrip}, hl, pal.gridLine);
        for (int i = 0; i < bins; ++i) {
            float z = db.depth(i);
            if (z == kInf) continue;
            float k = std::clamp(z / 500.0f, 0.0f, 1.0f);
            DrawRectangleRec({x0 + w * i / bins, y, w / bins + 0.5f, kStrip}, Fade(pal.text, 0.85f - 0.7f * k));
        }
        char buf[160];
        std::snprintf(buf, sizeof buf, "in frustum %d   culled %d   ideal %d   drawn %d%s   occluders %zu", inF, culled, ideal,
                      inF - culled, wrong ? "   WRONG" : "", active.size());
        drawText(buf, {12, kTop + 8}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* scenes[2] = {"city", "forest"};
        for (int i = 0; i < 2; ++i)
            if (ui::button({12.0f + i * 74, 10, 70, 26}, scenes[i], scene_ == i) && scene_ != i) scene_ = i, build();
        ui::toggle({166, 10, 66, 26}, "walk", &walk_);
        ui::slider({244, 10, 220, 26}, "bins (log2)", &binsLog_, 3, 10, "%.0f", 1);
        ui::toggle({476, 10, 80, 26}, "fusion", &fusion_);
        ui::toggle({562, 10, 110, 26}, "hi-z 1 level", &hiz1_);
        if (!walk_) ui::slider({12, 44, 260, 26}, "heading", &heading_, -180, 180, "%.0f°", 1);
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%d bins%s", 1 << int(binsLog_), fusion_ ? "" : " · no fusion");
        return buf;
    }

private:
    int scene_ = 0;
    std::vector<Aabb> solids_, items_;
    std::vector<std::vector<Vec2>> occluders_;
    std::vector<Vec2> route_;
    Vec2 eye_{};
    size_t leg_ = 0;
    float along_ = 0, look_ = 0, time_ = 0, heading_ = 0, binsLog_ = 7;
    bool walk_ = true, fusion_ = true, hiz1_ = false;
};

}  // namespace

int main() {
    static OcclusionDemo demo;
    ga::DemoConfig config;
    config.title = "Occlusion culling";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
