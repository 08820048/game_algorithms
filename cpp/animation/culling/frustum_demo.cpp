// 视锥剔除
//   - 拖动画布移动视点；spin 让视线方向慢慢转
//   - 墨色实心：完全在视锥内；墨色空心：与视锥相交；淡色：剔除
//   - planes：只做平面测试，朱红的盒子是假阳性（每个平面都不在外侧，却和视锥不相交）；exact：再测盒子的两个轴
//   - fp zone：盒子中心落在朱红区域里就会成为假阳性（盒子半宽 = box）
//   - bvh：画出层次剔除访问过的节点：虚线 = 整棵子树剔除，绿色 = 整棵子树完全在内、不再测试

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
constexpr float kTop = 80;

class FrustumDemo final : public Demo {
public:
    FrustumDemo() {
        Pcg32 r{5};
        boxes_ = randomBoxes(r, {{0, kTop}, {float(kWidth), float(kHeight)}}, 340, 1.5f, 30);
        bvh_.build(boxes_, 4);
        reset();
    }
    void reset() override { heading_ = -0.35f; }
    bool step() override {
        if (spin_) heading_ += 0.25f / 60;
        return true;
    }
    void update() override {
        Vector2 m = GetMousePosition();
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop + 4) eye_ = {m.x, m.y};
    }

    View view() const { return {eye_, fromAngle(heading_), fovDeg_ * kPi / 360, 8, far_}; }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        View v = view();
        Frustum f = makeFrustum(v);
        Color green{60, 150, 90, 255};

        if (zone_) {  // 假阳性区域：在格点上放一个半宽 box 的正方形，平面测试通过但精确测试不通过
            for (float y = kTop + 2; y < kHeight; y += 4)
                for (float x = 2; x < kWidth; x += 4) {
                    Aabb b{{x - zoneBox_, y - zoneBox_}, {x + zoneBox_, y + zoneBox_}};
                    if (classify(f, b) != Cull::Outside && !overlapsExact(f, b))
                        DrawRectangleV({x - 2, y - 2}, {4, 4}, Fade(pal.path, 0.22f));
                }
        }
        // 视锥
        Vector2 c[4];
        for (int i = 0; i < 4; ++i) c[i] = {f.corners[i].x, f.corners[i].y};
        DrawTriangle(c[0], c[2], c[1], Fade(pal.frontier, 0.10f));
        DrawTriangle(c[0], c[3], c[2], Fade(pal.frontier, 0.10f));
        for (int i = 0; i < 4; ++i) DrawLineEx(c[i], c[(i + 1) % 4], 1.4f, pal.text);
        drawDashedLine({eye_.x, eye_.y}, c[0], hl, pal.textMuted);
        drawDashedLine({eye_.x, eye_.y}, c[1], hl, pal.textMuted);

        // 层次剔除访问过的节点
        if (showBvh_) drawNodes(f, 0, 15, hl, green);

        int fp = 0;
        for (const Aabb& b : boxes_) {
            Cull k = classify(f, b);
            bool ex = overlapsExact(f, b);
            Rectangle r{b.min.x, b.min.y, b.max.x - b.min.x, b.max.y - b.min.y};
            if (k != Cull::Outside && !ex) {
                ++fp;
                if (exact_) DrawRectangleLinesEx(r, 1.2f, Fade(pal.path, 0.6f));
                else DrawRectangleRec(r, Fade(pal.path, 0.75f)), DrawRectangleLinesEx(r, 1.2f, pal.path);
                continue;
            }
            if (k == Cull::Outside) DrawRectangleLinesEx(r, hl, Fade(pal.textMuted, 0.45f));
            else if (k == Cull::Inside) DrawRectangleRec(r, Fade(pal.text, 0.8f));
            else DrawRectangleLinesEx(r, 1.4f, pal.text);
        }
        DrawCircleV({eye_.x, eye_.y}, 5, pal.text);

        CullStats sb, st;
        int vis = 0;
        cullBrute(f, boxes_, [&](int) { ++vis; }, sb);
        cullBvh(bvh_, f, boxes_, true, [](int) {}, st);
        char buf[160];
        std::snprintf(buf, sizeof buf, "drawn %d   false positives %d   plane tests: one by one %ld, bvh %ld (%ld nodes)",
                      exact_ ? vis - fp : vis, fp, sb.planeTests, st.planeTests, st.nodes);
        drawText(buf, {12, kTop + 8}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 10, 70, 26}, "planes", !exact_)) exact_ = false;
        if (ui::button({86, 10, 70, 26}, "exact", exact_)) exact_ = true;
        ui::toggle({168, 10, 66, 26}, "bvh", &showBvh_);
        ui::toggle({240, 10, 66, 26}, "spin", &spin_);
        ui::slider({318, 10, 230, 26}, "fov", &fovDeg_, 20, 140, "%.0f°", 1);
        ui::slider({560, 10, 228, 26}, "far", &far_, 80, 520, "%.0f", 1);
        ui::toggle({12, 44, 86, 26}, "fp zone", &zone_);
        ui::slider({110, 44, 230, 26}, "box", &zoneBox_, 2, 60, "%.0f", 1);
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "fov %.0f° · far %.0f", fovDeg_, far_);
        return buf;
    }

private:
    void drawNodes(const Frustum& f, int id, unsigned mask, float hl, Color green) const {
        const Bvh::Node& n = bvh_.nodes[id];
        Rectangle r{n.box.min.x, n.box.min.y, n.box.max.x - n.box.min.x, n.box.max.y - n.box.min.y};
        long t = 0;
        Cull k = classify(f, n.box, mask, t);
        const Palette& pal = palette();
        if (k == Cull::Outside) {
            Vector2 a{r.x, r.y}, b{r.x + r.width, r.y}, c{r.x + r.width, r.y + r.height}, d{r.x, r.y + r.height};
            Color m = Fade(pal.textMuted, 0.7f);
            drawDashedLine(a, b, hl, m, 3, 3), drawDashedLine(b, c, hl, m, 3, 3);
            drawDashedLine(c, d, hl, m, 3, 3), drawDashedLine(d, a, hl, m, 3, 3);
            return;
        }
        if (mask == 0) {
            DrawRectangleLinesEx(r, 1.6f, green);
            return;
        }
        if (n.leaf()) return;
        drawNodes(f, n.left, mask, hl, green);
        drawNodes(f, n.right, mask, hl, green);
    }

    std::vector<Aabb> boxes_;
    Bvh bvh_;
    Vec2 eye_{150, 330};
    float heading_ = 0, fovDeg_ = 70, far_ = 380, zoneBox_ = 30;
    bool exact_ = false, showBvh_ = false, spin_ = true, zone_ = true;
};

}  // namespace

int main() {
    static FrustumDemo demo;
    ga::DemoConfig config;
    config.title = "Frustum culling";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
