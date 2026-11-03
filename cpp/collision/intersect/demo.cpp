// 基础相交测试演示
//   - 拖动盒子 A、B 和圆 C、D；重叠的形状描红，箭头是把后一个形状推开的最小分离向量（MTV）
//   - 拖动射线的起点（实心圆）和方向柄（空心圆）；射线停在第一个命中点，短箭头是命中面的法线
//   - slabs：显示盒子 A 的两条"板"，以及射线在每条板内的参数区间
//   - separate：沿 MTV 把所有重叠的形状推开

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "intersect.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }

class IntersectDemo final : public Demo {
public:
    IntersectDemo() { reset(); }

    void reset() override {
        boxes_[0] = {{150, 130}, {320, 240}};
        boxes_[1] = {{270, 200}, {400, 330}};
        circles_[0] = {{560, 180}, 62};
        circles_[1] = {{640, 290}, 48};
        rayOrigin_ = {70, 420};
        rayTip_ = {250, 360};
    }
    bool step() override { return false; }

    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 p{m.x, std::max(m.y, kTop + 4)};
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop) {
            grabOffset_ = {0, 0};
            if (distance(p, rayOrigin_) < 12) drag_ = Drag::RayOrigin;
            else if (distance(p, rayTip_) < 12) drag_ = Drag::RayTip;
            else {
                drag_ = Drag::None;
                for (int i = 1; i >= 0 && drag_ == Drag::None; --i) {
                    if (contains(circles_[i], p)) {
                        drag_ = i == 0 ? Drag::Circle0 : Drag::Circle1;
                        grabOffset_ = circles_[i].center - p;
                    }
                }
                for (int i = 1; i >= 0 && drag_ == Drag::None; --i) {
                    if (contains(boxes_[i], p)) {
                        drag_ = i == 0 ? Drag::Box0 : Drag::Box1;
                        grabOffset_ = boxes_[i].min - p;
                    }
                }
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = Drag::None;
        switch (drag_) {
            case Drag::RayOrigin: rayOrigin_ = p; break;
            case Drag::RayTip: rayTip_ = p; break;
            case Drag::Circle0: circles_[0].center = p + grabOffset_; break;
            case Drag::Circle1: circles_[1].center = p + grabOffset_; break;
            case Drag::Box0:
            case Drag::Box1: {
                Aabb& b = boxes_[drag_ == Drag::Box0 ? 0 : 1];
                Vec2 size = b.max - b.min;
                b.min = p + grabOffset_;
                b.max = b.min + size;
                break;
            }
            case Drag::None: break;
        }
    }

    void draw() const override {
        const Palette& p = palette();
        const float hl = hairline();
        for (int x = 0; x <= kWidth; x += 40) DrawLineEx({float(x), kTop}, {float(x), float(kHeight)}, hl, p.gridLine);
        for (int y = int(kTop); y <= kHeight; y += 40) DrawLineEx({0, float(y)}, {float(kWidth), float(y)}, hl, p.gridLine);

        ga::Ray ray{rayOrigin_, rayTip_ - rayOrigin_};
        if (showSlabs_) drawSlabs(ray);

        // 形状：与任何形状重叠时描红
        bool hot[4] = {};
        auto contacts = allContacts(hot);
        for (int i = 0; i < 2; ++i) drawBox(boxes_[i], i == 0 ? "A" : "B", hot[i]);
        for (int i = 0; i < 2; ++i) drawCircle(circles_[i], i == 0 ? "C" : "D", hot[2 + i]);

        for (const Contact& c : contacts) {
            drawArrow(rl(c.point), rl(c.point + c.normal * std::max(c.depth, 10.0f)), 2, p.path, 8);
            char buf[16];
            std::snprintf(buf, sizeof buf, "%.0f", c.depth);
            drawText(buf, rl(c.point + c.normal * (std::max(c.depth, 10.0f) + 6) + Vec2{4, -14}), 12, p.path);
        }

        // 射线：画到第一个命中点为止，之后用虚线延伸
        auto hit = firstHit(ray);
        Vec2 dir = normalized(ray.dir);
        Vec2 far = rayOrigin_ + dir * 2000;
        if (hit) {
            DrawLineEx(rl(rayOrigin_), rl(hit->point), 2, p.text);
            drawDashedLine(rl(hit->point), rl(far), 1.2f, Fade(p.text, 0.35f), 6, 5);
            DrawCircleV(rl(hit->point), 5, p.path);
            if (hit->normal != Vec2{0, 0}) drawArrow(rl(hit->point), rl(hit->point + hit->normal * 34), 2, p.path, 8);
            char buf[24];
            std::snprintf(buf, sizeof buf, "t = %.2f", hit->t);
            drawText(buf, rl(hit->point + Vec2{10, 8}), 12, p.text);
        } else {
            drawDashedLine(rl(rayOrigin_), rl(far), 1.2f, Fade(p.text, 0.5f), 6, 5);
        }
        drawArrow(rl(rayOrigin_), rl(rayTip_), 1.2f, Fade(p.text, 0.6f), 7);
        DrawCircleV(rl(rayOrigin_), 7, p.text);
        DrawCircleV(rl(rayTip_), 6, p.background);
        DrawCircleLinesV(rl(rayTip_), 6, p.text);
        drawText("t = 1", rl(rayTip_ + Vec2{9, -18}), 11, p.textMuted);

        // 顶栏遮住越界的部分
        DrawRectangleRec({0, 0, float(kWidth), kTop}, p.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, p.gridLine);
    }

    void drawUi() override {
        ui::toggle({16, 13, 74, 26}, "slabs", &showSlabs_);
        if (ui::button({100, 13, 92, 26}, "separate")) separate();
    }

    std::string status() const override {
        bool hot[4] = {};
        int pairs = static_cast<int>(allContacts(hot).size());
        ga::Ray ray{rayOrigin_, rayTip_ - rayOrigin_};
        auto hit = firstHit(ray, &hitName_);
        char buf[96];
        if (hit) {
            std::snprintf(buf, sizeof buf, "overlaps %d · ray hit %s, t = %.2f", pairs, hitName_, hit->t);
        } else {
            std::snprintf(buf, sizeof buf, "overlaps %d · ray miss", pairs);
        }
        return buf;
    }

private:
    enum class Drag { None, Box0, Box1, Circle0, Circle1, RayOrigin, RayTip };

    // 所有形状两两之间的接触。hot[i] 标记第 i 个形状（A, B, C, D）是否与别的形状重叠
    std::vector<Contact> allContacts(bool hot[4]) const {
        std::vector<Contact> out;
        auto add = [&](std::optional<Contact> c, int i, int j) {
            if (!c) return;
            out.push_back(*c);
            hot[i] = hot[j] = true;
        };
        add(contact(boxes_[0], boxes_[1]), 0, 1);
        add(contact(circles_[0], circles_[1]), 2, 3);
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) add(contact(boxes_[i], circles_[j]), i, 2 + j);
        return out;
    }

    // 射线与四个形状中最近的命中
    std::optional<RayHit> firstHit(const ga::Ray& ray, const char** name = nullptr) const {
        std::optional<RayHit> best;
        static const char* kNames[] = {"A", "B", "C", "D"};
        auto consider = [&](std::optional<RayHit> h, int i) {
            if (h && (!best || h->t < best->t)) {
                best = h;
                if (name) *name = kNames[i];
            }
        };
        for (int i = 0; i < 2; ++i) consider(raycast(ray, boxes_[i]), i);
        for (int i = 0; i < 2; ++i) consider(raycast(ray, circles_[i]), 2 + i);
        return best;
    }

    // 沿最小分离向量反复推开重叠的形状，两个形状各移动一半
    void separate() {
        for (int iter = 0; iter < 20; ++iter) {
            auto push = [](auto& a, auto& b) {
                auto c = contact(a, b);
                if (!c) return;
                Vec2 d = c->normal * (c->depth * 0.5f + 0.5f);
                move(a, -d);
                move(b, d);
            };
            push(boxes_[0], boxes_[1]);
            push(circles_[0], circles_[1]);
            for (auto& b : boxes_)
                for (auto& c : circles_) push(b, c);
        }
    }
    static void move(Aabb& b, Vec2 d) { b.min += d; b.max += d; }
    static void move(Circle& c, Vec2 d) { c.center += d; }

    void drawSlabs(const ga::Ray& ray) const {
        const Palette& p = palette();
        const Aabb& a = boxes_[0];
        Color band = Fade(p.frontier, 0.16f);
        DrawRectangleRec({a.min.x, kTop, a.max.x - a.min.x, kHeight - kTop}, band);
        DrawRectangleRec({0, a.min.y, float(kWidth), a.max.y - a.min.y}, band);
        // 射线在每条板内的参数区间，画在射线两侧
        Vec2 side = normalized(perp(ray.dir)) * 5.0f;
        for (int axis = 0; axis < 2; ++axis) {
            float o = axis == 0 ? ray.origin.x : ray.origin.y, d = axis == 0 ? ray.dir.x : ray.dir.y;
            float lo = axis == 0 ? a.min.x : a.min.y, hi = axis == 0 ? a.max.x : a.max.y;
            float t1 = 0, t2 = 1e6f;
            if (std::abs(d) > 1e-6f) {
                t1 = (lo - o) / d;
                t2 = (hi - o) / d;
                if (t1 > t2) std::swap(t1, t2);
            } else if (o < lo || o > hi) {
                continue;
            }
            t1 = std::max(t1, 0.0f);
            if (t2 <= t1) continue;
            Vec2 off = axis == 0 ? side : -side;
            DrawLineEx(rl(ray.at(t1) + off), rl(ray.at(std::min(t2, 50.0f)) + off), 4,
                       axis == 0 ? p.frontier : Fade(p.text, 0.55f));
            char buf[24];
            std::snprintf(buf, sizeof buf, "%s [%.2f, %.2f]", axis == 0 ? "x" : "y", t1, t2);
            drawText(buf, rl(ray.at(t1) + off * 3.2f + Vec2{-10, -6}), 11, axis == 0 ? p.frontier : p.textMuted);
        }
    }

    void drawBox(const Aabb& b, const char* label, bool hot) const {
        const Palette& p = palette();
        Rectangle r{b.min.x, b.min.y, b.max.x - b.min.x, b.max.y - b.min.y};
        DrawRectangleRec(r, hot ? Fade(p.path, 0.12f) : Fade(p.text, 0.06f));
        DrawRectangleLinesEx(r, hot ? 2.0f : 1.5f, hot ? p.path : p.text);
        drawTextCentered(label, rl(b.center()), 16, hot ? p.path : p.text);
    }
    void drawCircle(const Circle& c, const char* label, bool hot) const {
        const Palette& p = palette();
        DrawCircleV(rl(c.center), c.radius, hot ? Fade(p.path, 0.12f) : Fade(p.text, 0.06f));
        DrawRing(rl(c.center), c.radius - (hot ? 2.0f : 1.5f), c.radius, 0, 360, 64, hot ? p.path : p.text);
        drawTextCentered(label, rl(c.center), 16, hot ? p.path : p.text);
    }

    Aabb boxes_[2];
    Circle circles_[2];
    Vec2 rayOrigin_, rayTip_;
    Drag drag_ = Drag::None;
    Vec2 grabOffset_{0, 0};
    bool showSlabs_ = false;
    mutable const char* hitName_ = "";
};

}  // namespace

int main() {
    static IntersectDemo demo;
    ga::DemoConfig config;
    config.title = "Intersection Tests";
    config.width = kWidth;
    config.height = kHeight;
    config.autoPlay = false;
    return ga::runDemo(demo, config);
}
