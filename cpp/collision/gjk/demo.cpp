// GJK 与 EPA 演示
//   - 左边是两个形状，右边是它们的闵可夫斯基差 A − B（缩小到 3/4 显示），十字是原点
//   - GJK 每一步：沿搜索方向（朱红箭头）取一个支撑点，加入单纯形，再收缩到离原点最近的部分
//   - 单纯形包住原点 → 相交，接着用 EPA 向外扩张多边形，找到离原点最近的边 = 穿透深度与方向
//   - 拖动形状移动，拖动空心小圆旋转；按钮切换形状 B，round 给 A 加上圆角

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "demo.h"
#include "gjk.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr float kSplit = 400;                 // 左：世界；右：闵可夫斯基差
constexpr Vec2 kOrigin{600, 290};             // 右侧面板中原点的位置
constexpr float kScale = 0.75f;

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }
inline Vec2 toDiff(Vec2 p) { return kOrigin + p * kScale; }

struct Body {
    ConvexShape local;
    float extent = 0;  // 用于放置旋转柄
    Vec2 pos;
    float angle = 0;

    ConvexShape world() const {
        ConvexShape s{{}, local.radius};
        for (Vec2 p : local.points) s.points.push_back(pos + rotate(p, angle));
        return s;
    }
    Vec2 handle() const { return pos + fromAngle(angle) * (extent + 18); }
};

// 沿一圈方向取支撑点，得到凸形状的轮廓（对圆角、圆、胶囊体都适用）
template <class F>
std::vector<Vec2> outline(F&& supportFn, int n = 96) {
    std::vector<Vec2> pts;
    for (int i = 0; i < n; ++i) pts.push_back(supportFn(fromAngle(2 * kPi * i / n)));
    return pts;
}

void fillConvex(const std::vector<Vec2>& pts, Color fill, Color line, float thick) {
    Vec2 c{0, 0};
    for (Vec2 p : pts) c += p;
    c = c / float(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        Vec2 p = pts[i], q = pts[(i + 1) % pts.size()];
        DrawTriangle(rl(c), rl(q), rl(p), fill);  // 轮廓按屏幕顺时针排列，raylib 要逆时针
    }
    for (size_t i = 0; i < pts.size(); ++i) DrawLineEx(rl(pts[i]), rl(pts[(i + 1) % pts.size()]), thick, line);
}

class GjkDemo final : public Demo {
public:
    GjkDemo() {
        a_.local.points = {{-45, -30}, {22, -46}, {54, 0}, {21, 43}, {-43, 28}};
        a_.extent = 56;
        a_.pos = {150, 300};
        a_.angle = 0.2f;
        b_.pos = {210, 245};
        b_.angle = -0.29f;  // 盒子转半圈形状不变，这样旋转柄朝外
        setKind(0);
    }

    void reset() override {
        wa_ = a_.world();
        wb_ = b_.world();
        epa_.reset();
        gjk_.emplace(wa_, wb_);
        lastWasGjk_ = false;
    }

    // GJK 一步一次迭代；相交后切换到 EPA，一步扩张一次
    bool step() override {
        if (gjk_->status() == Gjk::Status::Running) {
            lastWasGjk_ = true;
            Gjk::Status s = gjk_->step();
            if (s == Gjk::Status::Intersecting) epa_.emplace(wa_, wb_, gjk_->simplex());
            return s != Gjk::Status::Separated;
        }
        lastWasGjk_ = false;
        return epa_ && epa_->step();
    }

    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 p{std::clamp(m.x, 0.0f, kSplit - 8), std::clamp(m.y, kTop + 4, float(kHeight))};
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop && m.x < kSplit) {
            drag_ = Drag::None;
            if (distance(p, b_.handle()) < 10) drag_ = Drag::RotateB;
            else if (distance(p, a_.handle()) < 10) drag_ = Drag::RotateA;
            else if (inside(b_, p)) drag_ = Drag::MoveB, grab_ = b_.pos - p;
            else if (inside(a_, p)) drag_ = Drag::MoveA, grab_ = a_.pos - p;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = Drag::None;
        if (drag_ == Drag::None) return;
        Body& body = (drag_ == Drag::MoveA || drag_ == Drag::RotateA) ? a_ : b_;
        if (drag_ == Drag::MoveA || drag_ == Drag::MoveB) body.pos = p + grab_;
        else body.angle = angleOf(p - body.pos);
        notifyProblemChanged();
    }

    void draw() const override {
        const Palette& pal = palette();
        drawDiffPanel();
        // 左侧面板盖住超出范围的闵可夫斯基差
        DrawRectangleRec({0, kTop, kSplit, kHeight - kTop}, pal.background);
        const float hl = hairline();
        for (int x = 0; x <= kSplit; x += 40) DrawLineEx({float(x), kTop}, {float(x), float(kHeight)}, hl, pal.gridLine);
        for (int y = int(kTop); y <= kHeight; y += 40) DrawLineEx({0, float(y)}, {kSplit, float(y)}, hl, pal.gridLine);
        DrawLineEx({kSplit, kTop}, {kSplit, float(kHeight)}, hl * 2, pal.textMuted);
        drawWorldPanel();
        drawText("world", {12, kTop + 10}, 12, pal.textMuted);
        drawText("A - B", {kSplit + 12, kTop + 10}, 12, pal.textMuted);

        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        static const char* kNames[] = {"box", "tri", "circle", "capsule"};
        for (int i = 0; i < 4; ++i) {
            if (ui::button({16 + 72.0f * i, 13, 66, 26}, kNames[i], kind_ == i) && kind_ != i) {
                setKind(i);
                notifyProblemChanged();
            }
        }
        bool round = a_.local.radius > 0;
        if (ui::toggle({320, 13, 70, 26}, "round", &round)) {
            a_.local.radius = round ? 14 : 0;
            notifyProblemChanged();
        }
    }

    std::string status() const override {
        char buf[96];
        if (gjk_->status() == Gjk::Status::Running) {
            std::snprintf(buf, sizeof buf, "GJK iter %d · simplex %d", gjk_->iterations(), gjk_->simplex().count);
        } else if (gjk_->status() == Gjk::Status::Separated) {
            std::snprintf(buf, sizeof buf, "separated · distance %.1f · %d iters", gjk_->distance(), gjk_->iterations());
        } else if (!epa_->done()) {
            std::snprintf(buf, sizeof buf, "EPA iter %d · depth %.1f", epa_->iterations(), epa_->depth());
        } else {
            std::snprintf(buf, sizeof buf, "overlap · depth %.1f · GJK %d + EPA %d", epa_->depth(), gjk_->iterations(),
                          epa_->iterations());
        }
        return buf;
    }

private:
    enum class Drag { None, MoveA, MoveB, RotateA, RotateB };

    void setKind(int k) {
        kind_ = k;
        switch (k) {
            case 0: b_.local = {{{-55, -26}, {55, -26}, {55, 26}, {-55, 26}}, 0}, b_.extent = 61; break;
            case 1: b_.local = {{{56, 0}, {-28, 48.5f}, {-28, -48.5f}}, 0}, b_.extent = 56; break;
            case 2: b_.local = {{{0, 0}}, 42}, b_.extent = 42; break;
            default: b_.local = {{{-42, 0}, {42, 0}}, 22}, b_.extent = 64; break;
        }
    }

    static bool inside(const Body& b, Vec2 p) {
        ConvexShape w = b.world();
        ConvexShape probe{{p}, 0};
        return overlaps(w, probe);
    }

    void drawWorldPanel() const {
        const Palette& pal = palette();
        bool hit = gjk_->status() == Gjk::Status::Intersecting;
        bool finished = gjk_->status() == Gjk::Status::Separated || (epa_ && epa_->done());
        drawBody(a_, wa_, "A", pal.text, hit && finished);
        drawBody(b_, wb_, "B", pal.frontier, hit && finished);

        if (!finished && lastWasGjk_) {
            // 刚取的支撑点：A 上沿 d 最远的点，B 上沿 −d 最远的点
            const SupportPoint& s = gjk_->lastSupport();
            Vec2 d = normalized(gjk_->direction());
            drawArrow(rl(s.a), rl(s.a + d * 28), 2, pal.path, 7);
            drawArrow(rl(s.b), rl(s.b - d * 28), 2, pal.path, 7);
            DrawCircleV(rl(s.a), 4.5f, pal.path);
            DrawCircleV(rl(s.b), 4.5f, pal.path);
        }
        if (!finished) return;
        char buf[32];
        if (!hit) {  // 最近点与距离
            Vec2 pa = gjk_->pointA(), pb = gjk_->pointB();
            drawDashedLine(rl(pa), rl(pb), 1.5f, pal.path, 5, 4);
            DrawCircleV(rl(pa), 4.5f, pal.path);
            DrawCircleV(rl(pb), 4.5f, pal.path);
            std::snprintf(buf, sizeof buf, "%.1f", gjk_->distance());
            drawText(buf, rl((pa + pb) * 0.5f + Vec2{8, -6}), 12, pal.path);
        } else {     // 把 B 从 pointB 推到 pointA 就恰好分开
            Vec2 pa = epa_->pointA(), pb = epa_->pointB(), n = epa_->normal();
            Vec2 tip = epa_->depth() > 10 ? pa : pb + n * 10;
            drawArrow(rl(pb), rl(tip), 2.5f, pal.path, 9);
            DrawCircleV(rl(pa), 4, pal.path);
            DrawCircleV(rl(pb), 4, pal.path);
            std::snprintf(buf, sizeof buf, "%.1f", epa_->depth());
            drawText(buf, rl(tip + n * 8 + Vec2{4, -6}), 12, pal.path);
        }
    }

    void drawBody(const Body& b, const ConvexShape& w, const char* label, Color ink, bool hot) const {
        const Palette& pal = palette();
        Color line = hot ? pal.path : ink;
        fillConvex(outline([&](Vec2 d) { return support(w, d); }), hot ? Fade(pal.path, 0.12f) : Fade(ink, 0.08f), line, 1.5f);
        if (w.radius > 0 && w.points.size() > 1) {  // 圆角形状：用虚线画出"核心"
            for (size_t i = 0; i < w.points.size(); ++i) {
                if (w.points.size() == 2 && i == 1) break;
                drawDashedLine(rl(w.points[i]), rl(w.points[(i + 1) % w.points.size()]), hairline() * 1.5f, Fade(line, 0.5f), 4, 4);
            }
        }
        drawTextCentered(label, rl(b.pos), 16, line);
        DrawLineEx(rl(b.pos + fromAngle(b.angle) * (b.extent + 4)), rl(b.handle()), hairline() * 2, Fade(ink, 0.5f));
        DrawCircleV(rl(b.handle()), 6, pal.background);
        DrawCircleLinesV(rl(b.handle()), 6, ink);
    }

    void drawDiffPanel() const {
        const Palette& pal = palette();
        const float hl = hairline();
        for (int x = int(kSplit); x <= kWidth; x += 40) DrawLineEx({float(x), kTop}, {float(x), float(kHeight)}, hl, pal.gridLine);
        for (int y = int(kTop); y <= kHeight; y += 40) DrawLineEx({kSplit, float(y)}, {float(kWidth), float(y)}, hl, pal.gridLine);

        // A − B 本身：通过支撑函数采样出来，从来不需要真的构造它
        auto diff = outline([&](Vec2 d) { return toDiff(support(wa_, wb_, d).p); }, 128);
        fillConvex(diff, Fade(pal.text, 0.07f), Fade(pal.text, 0.55f), 1.2f);

        // 原点
        DrawLineEx(rl(kOrigin - Vec2{9, 0}), rl(kOrigin + Vec2{9, 0}), 1.5f, pal.text);
        DrawLineEx(rl(kOrigin - Vec2{0, 9}), rl(kOrigin + Vec2{0, 9}), 1.5f, pal.text);
        drawText("O", rl(kOrigin + Vec2{6, 4}), 12, pal.text);

        char buf[32];
        if (epa_) {
            const auto& poly = epa_->polytope();
            int n = static_cast<int>(poly.size());
            for (int i = 0; i < n; ++i) DrawLineEx(rl(toDiff(poly[i].p)), rl(toDiff(poly[(i + 1) % n].p)), 1.5f, pal.frontier);
            for (const auto& v : poly) DrawCircleV(rl(toDiff(v.p)), 3, pal.frontier);
            int e = epa_->closestEdge();
            DrawLineEx(rl(toDiff(poly[e].p)), rl(toDiff(poly[(e + 1) % n].p)), 3.5f, pal.path);
            Vec2 tip = toDiff(epa_->normal() * epa_->depth());
            drawArrow(rl(kOrigin), rl(tip), 2, pal.path, 8);
            std::snprintf(buf, sizeof buf, epa_->done() ? "depth %.1f" : "%.1f", epa_->depth());
            drawText(buf, rl(tip + epa_->normal() * 10 + Vec2{4, -6}), 12, pal.path);
            return;
        }

        // GJK 的单纯形
        const Simplex& s = gjk_->simplex();
        Vec2 v[3];
        for (int i = 0; i < s.count; ++i) v[i] = toDiff(s.v[i].p);
        if (s.count == 3) {
            DrawTriangle(rl(v[0]), rl(v[2]), rl(v[1]), Fade(pal.path, 0.2f));
            DrawTriangle(rl(v[0]), rl(v[1]), rl(v[2]), Fade(pal.path, 0.2f));  // 不管朝向，总有一个会画出来
        }
        for (int i = 0; i < s.count; ++i)
            for (int j = i + 1; j < s.count; ++j) DrawLineEx(rl(v[i]), rl(v[j]), 2.5f, pal.path);
        for (int i = 0; i < s.count; ++i) DrawCircleV(rl(v[i]), 4.5f, pal.path);
        if (lastWasGjk_) {
            DrawCircleLinesV(rl(toDiff(gjk_->lastSupport().p)), 8, pal.path);
        }

        if (gjk_->status() == Gjk::Status::Intersecting) return;
        Vec2 c = toDiff(s.closest());
        drawDashedLine(rl(c), rl(kOrigin), 1.2f, Fade(pal.path, 0.7f), 4, 4);
        if (gjk_->status() == Gjk::Status::Running) {
            // 下一步的搜索方向：从最近点指向原点
            Vec2 d = normalized(kOrigin - c);
            drawArrow(rl(c), rl(c + d * 36), 2, pal.path, 8);
        } else {
            std::snprintf(buf, sizeof buf, "distance %.1f", gjk_->distance());
            drawText(buf, rl(kOrigin + Vec2{14, -22}), 12, pal.path);
        }
    }

    Body a_, b_;
    int kind_ = 0;
    ConvexShape wa_, wb_;  // 当前一轮查询使用的世界坐标形状（Gjk / Epa 持有它们的指针）
    std::optional<Gjk> gjk_;
    std::optional<Epa> epa_;
    bool lastWasGjk_ = false;
    Drag drag_ = Drag::None;
    Vec2 grab_{0, 0};
};

}  // namespace

int main() {
    static GjkDemo demo;
    ga::DemoConfig config;
    config.title = "GJK & EPA";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 1.2f;
    return ga::runDemo(demo, config);
}
