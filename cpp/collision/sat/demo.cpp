// 分离轴定理演示
//   - 拖动形状移动，拖动空心小圆旋转；按钮切换形状 B（盒子 / 三角形 / 六边形 / 圆）
//   - 每一步检查一条候选轴：画出这条轴（边法线）、两个形状在轴上的投影，以及它们是否重叠
//   - 找到分离轴就立即停止；所有轴都重叠时，取推出距离最小的轴作为最小分离向量，并画出接触点

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "sat.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr float kPanelX = 640;  // 右侧轴列表

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }

struct Body {
    ConvexPolygon local;  // 局部坐标；isCircle 时不用
    bool isCircle = false;
    float radius = 0;     // 圆的半径；多边形则是外接圆半径（用于放置旋转柄）
    Vec2 pos;
    float angle = 0;

    ConvexPolygon world() const { return transformed(local, pos, angle); }
    Circle circle() const { return {pos, radius}; }
    Vec2 handle() const { return pos + fromAngle(angle) * (radius + 20); }
};

// 一条候选轴：来自 A 的边、B 的边，或者（B 是圆时）离圆心最近的顶点
struct Candidate {
    Vec2 axis;
    char owner;  // 'A' / 'B' / 'v'
    int edge;
};

class SatDemo final : public Demo {
public:
    SatDemo() {
        a_.local = makePolygon({{-60, -40}, {30, -62}, {72, 0}, {28, 58}, {-58, 38}});
        a_.radius = 72;
        a_.pos = {250, 300};
        a_.angle = 0.15f;
        b_.pos = {330, 210};
        b_.angle = 0.7f;
        setKind(0);
    }

    void reset() override {
        k_ = 0;
        done_ = false;
        cands_.clear();
        ConvexPolygon pa = a_.world();
        for (int i = 0; i < pa.size(); ++i) cands_.push_back({pa.normal(i), 'A', i});
        if (b_.isCircle) {
            cands_.push_back({closestVertexAxis(pa, b_.pos), 'v', closestVertex(pa, b_.pos)});
        } else {
            ConvexPolygon pb = b_.world();
            for (int i = 0; i < pb.size(); ++i) cands_.push_back({pb.normal(i), 'B', i});
        }
    }

    // 每一步检查一条轴；找到分离轴，或者所有轴都检查完，就结束
    bool step() override {
        if (done_) return false;
        ++k_;
        if (test(k_ - 1).push <= 0 || k_ == static_cast<int>(cands_.size())) done_ = true;
        return !done_;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 p{std::clamp(m.x, 0.0f, kPanelX - 10), std::clamp(m.y, kTop + 4, float(kHeight))};
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop && m.x < kPanelX) {
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
        const float hl = hairline();
        for (int x = 0; x <= kPanelX; x += 40) DrawLineEx({float(x), kTop}, {float(x), float(kHeight)}, hl, pal.gridLine);
        for (int y = int(kTop); y <= kHeight; y += 40) DrawLineEx({0, float(y)}, {kPanelX, float(y)}, hl, pal.gridLine);

        bool hit = done_ && test(k_ - 1).push > 0;  // 检查完所有轴且都重叠
        int shown = hit ? bestAxis() : k_ - 1;      // 结束且相交时，展示推出距离最小的那条轴

        drawBody(a_, "A", hit, pal.text);
        drawBody(b_, "B", hit, pal.frontier);
        if (shown >= 0) drawAxis(shown, !hit);  // 相交时只画最小分离向量，不画边法线
        if (hit) drawContact();

        drawPanel(shown, hit);
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        static const char* kNames[] = {"box", "tri", "hex", "circle"};
        for (int i = 0; i < 4; ++i) {
            if (ui::button({16 + 66.0f * i, 13, 60, 26}, kNames[i], kind_ == i) && kind_ != i) {
                setKind(i);
                notifyProblemChanged();
            }
        }
    }

    std::string status() const override {
        char buf[96];
        int n = static_cast<int>(cands_.size());
        if (k_ == 0) {
            std::snprintf(buf, sizeof buf, "axes 0/%d", n);
        } else if (!done_) {
            std::snprintf(buf, sizeof buf, "axis %d/%d · overlap", k_, n);
        } else if (test(k_ - 1).push <= 0) {
            std::snprintf(buf, sizeof buf, "separated by %s · %d/%d axes", name(k_ - 1).c_str(), k_, n);
        } else {
            std::snprintf(buf, sizeof buf, "overlap · depth %.0f · %d contacts", contactInfo().depth, contactInfo().count);
        }
        return buf;
    }

private:
    enum class Drag { None, MoveA, MoveB, RotateA, RotateB };

    void setKind(int k) {
        kind_ = k;
        b_.isCircle = k == 3;
        switch (k) {
            case 0: b_.local = makeBox({0, 0}, {78, 34}), b_.radius = 85; break;
            case 1: b_.local = regularPolygon({0, 0}, 70, 3), b_.radius = 70; break;
            case 2: b_.local = regularPolygon({0, 0}, 58, 6), b_.radius = 58; break;
            default: b_.local = {}, b_.radius = 52; break;
        }
    }

    static int closestVertex(const ConvexPolygon& p, Vec2 c) {
        int best = 0;
        for (int i = 1; i < p.size(); ++i)
            if (lengthSq(p.v[i] - c) < lengthSq(p.v[best] - c)) best = i;
        return best;
    }
    static bool inside(const Body& b, Vec2 p) { return b.isCircle ? contains(b.circle(), p) : contains(b.world(), p); }

    Interval projectA(Vec2 u) const { return project(a_.world(), u); }
    Interval projectB(Vec2 u) const { return b_.isCircle ? project(b_.circle(), u) : project(b_.world(), u); }
    AxisTest test(int i) const {
        Vec2 u = cands_[i].axis;
        return testAxis(projectA(u), projectB(u), u);
    }
    int bestAxis() const {
        int best = 0;
        for (int i = 1; i < k_; ++i)
            if (test(i).push < test(best).push) best = i;
        return best;
    }
    std::string name(int i) const {
        const Candidate& c = cands_[i];
        if (c.owner == 'v') return "v->C";
        return std::string(1, c.owner) + std::to_string(c.edge);
    }

    // 接触信息：多边形用参考边裁剪得到 1~2 个点，圆只有一个点
    Manifold contactInfo() const {
        ConvexPolygon pa = a_.world();
        if (b_.isCircle) {
            auto c = contact(pa, b_.circle());
            if (!c) return {};
            return Manifold{c->normal, c->depth, 1, {c->point}, {c->depth}};
        }
        auto m = collide(pa, b_.world());
        return m ? *m : Manifold{};
    }

    void drawBody(const Body& b, const char* label, bool hot, Color ink) const {
        const Palette& pal = palette();
        Color line = hot ? pal.path : ink;
        Color fill = hot ? Fade(pal.path, 0.12f) : Fade(ink, 0.08f);
        if (b.isCircle) {
            DrawCircleV(rl(b.pos), b.radius, fill);
            DrawRing(rl(b.pos), b.radius - 1.5f, b.radius, 0, 360, 64, line);
        } else {
            ConvexPolygon w = b.world();
            for (int i = 1; i + 1 < w.size(); ++i) {
                // raylib 的 DrawTriangle 要求逆时针（以 y 朝下的屏幕为准），所以反过来传
                DrawTriangle(rl(w.v[0]), rl(w.v[i + 1]), rl(w.v[i]), fill);
            }
            for (int i = 0; i < w.size(); ++i) DrawLineEx(rl(w.vertex(i)), rl(w.vertex(i + 1)), 1.5f, line);
        }
        drawTextCentered(label, rl(b.pos), 16, line);
        // 旋转柄
        DrawLineEx(rl(b.pos + fromAngle(b.angle) * (b.radius + 4)), rl(b.handle()), hairline() * 2, Fade(ink, 0.5f));
        DrawCircleV(rl(b.handle()), 6, pal.background);
        DrawCircleLinesV(rl(b.handle()), 6, ink);
    }

    // 画一条候选轴：它来自哪条边、两个投影区间、重叠或间隙
    void drawAxis(int i, bool showNormal) const {
        const Palette& pal = palette();
        const Candidate& cand = cands_[i];
        Vec2 u = cand.axis, side = perp(u);
        ConvexPolygon pa = a_.world();

        // 1. 在形状上标出这条轴的来源
        if (cand.owner == 'v') {
            Vec2 v = pa.v[cand.edge];
            drawDashedLine(rl(v), rl(b_.pos), 1.5f, pal.path, 5, 4);
            DrawCircleV(rl(v), 4.5f, pal.path);
        } else {
            ConvexPolygon w = cand.owner == 'A' ? pa : b_.world();
            Vec2 e0 = w.vertex(cand.edge), e1 = w.vertex(cand.edge + 1), mid = (e0 + e1) * 0.5f;
            DrawLineEx(rl(e0), rl(e1), 4, pal.path);
            if (showNormal) drawArrow(rl(mid), rl(mid + u * 30), 2, pal.path, 8);
        }

        // 2. 把投影画在一条与轴平行、放在两个形状旁边的直线上
        std::vector<Vec2> pts = pa.v;
        if (b_.isCircle) pts.push_back(b_.pos + side * b_.radius), pts.push_back(b_.pos - side * b_.radius);
        else for (Vec2 q : b_.world().v) pts.push_back(q);
        Vec2 c = (a_.pos + b_.pos) * 0.5f;
        float sMin = 1e9f, sMax = -1e9f;
        for (Vec2 q : pts) sMin = std::min(sMin, dot(q - c, side)), sMax = std::max(sMax, dot(q - c, side));
        // 选离画布中心更近的一侧
        auto outside = [&](float off) {
            Vec2 q = c + side * off;
            return std::max({20 - q.x, q.x - (kPanelX - 20), kTop + 30 - q.y, q.y - (kHeight - 20), 0.0f});
        };
        float off = outside(sMax + 46) <= outside(sMin - 46) ? sMax + 46 : sMin - 46;
        Vec2 base = c + side * off;
        float c0 = dot(c, u);
        auto at = [&](float s, float lift) { return base + u * (s - c0) + side * lift; };

        Interval ia = projectA(u), ib = projectB(u);
        float lo = std::min(ia.min, ib.min) - 40, hi = std::max(ia.max, ib.max) + 40;
        drawArrow(rl(at(lo, 0)), rl(at(hi, 0)), hairline() * 2, pal.textMuted, 8);

        // 投影线：从形状的极端点垂直落到轴上
        auto drop = [&](Vec2 q, Color col) {
            float s = dot(q, u);
            drawDashedLine(rl(q), rl(at(s, 0)), hairline() * 1.5f, Fade(col, 0.45f), 4, 4);
        };
        auto extremes = [&](const std::vector<Vec2>& vs, Color col) {
            Vec2 qmin = vs[0], qmax = vs[0];
            for (Vec2 q : vs) {
                if (dot(q, u) < dot(qmin, u)) qmin = q;
                if (dot(q, u) > dot(qmax, u)) qmax = q;
            }
            drop(qmin, col);
            drop(qmax, col);
        };
        extremes(pa.v, pal.text);
        if (b_.isCircle) extremes({b_.pos - u * b_.radius, b_.pos + u * b_.radius}, pal.frontier);
        else extremes(b_.world().v, pal.frontier);

        float lift = off > 0 ? 1.0f : -1.0f;  // 区间画在轴线远离形状的一侧
        DrawLineEx(rl(at(ia.min, 5 * lift)), rl(at(ia.max, 5 * lift)), 5, pal.text);
        DrawLineEx(rl(at(ib.min, 11 * lift)), rl(at(ib.max, 11 * lift)), 5, pal.frontier);

        AxisTest t = testAxis(ia, ib, u);
        char buf[32];
        if (t.push <= 0) {
            float g0 = std::min(ia.max, ib.max), g1 = std::max(ia.min, ib.min);
            DrawLineEx(rl(at(g0, 8 * lift)), rl(at(g1, 8 * lift)), 3, pal.path);
            std::snprintf(buf, sizeof buf, "gap %.0f", -t.push);
            drawLabel(buf, at((g0 + g1) / 2, 30 * lift), pal.path);
        } else {
            float o0 = std::max(ia.min, ib.min), o1 = std::min(ia.max, ib.max);
            DrawLineEx(rl(at(o0, 8 * lift)), rl(at(o1, 8 * lift)), 13, Fade(pal.path, 0.25f));
            std::snprintf(buf, sizeof buf, "push %.0f", t.push);
            drawLabel(buf, at((o0 + o1) / 2, 30 * lift), pal.text);
        }
    }

    void drawLabel(const char* s, Vec2 center, Color col) const {
        Vector2 sz = measureText(s, 12);
        DrawRectangleRec({center.x - sz.x / 2 - 3, center.y - sz.y / 2 - 1, sz.x + 6, sz.y + 2}, Fade(palette().background, 0.85f));
        drawTextCentered(s, rl(center), 12, col);
    }

    void drawContact() const {
        const Palette& pal = palette();
        Manifold m = contactInfo();
        if (m.count == 0) return;
        Vec2 mid{0, 0};
        for (int i = 0; i < m.count; ++i) mid += m.points[i];
        mid = mid / float(m.count);
        drawArrow(rl(mid), rl(mid + m.normal * std::max(m.depth, 12.0f)), 2.5f, pal.path, 9);
        for (int i = 0; i < m.count; ++i) {
            DrawCircleV(rl(m.points[i]), 5, pal.path);
            DrawCircleV(rl(m.points[i]), 2, pal.background);
        }
    }

    void drawPanel(int shown, bool hit) const {
        const Palette& pal = palette();
        DrawLineEx({kPanelX, kTop}, {kPanelX, float(kHeight)}, hairline(), pal.gridLine);
        drawText("axis   result", {kPanelX + 14, kTop + 14}, 12, pal.textMuted);
        float y = kTop + 36;
        for (int i = 0; i < static_cast<int>(cands_.size()); ++i, y += 20) {
            std::string label = name(i);
            Color col = i < k_ ? pal.text : Fade(pal.textMuted, 0.6f);
            if (i == shown) DrawRectangleRec({kPanelX + 6, y - 3, kWidth - kPanelX - 12, 19}, Fade(pal.path, 0.1f));
            drawText(label.c_str(), {kPanelX + 14, y}, 12, col);
            if (i >= k_) {
                drawText("-", {kPanelX + 70, y}, 12, col);
                continue;
            }
            AxisTest t = test(i);
            char buf[32];
            if (t.push <= 0) std::snprintf(buf, sizeof buf, "gap %.0f", -t.push);
            else std::snprintf(buf, sizeof buf, hit && i == shown ? "push %.0f min" : "push %.0f", t.push);
            drawText(buf, {kPanelX + 70, y}, 12, t.push <= 0 || (hit && i == shown) ? pal.path : col);
        }
    }

    Body a_, b_;
    int kind_ = 0;
    std::vector<Candidate> cands_;
    int k_ = 0;          // 已经检查了几条轴
    bool done_ = false;
    Drag drag_ = Drag::None;
    Vec2 grab_{0, 0};
};

}  // namespace

int main() {
    static SatDemo demo;
    ga::DemoConfig config;
    config.title = "Separating Axis Theorem";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 1.5f;
    return ga::runDemo(demo, config);
}
