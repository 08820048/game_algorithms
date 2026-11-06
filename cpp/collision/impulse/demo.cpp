// 碰撞响应演示：基于冲量的刚体物理
//   stack  ：10 个盒子叠成一摞。调低迭代次数或关闭热启动，看它怎么倒
//   pyramid：金字塔
//   ramp   ：斜面上摩擦系数不同的三个盒子
//   bounce ：恢复系数不同的五个球
// 点击场地放入新的物体（盒子与球交替）。朱红点是接触点，短线的长度表示法线冲量的大小

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "impulse.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr float kScale = 26;            // 每米多少像素
constexpr float kGroundY = kHeight - 22;  // 地面顶部（y = 0 米）在屏幕上的位置

inline Vector2 toScreen(Vec2 p) { return {kWidth / 2.0f + p.x * kScale, kGroundY - p.y * kScale}; }
inline Vec2 toWorld(Vector2 s) { return {(s.x - kWidth / 2.0f) / kScale, (kGroundY - s.y) / kScale}; }

enum class Scene { Stack, Pyramid, Ramp, Bounce };

class ImpulseDemo final : public Demo {
public:
    ImpulseDemo() { reset(); }

    void reset() override {
        world_ = World();
        world_.opt.iterations = static_cast<int>(iterations_);
        world_.opt.warmStart = warm_;
        auto& b = world_.bodies;
        b.push_back(makeBoxBody({0, -0.5f}, {16, 0.5f}, 0));  // 地面
        labels_.clear();
        switch (scene_) {
            case Scene::Stack:
                for (int i = 0; i < 10; ++i) b.push_back(makeBoxBody({0, 0.5f + i * 1.0f}, {0.5f, 0.5f}, 1));
                break;
            case Scene::Pyramid:
                for (int r = 0; r < 12; ++r)
                    for (int c = 0; c < 12 - r; ++c) b.push_back(makeBoxBody({(c - (11 - r) / 2.0f) * 1.02f, 0.5f + r * 1.0f}, {0.5f, 0.5f}, 1));
                break;
            case Scene::Ramp: {
                float th = 0.45f;  // 约 26°：tan = 0.48
                b.push_back(makeBoxBody({-2, 5}, {9, 0.25f}, 0, -th));
                Vec2 down{std::cos(th), -std::sin(th)}, up{std::sin(th), std::cos(th)};
                float mus[] = {0.9f, 0.45f, 0.1f};  // 最滑的放在最下面，滑走时不会撞到别人
                for (int i = 0; i < 3; ++i) {
                    Vec2 base = Vec2{-2, 5} + down * (-6.0f + i * 2.6f) + up * 0.86f;
                    Body box = makeBoxBody(base, {0.6f, 0.6f}, 1, -th);
                    box.friction = mus[i];
                    b.push_back(box);
                    char buf[16];
                    std::snprintf(buf, sizeof buf, "mu %.2f", mus[i]);
                    labels_.push_back({static_cast<int>(b.size()) - 1, buf});
                }
                b[1].friction = 1;
                break;
            }
            case Scene::Bounce:
                for (int i = 0; i < 5; ++i) {
                    Body ball = makeCircleBody({-6 + i * 3.0f, 12}, 0.6f, 1);
                    ball.restitution = i * 0.25f;
                    b.push_back(ball);
                    char buf[16];
                    std::snprintf(buf, sizeof buf, "e %.2f", ball.restitution);
                    labels_.push_back({static_cast<int>(b.size()) - 1, buf});
                }
                break;
        }
        spawnBall_ = false;
    }

    bool step() override {
        world_.step(1.0f / 60);
        // 掉出场地的物体删掉（只删最后放入的那些，避免打乱标签的下标）
        auto& b = world_.bodies;
        while (b.size() > 1 && b.back().pos.y < -20) b.pop_back();
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop + 4) {
            Vec2 p = toWorld(m);
            Body body = spawnBall_ ? makeCircleBody(p, 0.5f, 1) : makeBoxBody(p, {0.5f, 0.5f}, 1, 0.3f);
            body.restitution = bounce_;
            world_.bodies.push_back(body);
            spawnBall_ = !spawnBall_;
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        for (size_t i = 0; i < world_.bodies.size(); ++i) drawBody(world_.bodies[i]);
        // 接触点与法线冲量
        for (const Arbiter& a : world_.arbiters())
            for (int k = 0; k < a.count; ++k) {
                const ContactPoint& c = a.points[k];
                Vector2 p = toScreen(c.point);
                float len = std::min(40.0f, c.pn * 60);
                DrawLineEx(p, toScreen(c.point + a.normal * (len / kScale)), 1.5f, pal.path);
                DrawCircleV(p, 2.5f, pal.path);
            }
        for (const auto& [idx, text] : labels_) {
            if (idx >= static_cast<int>(world_.bodies.size())) continue;
            Vector2 p = toScreen(world_.bodies[idx].pos);
            drawTextCentered(text.c_str(), {p.x, p.y - 30}, 11, pal.textMuted);
        }
        drawText("click to drop a box / ball", {16, kTop + 10}, 11, pal.textMuted);
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[] = {"stack", "pyramid", "ramp", "bounce"};
        for (int i = 0; i < 4; ++i)
            if (ui::button({16 + 64.0f * i, 13, 58, 26}, names[i], int(scene_) == i) && int(scene_) != i) {
                scene_ = Scene(i);
                reset();
            }
        if (ui::slider({276, 13, 170, 26}, "iter", &iterations_, 1, 30, "%.0f", 1)) world_.opt.iterations = static_cast<int>(iterations_);
        if (ui::toggle({456, 13, 64, 26}, "warm", &warm_)) world_.opt.warmStart = warm_;
        if (ui::slider({532, 13, 180, 26}, "bounce", &bounce_, 0, 1, "%.2f", 0.05f))
            for (Body& b : world_.bodies) b.restitution = bounce_;
    }

    std::string status() const override {
        char buf[96];
        size_t points = 0;
        for (const Arbiter& a : world_.arbiters()) points += a.count;
        std::snprintf(buf, sizeof buf, "%zu bodies · %zu contact points · %d iterations", world_.bodies.size() - 1, points,
                      world_.opt.iterations);
        return buf;
    }

private:
    void drawBody(const Body& b) const {
        const Palette& pal = palette();
        bool fixed = b.invMass == 0;
        Color ink = fixed ? pal.textMuted : pal.text;
        Color fill = fixed ? Fade(pal.textMuted, 0.25f) : Fade(pal.text, 0.1f);
        if (b.shape == Body::Shape::Circle) {
            Vector2 c = toScreen(b.pos);
            float r = b.radius * kScale;
            DrawCircleV(c, r, fill);
            DrawCircleLinesV(c, r, ink);
            Vec2 spoke = b.pos + fromAngle(b.angle) * b.radius;
            DrawLineEx(c, toScreen(spoke), 1.2f, ink);  // 一条半径线，用来看出旋转
            return;
        }
        ConvexPolygon poly = worldPolygon(b);
        Vector2 pts[4];
        for (int k = 0; k < 4; ++k) pts[k] = toScreen(poly.v[k]);
        // makeBox 的顶点在 y 朝上的世界里是逆时针；翻到屏幕上正好是 raylib 要求的顺序
        DrawTriangle(pts[0], pts[1], pts[2], fill);
        DrawTriangle(pts[0], pts[2], pts[3], fill);
        for (int k = 0; k < 4; ++k) DrawLineEx(pts[k], pts[(k + 1) % 4], 1.2f, ink);
    }

    World world_;
    Scene scene_ = Scene::Stack;
    float iterations_ = 10;
    bool warm_ = true;
    float bounce_ = 0;
    bool spawnBall_ = false;
    std::vector<std::pair<int, std::string>> labels_;
};

}  // namespace

int main() {
    static ImpulseDemo demo;
    ga::DemoConfig config;
    config.title = "Impulse-Based Collision Response";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
