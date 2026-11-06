// 分层状态机演示：一个守卫
//   - 守卫沿路线巡逻；扇形是它的视野（柱子会挡住视线）
//   - 鼠标就是玩家：走进视野，守卫开始追击，靠近后攻击；躲到柱子后面两秒，它会去最后看到你的地方搜查
//   - 点击场地扔一块石头：守卫听到后过去查看
//   - flash：闪光弹，守卫被击晕 1.5 秒，之后回到原来的状态（历史转移）
//   - shoot：打掉守卫 25 点血；血量低于 30 时逃跑，回到 60 以上再回去巡逻
// 右侧是状态的层次结构，高亮的是当前活动的状态链；下面是最近的状态转移

#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "../../collision/intersect/intersect.h"
#include "demo.h"
#include "guard.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Rectangle kRoom{16, kTop + 12, 540, kHeight - kTop - 24};
constexpr float kPanelX = 576;
constexpr float kDt = 1.0f / 30;
constexpr float kPxPerMeter = 40;
constexpr float kViewRange = 190;
constexpr float kViewHalfAngle = 0.62f;  // 约 35°
constexpr float kHearRange = 260;

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }

const Aabb kPillars[] = {{{170, 170}, {215, 260}}, {{360, 300}, {420, 340}}, {{390, 120}, {430, 190}}};
const Vec2 kRoute[] = {{90, 110}, {500, 110}, {500, 420}, {90, 420}};

class FsmDemo final : public Demo {
public:
    FsmDemo() : states_(buildGuardHfsm(def_)), machine_(def_) { reset(); }

    void reset() override {
        ctx_ = GuardCtx{};
        machine_ = Hfsm<GuardCtx>(def_);
        machine_.start(states_.alive, ctx_);
        pos_ = kRoute[0];
        facing_ = {1, 0};
        waypoint_ = 1;
        log_.clear();
        noiseAt_.reset();
        target_ = pos_;
        lastSeenPos_ = pos_;
        prevLeaf_ = machine_.leaf();
    }

    bool step() override {
        // 1. 感知：写入上下文
        ctx_.t += kDt;
        ctx_.seesPlayer = playerIn_ && canSee(player_);
        if (ctx_.seesPlayer) ctx_.lastSeen = ctx_.t, lastSeenPos_ = player_;
        ctx_.playerDist = playerIn_ ? distance(pos_, player_) / kPxPerMeter : 99;
        ctx_.heardNoise = noiseAt_ && distance(*noiseAt_, pos_) < kHearRange;
        if (ctx_.heardNoise) investigateTarget_ = *noiseAt_;
        noiseAt_.reset();
        ctx_.stunEvent = flash_;
        flash_ = false;
        if (shoot_) ctx_.health -= 25, shoot_ = false;
        if (ctx_.health > 0) ctx_.health = std::min(100.0f, ctx_.health + 3 * kDt);

        // 2. 决策：状态机
        machine_.update(ctx_);
        int leaf = machine_.leaf();
        if (leaf != prevLeaf_) {
            // 从战斗转为搜查：去最后看到玩家的地方
            if (leaf == states_.investigate && (prevLeaf_ == states_.chase || prevLeaf_ == states_.attack)) investigateTarget_ = lastSeenPos_;
            char buf[64];
            std::snprintf(buf, sizeof buf, "%5.1fs  %s -> %s", ctx_.t, def_.states()[prevLeaf_].name.c_str(), machine_.leafName().c_str());
            log_.push_front(buf);
            if (log_.size() > 9) log_.pop_back();
            prevLeaf_ = leaf;
        }

        // 3. 行动：根据当前状态移动
        act(leaf);
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        playerIn_ = m.x > kRoom.x && m.x < kRoom.x + kRoom.width && m.y > kRoom.y && m.y < kRoom.y + kRoom.height && !ui::wantsMouse();
        if (playerIn_) player_ = {m.x, m.y};
        if (playerIn_ && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            noiseAt_ = player_;
            ripple_ = {player_, 0};
        }
        if (ripple_.second >= 0) ripple_.second += GetFrameTime();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx(kRoom, hl * 1.5f, pal.textMuted);
        for (const Vec2& p : kRoute) DrawCircleV(rl(p), 3, Fade(pal.textMuted, 0.6f));
        for (int i = 0; i < 4; ++i) drawDashedLine(rl(kRoute[i]), rl(kRoute[(i + 1) % 4]), hl * 1.5f, Fade(pal.textMuted, 0.5f), 4, 6);
        for (const Aabb& b : kPillars) {
            Rectangle r{b.min.x, b.min.y, b.max.x - b.min.x, b.max.y - b.min.y};
            DrawRectangleRec(r, Fade(pal.text, 0.18f));
            DrawRectangleLinesEx(r, 1.2f, pal.text);
        }
        int leaf = machine_.leaf();
        bool dead = leaf == states_.dead, stunned = leaf == states_.stunned;

        // 视野扇形：被柱子挡住的部分截断
        BeginScissorMode(int(kRoom.x), int(kRoom.y), int(kRoom.width), int(kRoom.height));
        if (!dead && !stunned) {
            Color cone = machine_.isActive(states_.combat) ? Fade(pal.path, 0.16f) : Fade(pal.frontier, 0.22f);
            const int rays = 36;
            Vector2 prev{};
            for (int i = 0; i <= rays; ++i) {
                float a = -kViewHalfAngle + 2 * kViewHalfAngle * i / rays;
                Vec2 dir = rotate(facing_, a);
                float len = kViewRange;
                for (const Aabb& b : kPillars)
                    if (auto hit = raycast(ga::Ray{pos_, dir}, b, kViewRange)) len = std::min(len, hit->t);
                Vector2 p = rl(pos_ + dir * len);
                if (i > 0) DrawTriangle(rl(pos_), p, prev, cone);
                prev = p;
            }
        }
        EndScissorMode();
        // 搜查目标、石头的波纹
        if (leaf == states_.investigate) {
            DrawCircleLinesV(rl(investigateTarget_), 8, pal.frontier);
            drawDashedLine(rl(pos_), rl(investigateTarget_), 1.2f, pal.frontier, 4, 4);
        }
        if (ripple_.second >= 0 && ripple_.second < 0.8f) {
            float r = 10 + ripple_.second * 300;
            DrawCircleLinesV(rl(ripple_.first), r, Fade(pal.textMuted, 1 - ripple_.second / 0.8f));
        }
        // 玩家
        if (playerIn_) {
            DrawCircleV(rl(player_), 7, ctx_.seesPlayer ? pal.path : pal.text);
            DrawCircleV(rl(player_), 3, pal.background);
        }
        // 守卫
        Color body = dead ? pal.textMuted : machine_.isActive(states_.combat) ? pal.path : pal.text;
        Vec2 face = stunned ? rotate(facing_, std::sin(ctx_.t * 20) * 0.6f) : facing_;
        DrawCircleV(rl(pos_), 11, body);
        DrawLineEx(rl(pos_), rl(pos_ + face * 16), 3, body);
        if (dead) {
            DrawLineEx(rl(pos_ + Vec2{-8, -8}), rl(pos_ + Vec2{8, 8}), 2.5f, pal.background);
            DrawLineEx(rl(pos_ + Vec2{-8, 8}), rl(pos_ + Vec2{8, -8}), 2.5f, pal.background);
        }
        if (leaf == states_.attack && std::fmod(ctx_.t, 0.6f) < 0.25f) DrawCircleLinesV(rl(pos_), 24, pal.path);
        if (stunned) drawTextCentered("* *", rl(pos_ + Vec2{0, -22}), 13, pal.frontier);
        // 血条
        float w = 30;
        DrawRectangleRec({pos_.x - w / 2, pos_.y + 16, w, 4}, Fade(pal.textMuted, 0.4f));
        DrawRectangleRec({pos_.x - w / 2, pos_.y + 16, w * std::max(0.0f, ctx_.health) / 100, 4}, ctx_.health < 30 ? pal.path : pal.text);
        drawText("mouse = player · click = throw a stone", {kRoom.x + 8, kRoom.y + kRoom.height - 18}, 11, pal.textMuted);

        drawPanel();
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({16, 13, 64, 26}, "flash")) flash_ = true;
        if (ui::button({86, 13, 64, 26}, "shoot")) shoot_ = true;
        if (ui::button({156, 13, 64, 26}, "revive")) reset();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s · health %.0f · %ld transitions", machine_.leafName().c_str(), std::max(0.0f, ctx_.health),
                      machine_.transitionsTaken());
        return buf;
    }

private:
    bool canSee(Vec2 p) const {
        Vec2 d = p - pos_;
        float dist = length(d);
        if (dist > kViewRange) return false;
        if (dist < 40) return true;  // 贴得很近：不在视野里也能察觉
        if (dot(d / dist, facing_) < std::cos(kViewHalfAngle)) return false;
        for (const Aabb& b : kPillars)
            if (auto hit = raycast(ga::Ray{pos_, d / dist}, b, dist)) return false;
        return true;
    }

    void moveToward(Vec2 goal, float speed) {
        Vec2 d = goal - pos_;
        float dist = length(d);
        if (dist < 1) return;
        Vec2 dir = d / dist;
        turnToward(dir);
        Vec2 next = pos_ + dir * std::min(dist, speed * kDt);
        // 简单地绕开柱子：撞上了就沿着另一个轴走
        auto blocked = [&](Vec2 q) {
            for (const Aabb& b : kPillars)
                if (overlaps(Circle{q, 11}, b)) return true;
            return false;
        };
        if (!blocked(next)) pos_ = next;
        else if (!blocked({next.x, pos_.y})) pos_.x = next.x;
        else if (!blocked({pos_.x, next.y})) pos_.y = next.y;
        else pos_ += perp(dir) * speed * kDt;
        pos_.x = std::clamp(pos_.x, kRoom.x + 12, kRoom.x + kRoom.width - 12);
        pos_.y = std::clamp(pos_.y, kRoom.y + 12, kRoom.y + kRoom.height - 12);
    }
    void turnToward(Vec2 dir) {
        float a = signedAngle(facing_, dir);
        facing_ = rotate(facing_, std::clamp(a, -6 * kDt, 6 * kDt));
    }

    void act(int leaf) {
        if (leaf == states_.patrol) {
            if (distance(pos_, kRoute[waypoint_]) < 4) waypoint_ = (waypoint_ + 1) % 4;
            moveToward(kRoute[waypoint_], 60);
        } else if (leaf == states_.investigate) {
            if (distance(pos_, investigateTarget_) > 4) moveToward(investigateTarget_, 75);
            else facing_ = rotate(facing_, 2.5f * kDt);  // 到了就四处张望
        } else if (leaf == states_.chase) {
            moveToward(lastSeenPos_, 100);
        } else if (leaf == states_.attack) {
            turnToward(normalized(player_ - pos_));
        } else if (leaf == states_.flee) {
            Vec2 away = pos_ - lastSeenPos_;
            moveToward(pos_ + normalized(lengthSq(away) > 0 ? away : Vec2{1, 0}) * 50, 105);
        }
    }

    void drawPanel() const {
        const Palette& pal = palette();
        DrawLineEx({kPanelX - 8, kTop}, {kPanelX - 8, float(kHeight)}, hairline(), pal.gridLine);
        float y = kTop + 14;
        drawText("state hierarchy", {kPanelX, y}, 11, pal.textMuted);
        y += 20;
        for (int s = 0; s < static_cast<int>(def_.states().size()); ++s) {
            const auto& st = def_.states()[s];
            bool on = machine_.isActive(s);
            float x = kPanelX + 6 + st.depth * 16;
            if (on) DrawRectangleRec({x - 4, y - 2, kWidth - 16 - (x - 4), 18}, Fade(s == machine_.leaf() ? pal.path : pal.frontier, 0.18f));
            drawText(st.name.c_str(), {x, y}, 12, on ? pal.text : pal.textMuted);
            y += 19;
        }
        y += 10;
        DrawLineEx({kPanelX, y}, {kWidth - 16.0f, y}, hairline(), pal.gridLine);
        y += 10;
        drawText("recent transitions", {kPanelX, y}, 11, pal.textMuted);
        y += 18;
        for (size_t i = 0; i < log_.size(); ++i, y += 16) drawText(log_[i].c_str(), {kPanelX, y}, 11, i == 0 ? pal.text : pal.textMuted);
    }

    HfsmDef<GuardCtx> def_;
    GuardStates states_;
    Hfsm<GuardCtx> machine_;
    GuardCtx ctx_;
    Vec2 pos_, facing_;
    int waypoint_ = 1;
    Vec2 player_{-100, -100};
    bool playerIn_ = false;
    std::optional<Vec2> noiseAt_;
    std::pair<Vec2, float> ripple_{{0, 0}, -1};
    Vec2 investigateTarget_, target_, lastSeenPos_;
    bool flash_ = false, shoot_ = false;
    int prevLeaf_ = -1;
    std::deque<std::string> log_;
};

}  // namespace

int main() {
    static FsmDemo demo;
    ga::DemoConfig config;
    config.title = "Hierarchical State Machine";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
