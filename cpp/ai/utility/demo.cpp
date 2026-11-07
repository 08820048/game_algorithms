// 效用 AI 演示：士兵与敌人
//   - 士兵每一帧给六个动作打分，执行得分最高的那个（右侧的条形图）
//   - 点击右侧某个动作，下方显示它的每个考量：响应曲线、当前输入落在曲线上的位置、得到的分数
//   - 拖动敌人（红点）改变局势；hurt 让士兵掉 30 点血
//   - momentum：当前动作的得分乘以 (1 + momentum)。调到 0 观察动作在两个选项之间抖动
//   - product / compensated / geomean：三种把考量分数组合起来的方式

#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

#include "demo.h"
#include "soldier.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::utility;
using namespace ga::soldier;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Vec2 kOrigin{16, kTop + 12};  // 场地左上角
constexpr float kPanelX = 474;
constexpr float kDt = 1.0f / 30;

const char* kInputNames[] = {"health", "ammo", "enemy dist", "medkit dist", "exposure"};

inline Vector2 rl(Vec2 v) { return {v.x + kOrigin.x, v.y + kOrigin.y}; }

class UtilityDemo final : public Demo {
public:
    UtilityDemo() : actions_(soldier::actions()), order_(orderByWeight(actions_)) { reset(); }

    void reset() override {
        world_ = World{};
        rng_ = Pcg32(11);
        frame_ = 0;
        switches_.clear();
    }

    bool step() override {
        ++frame_;
        if (enemyMoves_ && !dragging_) moveEnemy(world_, kDt, rng_);
        auto in = inputs(world_);
        Options opt{mode_, world_.action, momentum_};
        int next = choose(actions_, order_, in.data(), opt).action;
        if (next != world_.action && world_.action >= 0) switches_.push_back(frame_);
        while (!switches_.empty() && switches_.front() < frame_ - 300) switches_.pop_front();  // 只保留最近 10 秒
        world_.action = next;
        act(world_, world_.action, kDt, rng_);
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 local{m.x - kOrigin.x, m.y - kOrigin.y};
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse()) {
            if (world_.enemyAlive() && distance(local, world_.enemy) < 22) dragging_ = true;
            // 点击右侧的动作行：固定显示它的考量；再点一次取消固定
            for (int i = 0; i < kActionCount; ++i)
                if (CheckCollisionPointRec(m, rowRect(i))) pinned_ = pinned_ == i ? -1 : i;
        }
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) dragging_ = false;
        if (dragging_) {
            world_.enemy = clampToArena(local);
            world_.enemyGoal = world_.enemy;
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        drawArena();
        // 射程圈可能超出场地：用底色盖住场地外的部分
        DrawRectangleRec({0, 0, float(kWidth), kOrigin.y}, pal.background);
        DrawRectangleRec({0, 0, kOrigin.x, float(kHeight)}, pal.background);
        DrawRectangleRec({kOrigin.x + kArenaW, 0, kWidth - kOrigin.x - kArenaW, float(kHeight)}, pal.background);
        DrawRectangleRec({0, kOrigin.y + kArenaH, float(kWidth), kHeight - kOrigin.y - kArenaH}, pal.background);
        DrawRectangleLinesEx({kOrigin.x, kOrigin.y, kArenaW, kArenaH}, hl * 1.5f, pal.textMuted);
        drawPanel();
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        ui::toggle({16, 13, 112, 26}, "enemy moves", &enemyMoves_);
        if (ui::button({134, 13, 56, 26}, "hurt")) world_.health = std::max(1.0f, world_.health - 30);
        ui::slider({200, 13, 230, 26}, "momentum", &momentum_, 0, 1, "%.2f", 0.05f);
        const char* names[] = {"product", "compensated", "geomean"};
        const float xs[] = {446, 520, 626}, ws[] = {70, 102, 76};
        for (int m = 0; m < 3; ++m)
            if (ui::button({xs[m], 13, ws[m], 26}, names[m], mode_ == Combine(m))) mode_ = Combine(m);
    }

    std::string status() const override {
        char buf[128];
        std::snprintf(buf, sizeof buf, "%s · %zu switches in last 10 s · kills %d · downed %d",
                      world_.action >= 0 ? actions_[world_.action].name : "-", switches_.size(), world_.kills, world_.downs);
        return buf;
    }

private:
    Rectangle rowRect(int i) const { return {kPanelX - 4, kTop + 30 + i * 23.0f, kWidth - 12 - kPanelX, 21}; }

    void drawArena() const {
        const Palette& pal = palette();
        const World& w = world_;
        // 医疗点
        for (Vec2 m : kMedkits) {
            DrawRectangleRec({rl(m).x - 12, rl(m).y - 4, 24, 8}, Fade(pal.frontier, 0.9f));
            DrawRectangleRec({rl(m).x - 4, rl(m).y - 12, 8, 24}, Fade(pal.frontier, 0.9f));
            drawTextCentered("medkit", {rl(m).x, rl(m).y + 22}, 10, pal.textMuted);
        }
        // 敌人：射程圈与血条
        if (w.enemyAlive()) {
            DrawCircleV(rl(w.enemy), kEnemyRange, Fade(pal.path, 0.06f));
            DrawCircleLinesV(rl(w.enemy), kEnemyRange, Fade(pal.path, 0.4f));
            DrawCircleV(rl(w.enemy), 9, pal.path);
            if (dragging_) DrawCircleLinesV(rl(w.enemy), 13, pal.path);
            drawBar({rl(w.enemy).x - 15, rl(w.enemy).y + 14}, 30, w.enemyHealth / 150, pal.path);
        } else {
            char buf[48];
            std::snprintf(buf, sizeof buf, "enemy down, respawns in %.0fs", std::ceil(w.respawn));
            drawText(buf, {kOrigin.x + 8, kOrigin.y + 8}, 11, pal.textMuted);
        }
        // 士兵
        Vector2 p = rl(w.agent);
        DrawCircleLinesV(p, kAttackRange, Fade(pal.text, 0.12f));
        if (w.action == Attack && w.hitFlash > 0 && w.enemyAlive()) DrawLineEx(p, rl(w.enemy), 1.5f, pal.path);
        if (w.action == Heal) drawDashedLine(p, rl(kMedkits[bestMedkit(w)]), 1.2f, pal.frontier, 5, 4);
        DrawCircleV(p, 10, pal.text);
        drawBar({p.x - 16, p.y + 14}, 32, w.health / kMaxHealth, w.health < 35 ? pal.path : pal.frontier);
        for (int i = 0; i < int(kClip); ++i)  // 弹夹
            DrawRectangleRec({p.x - 16 + i * 4.2f, p.y + 21, 3, 5}, i < w.ammo ? pal.text : Fade(pal.textMuted, 0.35f));
        if (w.action == Reload) DrawRectangleRec({p.x - 16, p.y + 28, 32 * w.reloadTimer / kReloadTime, 2}, pal.text);
        if (w.action >= 0) drawTextCentered(actions_[w.action].name, {p.x, p.y - 22}, 12, pal.text);
        drawText("drag the red enemy · bars: health, ammo", {kOrigin.x + 8, kOrigin.y + kArenaH - 18}, 10, pal.textMuted);
    }

    static void drawBar(Vector2 at, float width, float fraction, Color color) {
        DrawRectangleRec({at.x, at.y, width, 4}, Fade(palette().textMuted, 0.3f));
        DrawRectangleRec({at.x, at.y, width * std::clamp(fraction, 0.0f, 1.0f), 4}, color);
    }

    void drawPanel() const {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawLineEx({kPanelX - 8, kTop}, {kPanelX - 8, float(kHeight)}, hl, pal.gridLine);
        auto in = inputs(world_);
        drawText("action        weight   score", {kPanelX, kTop + 12}, 11, pal.textMuted);
        float best = 0;
        float scores[kActionCount];
        for (int i = 0; i < kActionCount; ++i) {
            scores[i] = score(actions_[i], in.data(), mode_) * (i == world_.action ? 1 + momentum_ : 1);
            best = std::max(best, scores[i]);
        }
        const float barX = kPanelX + 140, barW = 120;
        for (int i = 0; i < kActionCount; ++i) {
            Rectangle r = rowRect(i);
            bool current = i == world_.action, shown = i == shownAction();
            if (current) DrawRectangleRec(r, Fade(pal.frontier, 0.22f));
            if (shown) DrawRectangleLinesEx(r, hl * 1.5f, pal.textMuted);
            float y = r.y + 4;
            drawText(actions_[i].name, {kPanelX + 2, y}, 12, current ? pal.text : pal.textMuted);
            char buf[16];
            std::snprintf(buf, sizeof buf, "%.2f", actions_[i].weight);
            drawText(buf, {kPanelX + 84, y}, 11, pal.textMuted);
            float base = score(actions_[i], in.data(), mode_);
            DrawRectangleRec({barX, y + 2, barW, 9}, Fade(pal.textMuted, 0.15f));
            DrawRectangleRec({barX, y + 2, barW * std::min(base, 1.0f), 9}, current ? pal.path : pal.textMuted);
            if (current && momentum_ > 0)  // 惯性带来的加成
                DrawRectangleRec({barX + barW * std::min(base, 1.0f), y + 2, barW * std::min(scores[i] - base, 1.0f - std::min(base, 1.0f)), 9},
                                 Fade(pal.path, 0.35f));
            std::snprintf(buf, sizeof buf, "%.2f", scores[i]);
            drawText(buf, {barX + barW + 6, y}, 11, current ? pal.text : pal.textMuted);
        }
        drawConsiderations(in.data());
    }

    int shownAction() const { return pinned_ >= 0 ? pinned_ : std::max(world_.action, 0); }

    // 选中动作的每个考量：响应曲线 + 当前输入的位置
    void drawConsiderations(const float* raw) const {
        const Palette& pal = palette();
        const Action& a = actions_[shownAction()];
        float y = kTop + 30 + int(kActionCount) * 23.0f + 10;
        DrawLineEx({kPanelX, y}, {kWidth - 16.0f, y}, hairline(), pal.gridLine);
        y += 8;
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s%s", a.name, pinned_ >= 0 ? "  (pinned, click again to follow)" : "  (click a row to pin)");
        drawText(buf, {kPanelX, y}, 11, pal.text);
        y += 18;
        if (a.considerations.empty()) {
            drawText("no considerations: score = weight", {kPanelX, y}, 11, pal.textMuted);
            return;
        }
        int n = static_cast<int>(a.considerations.size());
        const float plotW = 64, plotH = 36, rowH = (kHeight - 8 - y) / std::max(n, 4);
        for (const Consideration& c : a.considerations) {
            Rectangle plot{kPanelX + 2, y + 2, plotW, std::min(plotH, rowH - 8)};
            DrawRectangleLinesEx(plot, hairline(), pal.gridLine);
            Vector2 prev{};
            for (int k = 0; k <= 32; ++k) {
                float x = k / 32.0f;
                Vector2 q{plot.x + x * plot.width, plot.y + plot.height * (1 - c.curve(x))};
                if (k) DrawLineEx(prev, q, 1.5f, pal.text);
                prev = q;
            }
            float x = c.normalized(raw), s = c.score(raw);
            Vector2 dot{plot.x + x * plot.width, plot.y + plot.height * (1 - s)};
            DrawLineEx({dot.x, plot.y}, {dot.x, plot.y + plot.height}, hairline(), Fade(pal.path, 0.6f));
            DrawCircleV(dot, 3.5f, pal.path);
            drawText(c.name, {plot.x + plotW + 12, y + 2}, 11, pal.text);
            std::snprintf(buf, sizeof buf, "%s %.0f -> %.2f", kInputNames[c.input], raw[c.input], s);
            drawText(buf, {plot.x + plotW + 12, y + 17}, 11, pal.textMuted);
            y += rowH;
        }
    }

    std::vector<Action> actions_;
    std::vector<int> order_;
    World world_;
    Pcg32 rng_{11};
    long frame_ = 0;
    std::deque<long> switches_;
    bool enemyMoves_ = true, dragging_ = false;
    float momentum_ = 0.25f;
    Combine mode_ = Combine::Compensated;
    int pinned_ = -1;
};

}  // namespace

int main() {
    static UtilityDemo demo;
    ga::DemoConfig config;
    config.title = "Utility AI";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
