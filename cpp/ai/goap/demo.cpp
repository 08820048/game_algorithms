// GOAP 演示：野外求生
//   - 目标：火已点燃 + 吃饱了。规划器（A*）从当前的世界状态出发，找出代价最小的动作序列
//   - 角色依次走到每个动作的地点执行它；到了发现前提不再满足（比如鱼竿被拿走了），就重新规划
//   - axe / matches / rod：工具棚里是否有这件工具。中途切换会立即触发重新规划
//   - forward / backward：正向或反向搜索。右下角显示搜索展开了多少个节点

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "camp.h"
#include "demo.h"
#include "ui.h"
#include "vec2.h"

namespace {

using namespace ga;
using namespace ga::camp;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Rectangle kMap{16, kTop + 12, 440, kHeight - kTop - 24};
constexpr float kPanelX = 474;
constexpr float kDt = 1.0f / 30;
constexpr float kSpeed = 110;

const Vec2 kPlaces[] = {{90, 130}, {370, 140}, {235, 300}, {380, 420}, {85, 410}};
const char* kPlaceNames[] = {"tool shed", "forest", "camp", "river", "berry bushes"};

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }
inline bool has(uint64_t s, int f) { return (s >> f) & 1; }

class GoapDemo final : public Demo {
public:
    GoapDemo() : actions_(camp::actions()), plain_(plain(actions_)) { reset(); }

    void reset() override {
        state_ = start(axe_, matches_, rod_);
        pos_ = kPlaces[Camp];
        replans_ = 0;
        doneTimer_ = -1;
        replan("start");
    }

    bool step() override {
        if (doneTimer_ >= 0) {
            doneTimer_ += kDt;
            if (doneTimer_ > 2.5f) {  // 歇一会儿，火灭了、又饿了，重新开始
                state_ &= ~((uint64_t(1) << FireLit) | (uint64_t(1) << Fed));
                doneTimer_ = -1;
                replan("goal reached, starting over");
            }
            return true;
        }
        if (!plan_.found || stepIndex_ >= static_cast<int>(plan_.actions.size())) return true;
        const CampAction& a = actions_[plan_.actions[stepIndex_]];
        Vec2 target = kPlaces[a.place];
        if (timer_ < 0) {  // 正在走过去
            Vec2 d = target - pos_;
            float len = length(d);
            if (len > kSpeed * kDt) {
                pos_ += d / len * (kSpeed * kDt);
                return true;
            }
            pos_ = target;
            if (!a.action.pre.satisfiedBy(state_)) {  // 到了才发现做不了
                replan(std::string("cannot ") + a.action.name);
                return true;
            }
            timer_ = a.action.cost * 0.35f;  // 代价越高的动作做得越久
        }
        timer_ -= kDt;
        if (timer_ > 0) return true;
        state_ = a.action.eff.applyTo(state_);
        timer_ = -1;
        ++stepIndex_;
        if (goal().satisfiedBy(state_)) doneTimer_ = 0;
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx(kMap, hl * 1.5f, pal.textMuted);
        // 计划的路线
        if (plan_.found) {
            Vec2 from = pos_;
            for (int k = stepIndex_; k < static_cast<int>(plan_.actions.size()); ++k) {
                Vec2 to = kPlaces[actions_[plan_.actions[k]].place];
                if (distance(from, to) > 1) drawDashedLine(rl(from), rl(to), 1.5f, Fade(pal.frontier, 0.9f), 6, 5);
                from = to;
            }
        }
        drawPlaces();
        // 角色
        DrawCircleV(rl(pos_), 10, pal.text);
        int carry = 0;
        for (int f : {HasAxe, HasWood, HasMatches, HasRod, HasFish, HasSpear, HasMeat})
            if (has(state_, f)) {
                const char* tag[] = {"", "A", "W", "", "M", "", "", "R", "F", "S", "Me"};
                drawText(tag[f], {pos_.x + 13 + carry * 14.0f, pos_.y - 6}, 11, pal.path);
                ++carry;
            }
        if (timer_ >= 0 && plan_.found && stepIndex_ < static_cast<int>(plan_.actions.size()))
            drawTextCentered(actions_[plan_.actions[stepIndex_]].action.name.c_str(), {pos_.x, pos_.y - 22}, 11, pal.text);
        if (doneTimer_ >= 0) drawTextCentered("warm and fed!", {pos_.x, pos_.y - 22}, 12, pal.path);
        drawText("A axe  W wood  M matches  R rod  F fish  S spear  Me meat", {kMap.x + 8, kMap.y + kMap.height - 18}, 10, pal.textMuted);

        drawPanel();
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool changed = false;
        changed |= toolToggle({16, 13, 56, 26}, "axe", &axe_, AxeAvailable, HasAxe);
        changed |= toolToggle({78, 13, 86, 26}, "matches", &matches_, MatchesAvailable, HasMatches);
        changed |= toolToggle({170, 13, 52, 26}, "rod", &rod_, RodAvailable, HasRod);
        if (ui::button({240, 13, 76, 26}, "forward", !backward_) && backward_) backward_ = false, changed = true;
        if (ui::button({322, 13, 80, 26}, "backward", backward_) && !backward_) backward_ = true, changed = true;
        if (changed && doneTimer_ < 0) replan("world changed");
    }

    std::string status() const override {
        char buf[96];
        if (!plan_.found) std::snprintf(buf, sizeof buf, "no plan");
        else std::snprintf(buf, sizeof buf, "step %d/%zu · plan cost %.0f · %d replans", std::min(stepIndex_ + 1, int(plan_.actions.size())),
                           plan_.actions.size(), plan_.cost, replans_);
        return buf;
    }

private:
    // 切换工具：只影响还在棚里（没被角色拿走）的那件
    bool toolToggle(Rectangle r, const char* label, bool* value, int availableFact, int hasFact) {
        if (!ui::toggle(r, label, value)) return false;
        if (has(state_, hasFact)) return false;
        if (*value) state_ |= uint64_t(1) << availableFact;
        else state_ &= ~(uint64_t(1) << availableFact);
        return true;
    }

    void replan(std::string reason) {
        plan_ = backward_ ? planBackward(plain_, state_, goal()) : planForward(plain_, state_, goal());
        stepIndex_ = 0;
        timer_ = -1;
        reason_ = std::move(reason);
        ++replans_;
    }

    void drawPlaces() const {
        const Palette& pal = palette();
        // 工具棚：显示棚里还有哪些工具
        Vec2 s = kPlaces[Shed];
        DrawRectangleLinesEx({s.x - 26, s.y - 20, 52, 40}, 1.5f, pal.text);
        DrawTriangle({s.x - 30, s.y - 20}, {s.x + 30, s.y - 20}, {s.x, s.y - 40}, Fade(pal.text, 0.15f));
        std::string tools;
        if (has(state_, AxeAvailable)) tools += "A ";
        if (has(state_, MatchesAvailable)) tools += "M ";
        if (has(state_, RodAvailable)) tools += "R";
        drawTextCentered(tools.empty() ? "empty" : tools.c_str(), {s.x, s.y}, 11, tools.empty() ? pal.textMuted : pal.path);
        // 树林
        Vec2 f = kPlaces[Forest];
        for (Vec2 o : {Vec2{-24, 6}, Vec2{0, -10}, Vec2{22, 8}, Vec2{-8, 22}}) DrawCircleV(rl(f + o), 11, Fade(pal.text, 0.25f));
        // 营地与火堆
        Vec2 c = kPlaces[Camp];
        DrawCircleLinesV(rl(c), 22, pal.textMuted);
        if (has(state_, FireLit)) {
            DrawTriangle({c.x - 10, c.y + 8}, {c.x + 10, c.y + 8}, {c.x, c.y - 16}, pal.frontier);
            DrawTriangle({c.x - 5, c.y + 8}, {c.x + 5, c.y + 8}, {c.x, c.y - 6}, pal.path);
        } else {
            DrawLineEx({c.x - 10, c.y + 6}, {c.x + 10, c.y - 2}, 3, pal.textMuted);
            DrawLineEx({c.x - 10, c.y - 2}, {c.x + 10, c.y + 6}, 3, pal.textMuted);
        }
        // 河
        Vec2 r = kPlaces[River];
        for (int i = 0; i < 3; ++i) {
            Vector2 prev{};
            for (int k = 0; k <= 20; ++k) {
                Vector2 p{r.x - 60 + k * 6.0f, r.y - 10 + i * 10 + std::sin(k * 0.8f) * 3};
                if (k) DrawLineEx(prev, p, 1.5f, Fade(pal.textMuted, 0.8f));
                prev = p;
            }
        }
        // 浆果丛
        Vec2 b = kPlaces[Bushes];
        DrawCircleV(rl(b), 18, Fade(pal.text, 0.15f));
        for (Vec2 o : {Vec2{-6, -4}, Vec2{5, 3}, Vec2{-2, 8}, Vec2{8, -7}}) DrawCircleV(rl(b + o), 3, pal.path);
        for (int i = 0; i < 5; ++i) drawTextCentered(kPlaceNames[i], {kPlaces[i].x, kPlaces[i].y + 34}, 11, pal.textMuted);
    }

    void drawPanel() const {
        const Palette& pal = palette();
        DrawLineEx({kPanelX - 8, kTop}, {kPanelX - 8, float(kHeight)}, hairline(), pal.gridLine);
        float y = kTop + 12;
        drawText("world state", {kPanelX, y}, 11, pal.textMuted);
        y += 18;
        for (int f = 0; f < kFactCount; ++f) {
            float x = kPanelX + (f % 2) * 158;
            bool on = has(state_, f);
            bool isGoal = f == FireLit || f == Fed;
            drawText(on ? "[x]" : "[ ]", {x, y}, 11, on ? (isGoal ? pal.path : pal.text) : pal.textMuted);
            drawText(factName(f), {x + 26, y}, 11, on ? pal.text : pal.textMuted);
            if (f % 2) y += 16;
        }
        y += 10;
        DrawLineEx({kPanelX, y}, {kWidth - 16.0f, y}, hairline(), pal.gridLine);
        y += 10;
        drawText("plan  (goal: fire lit + fed)", {kPanelX, y}, 11, pal.textMuted);
        y += 18;
        char buf[64];
        if (!plan_.found) drawText("no plan can reach the goal", {kPanelX, y}, 12, pal.path), y += 18;
        for (int k = 0; plan_.found && k < static_cast<int>(plan_.actions.size()); ++k, y += 17) {
            const CampAction& a = actions_[plan_.actions[k]];
            bool current = k == stepIndex_ && doneTimer_ < 0;
            if (current) DrawRectangleRec({kPanelX - 4, y - 2, kWidth - 12 - kPanelX, 16}, Fade(pal.frontier, 0.25f));
            std::snprintf(buf, sizeof buf, "%d. %-14s %-12s %2.0f", k + 1, a.action.name.c_str(), kPlaceNames[a.place], a.action.cost);
            drawText(buf, {kPanelX, y}, 11, k < stepIndex_ ? pal.textMuted : pal.text);
        }
        y += 8;
        std::snprintf(buf, sizeof buf, "total cost %.0f", plan_.cost);
        if (plan_.found) drawText(buf, {kPanelX, y}, 11, pal.text);
        y = kHeight - 56;
        DrawLineEx({kPanelX, y}, {kWidth - 16.0f, y}, hairline(), pal.gridLine);
        std::snprintf(buf, sizeof buf, "%s search: expanded %d nodes", backward_ ? "backward" : "forward", plan_.expanded);
        drawText(buf, {kPanelX, y + 8}, 11, pal.textMuted);
        std::snprintf(buf, sizeof buf, "last replan: %s", reason_.c_str());
        drawText(buf, {kPanelX, y + 24}, 11, pal.textMuted);
    }

    std::vector<CampAction> actions_;
    std::vector<GoapAction> plain_;
    uint64_t state_ = 0;
    Plan plan_;
    int stepIndex_ = 0;
    float timer_ = -1;
    float doneTimer_ = -1;
    Vec2 pos_;
    bool axe_ = true, matches_ = true, rod_ = true, backward_ = false;
    int replans_ = 0;
    std::string reason_;
};

}  // namespace

int main() {
    static GoapDemo demo;
    ga::DemoConfig config;
    config.title = "Goal-Oriented Action Planning";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
