// 行为树演示：一个搬箱子的工人
//   - 任务：走到门口 → 开门（没锁就直接开；锁了有钥匙就开锁；都不行就踹门，最多 3 次）→ 进屋拿箱子 → 搬到外面的放置点
//   - 饿了（hunger > 70）先去吃东西；鼠标是敌人，靠近时工人逃跑。这两个分支优先级更高，会打断搬运
//   - 右侧是行为树：黄色 = 运行中，墨色 = 成功，朱红 = 失败，浅色 = 这一帧没有执行到
//   - locked / key 切换门锁和钥匙；hungry 让工人立刻饿

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "bt.h"
#include "demo.h"
#include "ui.h"
#include "vec2.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Rectangle kWorld{16, kTop + 12, 440, kHeight - kTop - 24};
constexpr float kPanelX = 474;
constexpr float kDt = 1.0f / 30;
constexpr float kSpeed = 95;

// 房间：左墙上有一扇门
constexpr float kRoomL = 300, kRoomR = 444, kRoomT = 130, kRoomB = 420, kDoorT = 250, kDoorB = 300;
const Vec2 kDoorOut{276, 275}, kDoorIn{324, 275}, kChest{400, 360}, kDrop{70, 110}, kFood{70, 440};

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }
inline bool insideRoom(Vec2 p) { return p.x > kRoomL && p.x < kRoomR && p.y > kRoomT && p.y < kRoomB; }

enum class Door { Closed, Open, Broken };

struct World {
    Vec2 pos{120, 300};
    bool carrying = false;
    bool chestInRoom = true;
    float hunger = 20;
    Door door = Door::Closed;
    bool locked = true, hasKey = false;
    bool enemyNear = false;
    Vec2 enemy{-1000, -1000};
    int timerOwner = -1;  // 哪个动作正在计时
    float timer = 0;
    int delivered = 0;
    Pcg32 rng{3};
    std::string doing;
};

class BtDemo final : public Demo {
public:
    BtDemo() { build(); reset(); }

    void reset() override {
        bool key = world_.hasKey;
        world_ = World{};
        world_.locked = lockDoor_, world_.hasKey = key;
        tree_ = Bt<World>(def_);
    }

    bool step() override {
        world_.hunger = std::min(100.0f, world_.hunger + 4 * kDt);
        world_.enemyNear = distance(world_.enemy, world_.pos) < 100;
        world_.doing.clear();
        tree_.tick(world_);
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        bool in = m.x > kWorld.x && m.x < kWorld.x + kWorld.width && m.y > kWorld.y && m.y < kWorld.y + kWorld.height && !ui::wantsMouse();
        world_.enemy = in ? Vec2{m.x, m.y} : Vec2{-1000, -1000};
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx(kWorld, hl * 1.5f, pal.textMuted);
        // 房间与门
        DrawRectangleRec({kRoomL, kRoomT, kRoomR - kRoomL, kRoomB - kRoomT}, Fade(pal.text, 0.04f));
        auto wall = [&](Vec2 a, Vec2 b) { DrawLineEx(rl(a), rl(b), 4, pal.text); };
        wall({kRoomL, kRoomT}, {kRoomR, kRoomT});
        wall({kRoomR, kRoomT}, {kRoomR, kRoomB});
        wall({kRoomL, kRoomB}, {kRoomR, kRoomB});
        wall({kRoomL, kRoomT}, {kRoomL, kDoorT});
        wall({kRoomL, kDoorB}, {kRoomL, kRoomB});
        if (world_.door == Door::Closed) {
            DrawLineEx({kRoomL, kDoorT}, {kRoomL, kDoorB}, 6, world_.locked ? pal.path : pal.frontier);
            if (world_.locked) DrawCircleV({kRoomL, (kDoorT + kDoorB) / 2}, 4, pal.background);
        } else if (world_.door == Door::Open) {
            DrawLineEx({kRoomL, kDoorT}, {kRoomL + 40, kDoorT - 22}, 4, pal.frontier);  // 推开的门板
        } else {
            for (int i = 0; i < 4; ++i) DrawRectangleRec({kRoomL + 6.0f + i * 9, kDoorB + 6.0f + (i % 2) * 7, 6, 3}, pal.frontier);
        }
        drawText(world_.door == Door::Closed ? (world_.locked ? "locked" : "closed") : world_.door == Door::Open ? "open" : "broken",
                 {kRoomL - 54, kDoorT - 20}, 11, pal.textMuted);
        // 箱子、放置点、食物
        if (world_.chestInRoom && !world_.carrying) {
            DrawRectangleRec({kChest.x - 10, kChest.y - 10, 20, 20}, pal.frontier);
            DrawRectangleLinesEx({kChest.x - 10, kChest.y - 10, 20, 20}, 1.2f, pal.text);
        }
        DrawRectangleLinesEx({kDrop.x - 16, kDrop.y - 16, 32, 32}, 1.2f, pal.textMuted);
        drawTextCentered("drop", {kDrop.x, kDrop.y + 26}, 11, pal.textMuted);
        DrawCircleV(rl(kFood), 9, Fade(pal.path, 0.7f));
        drawTextCentered("food", {kFood.x, kFood.y + 20}, 11, pal.textMuted);
        char buf[32];
        std::snprintf(buf, sizeof buf, "delivered %d", world_.delivered);
        drawText(buf, {kDrop.x - 16, kDrop.y + 40}, 11, pal.textMuted);
        // 敌人
        if (world_.enemy.x > 0) {
            DrawCircleLinesV(rl(world_.enemy), 100, Fade(pal.path, 0.35f));
            DrawCircleV(rl(world_.enemy), 7, pal.path);
        }
        // 工人
        Vec2 p = world_.pos;
        DrawCircleV(rl(p), 10, pal.text);
        if (world_.carrying) {
            DrawRectangleRec({p.x - 7, p.y - 24, 14, 14}, pal.frontier);
            DrawRectangleLinesEx({p.x - 7, p.y - 24, 14, 14}, 1, pal.text);
        }
        DrawRectangleRec({p.x - 15, p.y + 14, 30, 4}, Fade(pal.textMuted, 0.4f));
        DrawRectangleRec({p.x - 15, p.y + 14, 30 * world_.hunger / 100, 4}, world_.hunger > 70 ? pal.path : pal.frontier);
        if (!world_.doing.empty()) drawTextCentered(world_.doing.c_str(), {p.x, p.y - (world_.carrying ? 36.0f : 22.0f)}, 11, pal.text);
        drawText("mouse = enemy · bar = hunger", {kWorld.x + 8, kWorld.y + kWorld.height - 18}, 11, pal.textMuted);

        drawTree();
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::toggle({16, 13, 76, 26}, "locked", &lockDoor_) && world_.door == Door::Closed) world_.locked = lockDoor_;
        ui::toggle({98, 13, 60, 26}, "key", &world_.hasKey);
        if (ui::button({164, 13, 70, 26}, "hungry")) world_.hunger = 100;
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s · hunger %.0f · delivered %d", world_.doing.empty() ? "-" : world_.doing.c_str(), world_.hunger,
                      world_.delivered);
        return buf;
    }

private:
    // ---- 动作的实现 ----
    // 走向 target：要进出房间时，先走到门的这一侧，再穿过门走到另一侧
    static Status goTo(World& w, Vec2 target, const char* label) {
        w.doing = label;
        Vec2 waypoint = target;
        if (insideRoom(w.pos) != insideRoom(target)) {
            Vec2 near = insideRoom(w.pos) ? kDoorIn : kDoorOut, far = insideRoom(w.pos) ? kDoorOut : kDoorIn;
            bool inDoorway = std::abs(w.pos.y - kDoorOut.y) < 1 && w.pos.x >= kDoorOut.x - 1 && w.pos.x <= kDoorIn.x + 1;
            waypoint = inDoorway ? far : near;
        }
        Vec2 d = waypoint - w.pos;
        float len = length(d), stepLen = kSpeed * kDt;
        if (len <= stepLen) {
            w.pos = waypoint;
            return waypoint == target ? Status::Success : Status::Running;
        }
        w.pos += d / len * stepLen;
        return Status::Running;
    }

    // 持续 seconds 秒的动作；时间到了调用 done(w) 决定成功还是失败
    template <class Done>
    static Status timed(World& w, int id, float seconds, const char* label, Done&& done) {
        w.doing = label;
        if (w.timerOwner != id) w.timerOwner = id, w.timer = seconds;
        w.timer -= kDt;
        if (w.timer > 0) return Status::Running;
        w.timerOwner = -1;
        return done(w);
    }

    void build() {
        auto& d = def_;
        auto stop = [](World& w) { w.timerOwner = -1; };  // 被打断时取消计时
        int nextId = 0;
        auto wait = [&](const char* name, float seconds, auto done) {
            int id = nextId++;
            return d.action(name, [=](World& w) { return timed(w, id, seconds, name, done); }, stop);
        };
        auto ok = [](World&) { return Status::Success; };

        int flee = d.sequence("flee", {
            d.condition("enemy near?", [](const World& w) { return w.enemyNear; }),
            d.action("run away", [](World& w) {
                w.doing = "run away!";
                Vec2 next = w.pos + normalized(w.pos - w.enemy) * (kSpeed * 1.3f * kDt);
                bool throughWall = insideRoom(next) != insideRoom(w.pos) && (next.y < kDoorT || next.y > kDoorB || w.door == Door::Closed);
                if (!throughWall) w.pos = next;
                w.pos.x = std::clamp(w.pos.x, kWorld.x + 12, kWorld.x + kWorld.width - 12);
                w.pos.y = std::clamp(w.pos.y, kWorld.y + 12, kWorld.y + kWorld.height - 12);
                return Status::Running;
            }),
        }, /*memory=*/false);

        int eat = d.sequence("eat", {
            d.condition("hungry?", [](const World& w) { return w.hunger > 70; }),
            d.action("go to food", [](World& w) { return goTo(w, kFood, "go to food"); }),
            wait("eat", 1.2f, [](World& w) { w.hunger = 0; return Status::Success; }),
        });

        auto open = [&] { return wait("open", 0.5f, [](World& w) { w.door = Door::Open; return Status::Success; }); };
        int openDoor = d.selector("open door", {
            d.condition("door open?", [](const World& w) { return w.door != Door::Closed; }),
            d.sequence("just open", {d.condition("not locked?", [](const World& w) { return !w.locked; }), open()}),
            d.sequence("use key", {d.condition("have key?", [](const World& w) { return w.hasKey; }),
                                   wait("unlock", 0.9f, [](World& w) { w.locked = false; return Status::Success; }), open()}),
            d.retry(wait("kick door", 0.7f, [](World& w) {
                if (w.rng.nextFloat() >= 0.3f) return Status::Failure;  // 30% 的机会踹开
                w.door = Door::Broken;
                return Status::Success;
            }), 3),
        });

        int fetch = d.sequence("fetch chest", {
            // 已经拿着箱子（之前被打断过）就不用再去门口
            d.action("go to door", [](World& w) { return w.carrying ? Status::Success : goTo(w, insideRoom(w.pos) ? kDoorIn : kDoorOut, "go to door"); }),
            openDoor,
            d.action("go to chest", [](World& w) { return w.carrying ? Status::Success : goTo(w, kChest, "go to chest"); }),
            d.action("pick up", [](World& w) { w.carrying = true, w.chestInRoom = false; return Status::Success; }),
            d.action("carry to drop", [](World& w) { return goTo(w, kDrop, "carry"); }),
            d.action("drop", [this](World& w) {
                // 交货后门被关上（坏了也修好），按开关决定是否重新上锁
                w.carrying = false, w.chestInRoom = true, w.door = Door::Closed, w.locked = lockDoor_, ++w.delivered;
                return Status::Success;
            }),
        });
        int sulk = wait("sulk", 2.0f, ok);
        d.setRoot(d.selector("root", {flee, eat, fetch, sulk}, /*memory=*/false));
    }

    // 节点在树里的缩进层级，用于绘制
    void layout(int n, int depth, std::vector<std::pair<int, int>>& out) const {
        out.push_back({n, depth});
        for (int c : def_.nodes()[n].children) layout(c, depth + 1, out);
    }

    void drawTree() const {
        const Palette& pal = palette();
        DrawLineEx({kPanelX - 8, kTop}, {kPanelX - 8, float(kHeight)}, hairline(), pal.gridLine);
        std::vector<std::pair<int, int>> rows;
        layout(def_.root(), 0, rows);
        float y = kTop + 12;
        using K = BtDef<World>::Kind;
        for (auto [n, depth] : rows) {
            const auto& node = def_.nodes()[n];
            const char* glyph = node.kind == K::Sequence ? "->" : node.kind == K::Selector ? "?" : node.kind == K::Retry ? "R" :
                                node.kind == K::Condition ? "if" : "  ";
            bool visited = tree_.visitedThisTick(n);
            Status s = tree_.lastStatus(n);
            Color c = !visited ? Fade(pal.textMuted, 0.6f) : s == Status::Running ? pal.frontier : s == Status::Success ? pal.text : pal.path;
            float x = kPanelX + depth * 14;
            if (visited && s == Status::Running) DrawRectangleRec({x - 3, y - 2, kWidth - 12 - (x - 3), 16}, Fade(pal.frontier, 0.18f));
            drawText(glyph, {x, y}, 11, visited ? c : Fade(pal.textMuted, 0.6f));
            std::string label = node.name;
            if (!node.memory && (node.kind == K::Sequence || node.kind == K::Selector)) label += "  (reactive)";
            drawText(label.c_str(), {x + 20, y}, 12, c);
            y += 16.5f;
        }
    }

    BtDef<World> def_;
    Bt<World> tree_{def_};
    World world_;
    bool lockDoor_ = true;
};

}  // namespace

int main() {
    static BtDemo demo;
    ga::DemoConfig config;
    config.title = "Behavior Tree";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
