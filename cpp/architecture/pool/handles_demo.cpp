// 句柄与悬空引用
//   - 敌人放在一个 24 格的对象池里，寿命到了就销毁，新敌人住进空出来的槽位
//   - 炮台每隔一会儿发射一枚导弹，锁定一个敌人：记下它的引用
//   - index：只记槽位编号（相当于裸指针 / 下标）。目标死后察觉不到，槽位被新敌人占用后转去追这个无关的敌人（朱红）
//   - handle：编号 + 代数。代数对不上，导弹知道目标没了，直线飞走（灰色）
//   - gen bits：代数只有几位时会回绕，旧句柄又"有效"了。配合 LIFO（刚空出的槽位最先被重用）最明显
//   - 右侧：每个槽位的代数，空闲槽位上的数字是空闲列表里的顺序（1 = 下一个被用）

#include <cmath>
#include <memory>
#include <cstdio>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "pool.h"
#include "ui.h"
#include "vec2.h"

namespace {

using namespace ga;
using namespace ga::pool;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kField = 560;
constexpr uint32_t kSlots = 24;

struct Enemy {
    Vec2 pos, vel;
    float life;
    uint32_t serial;  // 全局唯一的编号：用来判断导弹追的是不是原来那个
};
struct Missile {
    Vec2 pos, vel;
    uint32_t index, gen, serial;  // 引用：槽位编号 + 代数；serial 是真正的目标（只用于统计）
    int state = 0;                // 0 追踪原目标，1 追错了目标，2 发现目标没了
    float fade = 1;
};

class HandlesDemo final : public Demo {
public:
    HandlesDemo() { reset(); }
    void reset() override {
        pool_ = std::make_unique<Pool<Enemy, uint8_t>>(kSlots, FreeOrder(order_));
        missiles_.clear();
        rng_ = Pcg32{17};
        serial_ = 0, fireTimer_ = 0, spawnTimer_ = 0;
        fired_ = hits_ = lost_ = wrong_ = 0;
        for (int i = 0; i < 14; ++i) spawn();
    }
    bool step() override {
        const float dt = 1 / 60.0f;
        // 敌人：游荡、寿命到了就销毁
        std::vector<BasicHandle<uint8_t>> dying;
        for (uint32_t i = 0; i < kSlots; ++i) {
            if (!pool_->alive(i)) continue;
            Enemy& e = *pool_->getUnchecked(i);
            e.pos += e.vel * dt;
            if (e.pos.x < 20 || e.pos.x > kField - 20) e.vel.x = -e.vel.x;
            if (e.pos.y < kTop + 20 || e.pos.y > 330) e.vel.y = -e.vel.y;
            e.life -= dt;
            if (e.life <= 0) dying.push_back({i, pool_->generation(i)});
        }
        for (auto h : dying) pool_->destroy(h);
        spawnTimer_ += dt;
        while (spawnTimer_ > 0.18f) {
            spawnTimer_ -= 0.18f;
            if (pool_->size() < 16) spawn();
        }
        // 炮台
        fireTimer_ += dt;
        if (fireTimer_ > 0.45f && pool_->size() > 0) {
            fireTimer_ = 0;
            uint32_t k = rng_.next() % pool_->size(), i = 0;
            for (;; ++i)
                if (pool_->alive(i) && k-- == 0) break;
            Missile m;
            m.pos = {kField / 2, kHeight - 30.0f};
            m.vel = {0, -150};
            m.index = i, m.gen = pool_->generation(i), m.serial = pool_->getUnchecked(i)->serial;
            missiles_.push_back(m);
            ++fired_;
        }
        // 导弹：解析引用
        uint32_t mask = (1u << int(bits_)) - 1;
        for (Missile& m : missiles_) {
            if (m.state != 2) {
                Enemy* t = pool_->getUnchecked(m.index);
                bool ok = t && (mode_ == 0 || (pool_->generation(m.index) & mask) == (m.gen & mask));
                if (!t && mode_ == 0) {
                    // 只有编号：槽位空着时什么也察觉不到，继续飞，等槽位被新敌人占用
                } else if (!ok) {
                    m.state = 2, ++lost_;
                } else {
                    if (t->serial != m.serial && m.state == 0) m.state = 1, ++wrong_;
                    Vec2 want = normalized(t->pos - m.pos) * 190;
                    m.vel += (want - m.vel) * 0.06f;
                    if (length(t->pos - m.pos) < 12) {  // 命中：敌人销毁
                        pool_->destroy({m.index, pool_->generation(m.index)});
                        m.fade = 0, ++hits_;
                    }
                }
            } else {
                m.fade -= dt * 0.8f;
            }
            m.pos += m.vel * dt;
        }
        std::erase_if(missiles_, [](const Missile& m) {
            return m.fade <= 0 || m.pos.y < kTop - 10 || m.pos.x < -10 || m.pos.x > kField + 10 || m.pos.y > kHeight + 10;
        });
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawLineEx({kField, kTop}, {kField, float(kHeight)}, hl, pal.gridLine);
        for (uint32_t i = 0; i < kSlots; ++i) {
            if (!pool_->alive(i)) continue;
            const Enemy& e = *const_cast<Pool<Enemy, uint8_t>&>(*pool_).getUnchecked(i);
            DrawCircleV({e.pos.x, e.pos.y}, 10, pal.text);
            char b[8];
            std::snprintf(b, sizeof b, "%u", i);
            drawTextCentered(b, {e.pos.x, e.pos.y}, 10, pal.background);
        }
        DrawRectangleV({kField / 2 - 10, kHeight - 24.0f}, {20, 18}, pal.textMuted);
        for (const Missile& m : missiles_) {
            Color c = m.state == 0 ? pal.text : m.state == 1 ? pal.path : pal.textMuted;
            Vec2 tail = m.pos - normalized(m.vel) * 12;
            DrawLineEx({tail.x, tail.y}, {m.pos.x, m.pos.y}, 3, Fade(c, m.fade));
            DrawCircleV({m.pos.x, m.pos.y}, 2.5f, Fade(c, m.fade));
        }
        char buf[160];
        std::snprintf(buf, sizeof buf, "fired %d   hits %d   wrong target %d   lost (stale detected) %d", fired_, hits_, wrong_, lost_);
        drawText(buf, {12, kTop + 8}, 11, pal.textMuted);

        // 槽位
        drawText("slot  gen", {kField + 16, kTop + 8}, 11, pal.textMuted);
        std::vector<int> freeRank(kSlots, 0);
        int rank = 1;
        for (uint32_t f = pool_->freeHead(); f != Pool<Enemy, uint8_t>::kNone; f = pool_->nextFree(f)) freeRank[f] = rank++;
        uint32_t mask = (1u << int(bits_)) - 1;
        for (uint32_t i = 0; i < kSlots; ++i) {
            float x = kField + 16 + float(i % 4) * 54, y = kTop + 28 + float(i / 4) * 52;
            Rectangle r{x, y, 48, 44};
            bool alive = pool_->alive(i);
            if (alive) DrawRectangleRec(r, Fade(pal.text, 0.85f));
            DrawRectangleLinesEx(r, hl, pal.gridLine);
            Color tc = alive ? pal.background : pal.textMuted;
            char b[16];
            std::snprintf(b, sizeof b, "%u", i);
            drawText(b, {x + 4, y + 3}, 10, tc);
            std::snprintf(b, sizeof b, "g%u", mode_ == 0 ? pool_->generation(i) : pool_->generation(i) & mask);
            drawText(b, {x + 4, y + 26}, 11, alive ? pal.background : pal.text);
            if (!alive) {
                std::snprintf(b, sizeof b, "#%d", freeRank[i]);
                drawText(b, {x + 26, y + 3}, 10, pal.path);
            }
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        if (ui::button({12, 10, 70, 26}, "index", mode_ == 0)) mode_ = 0, ch = true;
        if (ui::button({86, 10, 70, 26}, "handle", mode_ == 1)) mode_ = 1, ch = true;
        if (ui::button({168, 10, 60, 26}, "LIFO", order_ == 0)) order_ = 0, ch = true;
        if (ui::button({232, 10, 60, 26}, "FIFO", order_ == 1)) order_ = 1, ch = true;
        if (mode_ == 1) ch |= ui::slider({304, 10, 230, 26}, "gen bits", &bits_, 1, 8, "%.0f", 1);
        if (ch) reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s · %s · wrong %d · lost %d", mode_ ? "handle" : "index", order_ ? "FIFO" : "LIFO", wrong_, lost_);
        return buf;
    }

private:
    void spawn() {
        Enemy e{{30 + rng_.nextFloat() * (kField - 60), kTop + 30 + rng_.nextFloat() * 200},
                fromAngle(rng_.nextFloat() * 2 * kPi) * (20 + rng_.nextFloat() * 30), 1.5f + rng_.nextFloat() * 3.0f, serial_++};
        pool_->create(e);
    }

    std::unique_ptr<Pool<Enemy, uint8_t>> pool_;
    std::vector<Missile> missiles_;
    Pcg32 rng_{17};
    uint32_t serial_ = 0;
    float fireTimer_ = 0, spawnTimer_ = 0, bits_ = 8;
    int fired_ = 0, hits_ = 0, lost_ = 0, wrong_ = 0, mode_ = 0, order_ = 0;
};

}  // namespace

int main() {
    static HandlesDemo demo;
    ga::DemoConfig config;
    config.title = "Handles and dangling references";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
