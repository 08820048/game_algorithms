// ECS 的两种存储
//   - 左边：小生物（Pos、Vel、Health）在场地里游走；石头只有 Pos；一部分生物带 Shield
//     走进火区加上 Burning（掉血），走进水区去掉 Burning；血量耗尽就销毁，过一会儿补充新的
//   - 右边：同一个世界在两种存储里的样子。archetype：组件组合相同的实体在同一张表里，每种组件一列；
//     sparse set：每种组件一个紧凑数组。朱红框：刚刚换了表（archetype）或刚加入数组（sparse set）的实体
//   - 底部计数：archetype 搬了多少行、复制了多少字节；sparse set 只增删一个组件

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "ecs.h"
#include "ui.h"
#include "vec2.h"

namespace {

using namespace ga;
using namespace ga::ecs;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kField = 430;

struct Pos { Vec2 p; };
struct Vel { Vec2 v; };
struct Health { float hp = 100; };
struct Burning { float dps = 25; };
struct Shield {};

const char* kNames[5] = {"Pos", "Vel", "Health", "Burning", "Shield"};
const int kSizes[5] = {8, 8, 4, 4, 0};

struct Zone {
    Vec2 c;
    float r;
    bool fire;
};

class StorageDemo final : public Demo {
public:
    StorageDemo() { reset(); }
    void reset() override {
        aw_ = ArchetypeWorld<Pos, Vel, Health, Burning, Shield>{};
        sw_ = SparseWorld<Pos, Vel, Health, Burning, Shield>{};
        ids_.clear(), flash_.clear();
        rng_ = Pcg32{12};
        moves_ = 0, bytes_ = 0, sparseOps_ = 0, respawn_ = 0;
        for (int i = 0; i < 4; ++i) {  // 石头
            Entity a = aw_.create(), b = sw_.create();
            Vec2 p{40 + rng_.nextFloat() * (kField - 80), kTop + 40 + rng_.nextFloat() * 300};
            aw_.add<Pos>(a, {p}), sw_.add<Pos>(b, {p});
            ids_.push_back({a, b});
        }
        for (int i = 0; i < 26; ++i) spawnCreature();
    }
    bool step() override {
        const float dt = 1 / 60.0f;
        std::vector<std::pair<Entity, int>> changes;  // (实体, 0 加 Burning / 1 去掉 / 2 销毁)
        aw_.each<Pos, Vel, Health>([&](Entity e, Pos& p, Vel& v, Health& h) {
            p.p += v.v * dt;
            if (p.p.x < 10 || p.p.x > kField - 10) v.v.x = -v.v.x;
            if (p.p.y < kTop + 10 || p.p.y > kHeight - 40) v.v.y = -v.v.y;
            bool burning = aw_.has<Burning>(e);
            if (burning) h.hp -= 25 * dt * (aw_.has<Shield>(e) ? 0.3f : 1);
            for (const Zone& z : zones_)
                if (length(p.p - z.c) < z.r) {
                    if (z.fire && !burning) changes.push_back({e, 0});
                    if (!z.fire && burning) changes.push_back({e, 1});
                }
            if (h.hp <= 0) changes.push_back({e, 2});
        });
        // 两个世界执行同样的结构变化
        for (auto [e, kind] : changes) {
            int i = indexOf(e);
            if (i < 0) continue;
            Entity b = ids_[i].second;
            if (kind == 0 && !aw_.has<Burning>(e)) {
                aw_.add<Burning>(e), sw_.add<Burning>(b), countMove(e, true), flash_[e.index] = 30;
            } else if (kind == 1 && aw_.has<Burning>(e)) {
                countMove(e, false), aw_.remove<Burning>(e), sw_.remove<Burning>(b), flash_[e.index] = 30;
            } else if (kind == 2) {
                aw_.destroy(e), sw_.destroy(b), ids_.erase(ids_.begin() + i);
            }
        }
        // 同步位置和血量到 sparse 世界（演示里只用来显示）
        for (auto& [a, b] : ids_) {
            if (Pos* p = aw_.get<Pos>(a)) sw_.get<Pos>(b)->p = p->p;
            if (Health* h = aw_.get<Health>(a)) sw_.get<Health>(b)->hp = h->hp;
        }
        for (auto& [k, f] : flash_) f = std::max(0, f - 1);
        respawn_ += dt;
        if (respawn_ > 0.8f && aw_.count() < 30) respawn_ = 0, spawnCreature();
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        auto& aw = const_cast<ArchetypeWorld<Pos, Vel, Health, Burning, Shield>&>(aw_);
        for (const Zone& z : zones_)
            DrawCircleV({z.c.x, z.c.y}, z.r, z.fire ? Fade(Color{230, 90, 40, 255}, 0.22f) : Fade(Color{60, 130, 220, 255}, 0.2f));
        drawText("fire", {zones_[0].c.x - 12, zones_[0].c.y - 6}, 11, pal.textMuted);
        drawText("water", {zones_[1].c.x - 16, zones_[1].c.y - 6}, 11, pal.textMuted);
        for (auto& [a, b] : ids_) {
            Vec2 p = aw.get<Pos>(a)->p;
            bool creature = aw.has<Vel>(a);
            if (!creature) {
                DrawRectangleV({p.x - 7, p.y - 7}, {14, 14}, Fade(pal.wall, 0.6f));
            } else {
                Color c = aw.has<Burning>(a) ? Color{230, 90, 40, 255} : pal.text;
                DrawCircleV({p.x, p.y}, 9, c);
                if (aw.has<Shield>(a)) DrawCircleLinesV({p.x, p.y}, 12, pal.text);
                float hp = aw.get<Health>(a)->hp / 100;
                DrawRectangleV({p.x - 9, p.y - 15}, {18 * hp, 2}, pal.textMuted);
            }
            char t[8];
            std::snprintf(t, sizeof t, "%u", a.index);
            drawTextCentered(t, {p.x, p.y}, 9, creature ? pal.background : pal.text);
            if (flashOf(a.index)) DrawCircleLinesV({p.x, p.y}, 15, pal.path);
        }
        DrawLineEx({kField + 8, kTop}, {kField + 8, float(kHeight)}, hl, pal.gridLine);

        // 存储
        const float x0 = kField + 20, cell = 13;
        float y = kTop + 8;
        auto cellAt = [&](float x, float yy, uint32_t idx, bool hot) {
            Rectangle r{x, yy, cell - 1, cell - 1};
            DrawRectangleRec(r, Fade(pal.text, 0.12f));
            if (hot) DrawRectangleLinesEx(r, 1.4f, pal.path);
            char t[8];
            std::snprintf(t, sizeof t, "%u", idx);
            drawTextCentered(t, {x + cell / 2 - 0.5f, yy + cell / 2 - 0.5f}, 8, pal.text);
        };
        const int perLine = int((kWidth - x0 - 70) / cell);
        if (view_ == 0) {
            for (auto& t : aw.tables()) {
                if (t.size() == 0) continue;
                std::string name;
                for (int c = 0; c < 5; ++c)
                    if (t.mask & (1u << c)) name += std::string(name.empty() ? "" : " ") + kNames[c];
                char head[96];
                std::snprintf(head, sizeof head, "[%s]  %zu rows", name.empty() ? "-" : name.c_str(), t.size());
                drawText(head, {x0, y}, 10, pal.textMuted);
                y += 14;
                for (int c = 0; c < 5; ++c) {
                    if (!(t.mask & (1u << c))) continue;
                    drawText(kNames[c], {x0, y + 1}, 9, pal.textMuted);
                    for (size_t row = 0; row < t.size() && int(row) < perLine; ++row)
                        cellAt(x0 + 52 + float(row) * cell, y, t.entities[row], flashOf(t.entities[row]));
                    y += cell;
                }
                y += 6;
            }
        } else {
            auto row = [&](const char* name, const std::vector<uint32_t>& ents) {
                char head[48];
                std::snprintf(head, sizeof head, "%s  %zu", name, ents.size());
                drawText(head, {x0, y}, 10, pal.textMuted);
                y += 14;
                for (size_t k = 0; k < ents.size(); ++k) {
                    if (k > 0 && k % perLine == 0) y += cell;
                    uint32_t aIdx = archetypeIndexOfSparse(ents[k]);
                    cellAt(x0 + float(k % perLine) * cell, y, aIdx, flashOf(aIdx));
                }
                y += cell + 8;
            };
            auto& sw = const_cast<SparseWorld<Pos, Vel, Health, Burning, Shield>&>(sw_);
            row("Pos", sw.pool<Pos>().entities());
            row("Vel", sw.pool<Vel>().entities());
            row("Health", sw.pool<Health>().entities());
            row("Burning", sw.pool<Burning>().entities());
            row("Shield", sw.pool<Shield>().entities());
        }
        char buf[200];
        std::snprintf(buf, sizeof buf, "archetype: %ld rows moved, %ld bytes copied   sparse set: %ld single-component adds/removes", moves_,
                      bytes_, sparseOps_);
        drawText(buf, {12, kHeight - 18.0f}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 10, 100, 26}, "archetype", view_ == 0)) view_ = 0;
        if (ui::button({116, 10, 100, 26}, "sparse set", view_ == 1)) view_ = 1;
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%u entities · %ld rows moved", aw_.count(), moves_);
        return buf;
    }

private:
    void spawnCreature() {
        Entity a = aw_.create(), b = sw_.create();
        Vec2 p{30 + rng_.nextFloat() * (kField - 60), kTop + 30 + rng_.nextFloat() * 320};
        Vec2 v = fromAngle(rng_.nextFloat() * 2 * kPi) * (30 + rng_.nextFloat() * 40);
        aw_.add<Pos>(a, {p}), aw_.add<Vel>(a, {v}), aw_.add<Health>(a);
        sw_.add<Pos>(b, {p}), sw_.add<Vel>(b, {v}), sw_.add<Health>(b);
        if (rng_.next() % 3 == 0) aw_.add<Shield>(a), sw_.add<Shield>(b);
        ids_.push_back({a, b});
        flash_[a.index] = 30;
    }
    int indexOf(Entity a) const {
        for (size_t i = 0; i < ids_.size(); ++i)
            if (ids_[i].first == a) return int(i);
        return -1;
    }
    uint32_t archetypeIndexOfSparse(uint32_t s) const {
        for (auto& [a, b] : ids_)
            if (b.index == s) return a.index;
        return s;
    }
    bool flashOf(uint32_t idx) const {
        auto it = flash_.find(idx);
        return it != flash_.end() && it->second > 0;
    }
    void countMove(Entity e, bool adding) {
        // 搬一行要复制两张表都有的组件：加 Burning 时是原有的全部，去掉时是除 Burning 以外的全部
        (void)adding;
        int bytes = 0;
        bytes += aw_.has<Pos>(e) ? kSizes[0] : 0;
        bytes += aw_.has<Vel>(e) ? kSizes[1] : 0;
        bytes += aw_.has<Health>(e) ? kSizes[2] : 0;
        ++moves_, bytes_ += bytes, ++sparseOps_;
    }

    ArchetypeWorld<Pos, Vel, Health, Burning, Shield> aw_;
    SparseWorld<Pos, Vel, Health, Burning, Shield> sw_;
    std::vector<std::pair<Entity, Entity>> ids_;
    std::map<uint32_t, int> flash_;
    std::vector<Zone> zones_{{{120, 200}, 55, true}, {{320, 360}, 60, false}};
    Pcg32 rng_{12};
    long moves_ = 0, bytes_ = 0, sparseOps_ = 0;
    float respawn_ = 0;
    int view_ = 0;
};

}  // namespace

int main() {
    static StorageDemo demo;
    ga::DemoConfig config;
    config.title = "ECS storage";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
