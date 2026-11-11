#include "ecs.h"

#include <cstdio>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;
using namespace ga::ecs;

struct Pos { float x = 0, y = 0; };
struct Vel { float x = 0, y = 0; };
struct Health { float hp = 100; };
struct Burning { float t = 1; };
struct Shield {};

// 参照：每个实体一组 optional
struct Ref {
    uint32_t gen;
    std::optional<Pos> p;
    std::optional<Vel> v;
    std::optional<Health> h;
    std::optional<Burning> b;
    bool shield = false;
};

template <class W>
static void checkSame(W& w, std::map<uint32_t, Ref>& ref, const std::vector<Entity>& all) {
    CHECK(w.count() == ref.size());
    for (Entity e : all) {
        auto it = ref.find(e.index);
        bool alive = it != ref.end() && it->second.gen == e.gen;
        CHECK(w.alive(e) == alive);
        if (!alive) {
            CHECK(!w.template has<Pos>(e) && w.template get<Pos>(e) == nullptr);
            continue;
        }
        const Ref& r = it->second;
        CHECK(w.template has<Pos>(e) == r.p.has_value());
        CHECK(w.template has<Vel>(e) == r.v.has_value());
        CHECK(w.template has<Health>(e) == r.h.has_value());
        CHECK(w.template has<Burning>(e) == r.b.has_value());
        CHECK(w.template has<Shield>(e) == r.shield);
        if (r.p) CHECK(w.template get<Pos>(e)->x == r.p->x && w.template get<Pos>(e)->y == r.p->y);
        if (r.v) CHECK(w.template get<Vel>(e)->x == r.v->x);
        if (r.h) CHECK(w.template get<Health>(e)->hp == r.h->hp);
        if (r.b) CHECK(w.template get<Burning>(e)->t == r.b->t);
    }
    // 查询恰好访问拥有这些组件的实体，每个一次，参数是正确的组件
    std::multiset<uint32_t> got, want;
    w.template each<Pos, Vel>([&](Entity e, Pos& p, Vel& v) {
        got.insert(e.index);
        auto& r = ref.at(e.index);
        CHECK(e.gen == r.gen && p.x == r.p->x && v.x == r.v->x);
    });
    for (auto& [i, r] : ref)
        if (r.p && r.v) want.insert(i);
    CHECK(got == want);
    got.clear(), want.clear();
    w.template each<Health, Burning, Pos>([&](Entity e, Health& h, Burning& b, Pos&) {
        got.insert(e.index);
        CHECK(h.hp == ref.at(e.index).h->hp && b.t == ref.at(e.index).b->t);
    });
    for (auto& [i, r] : ref)
        if (r.h && r.b && r.p) want.insert(i);
    CHECK(got == want);
}

template <class W>
static void testRandom(uint64_t seed) {
    W w;
    Pcg32 rng{seed};
    std::map<uint32_t, Ref> ref;
    std::vector<Entity> all;
    auto pick = [&]() -> Entity { return all[rng.next() % all.size()]; };
    auto live = [&](Entity e) { auto it = ref.find(e.index); return it != ref.end() && it->second.gen == e.gen; };
    for (int step = 0; step < 30000; ++step) {
        int op = int(rng.next() % 12);
        float val = float(rng.next() % 1000);
        if (op < 2 || all.empty()) {
            Entity e = w.create();
            CHECK(!live(e));
            all.push_back(e);
            ref[e.index] = {e.gen};
            continue;
        }
        Entity e = pick();
        bool ok = live(e);
        Ref* r = ok ? &ref[e.index] : nullptr;
        switch (op) {
            case 2: w.destroy(e); if (ok) ref.erase(e.index); break;
            case 3: w.template add<Pos>(e, {val, -val}); if (ok) r->p = Pos{val, -val}; break;
            case 4: w.template add<Vel>(e, {val, 1}); if (ok) r->v = Vel{val, 1}; break;
            case 5: w.template add<Health>(e, {val}); if (ok) r->h = Health{val}; break;
            case 6: w.template add<Burning>(e, {val}); if (ok) r->b = Burning{val}; break;
            case 7: w.template add<Shield>(e); if (ok) r->shield = true; break;
            case 8: w.template remove<Pos>(e); if (ok) r->p.reset(); break;
            case 9: w.template remove<Burning>(e); if (ok) r->b.reset(); break;
            case 10: w.template remove<Shield>(e); if (ok) r->shield = false; break;
            case 11: {  // 系统：在遍历中修改组件、增删组件、销毁实体（延迟到遍历结束）
                std::vector<std::pair<int, uint32_t>> plan;
                w.template each<Pos, Vel>([&](Entity x, Pos& p, Vel& v) {
                    p.x += v.x;
                    uint32_t k = rng.next() % 16;
                    bool hadBurning = w.template has<Burning>(x);
                    if (k == 0) w.destroy(x), plan.push_back({0, x.index});
                    else if (k == 1) w.template add<Burning>(x, {7}), plan.push_back({1, x.index});
                    else if (k == 2) w.template remove<Vel>(x), plan.push_back({2, x.index});
                    // 遍历中实体还活着，组件还在：命令要等遍历结束才执行
                    CHECK(w.alive(x) && w.template has<Vel>(x) && w.template has<Burning>(x) == hadBurning);
                    // 嵌套的遍历结束时，外层的命令还不能执行
                    if (k < 3) {
                        w.template each<Health>([](Entity, Health&) {});
                        CHECK(w.alive(x) && w.template has<Vel>(x) && w.template has<Burning>(x) == hadBurning);
                    }
                });
                for (auto& [i, rr] : ref)
                    if (rr.p && rr.v) rr.p->x += rr.v->x;
                for (auto [kind, i] : plan) {
                    if (!ref.count(i)) continue;
                    if (kind == 0) ref.erase(i);
                    else if (kind == 1) ref[i].b = Burning{7};
                    else ref[i].v.reset();
                }
                break;
            }
        }
        if (step % 1000 == 999) checkSame(w, ref, all);
    }
    checkSame(w, ref, all);
}

// archetype 的内部一致性：每张表里，mask 里的列和实体一样多，其余的列为空
static void testTables() {
    ArchetypeWorld<Pos, Vel, Health, Burning, Shield> w;
    Pcg32 rng{5};
    std::vector<Entity> es;
    for (int i = 0; i < 2000; ++i) es.push_back(w.create());
    for (int step = 0; step < 20000; ++step) {
        Entity e = es[rng.next() % es.size()];
        switch (rng.next() % 6) {
            case 0: w.add<Pos>(e, {float(step), 0}); break;
            case 1: w.add<Vel>(e); break;
            case 2: w.add<Burning>(e); break;
            case 3: w.remove<Pos>(e); break;
            case 4: w.remove<Burning>(e); break;
            case 5: w.add<Shield>(e); break;
        }
    }
    size_t total = 0;
    std::set<uint32_t> masks;
    for (auto& t : w.tables()) {
        CHECK(masks.insert(t.mask).second);  // 每种组合只有一张表
        total += t.size();
        auto check = [&](auto& col, uint32_t bit) { CHECK(col.size() == ((t.mask & bit) ? t.size() : 0)); };
        check(t.template col<Pos>(), 1), check(t.template col<Vel>(), 2), check(t.template col<Health>(), 4);
        check(t.template col<Burning>(), 8), check(t.template col<Shield>(), 16);
    }
    CHECK(total == 2000);
    CHECK(w.tableCount() <= 32);
}

// 实体编号重用：销毁后的旧句柄失效，新实体的代数不同
static void testEntities() {
    Entities ents;
    std::vector<Entity> a;
    for (int i = 0; i < 100; ++i) a.push_back(ents.create());
    for (int i = 0; i < 100; i += 2) CHECK(ents.destroy(a[i]) && !ents.destroy(a[i]));
    // FIFO：最早销毁的编号最先重用（重用分散到所有空闲编号，代数回绕得慢，见 6.1）
    for (int i = 0; i < 50; ++i) {
        Entity e = ents.create();
        CHECK(e.index == uint32_t(2 * i) && e.gen == 1);
    }
    for (int i = 0; i < 100; i += 2) CHECK(!ents.alive(a[i]));
    for (int i = 1; i < 100; i += 2) CHECK(ents.alive(a[i]));
    CHECK(ents.count() == 100);
}

// 稀疏集合的查询从最小的集合开始：访问顺序就是最小集合的 dense 顺序。
// archetype 去掉一个实体没有的组件时不搬动
static void testQueryOrderAndNoops() {
    SparseWorld<Pos, Vel, Health, Burning, Shield> sw;
    std::vector<Entity> es;
    for (int i = 0; i < 50; ++i) es.push_back(sw.create()), sw.add<Pos>(es.back());
    for (int i : {31, 7, 19, 3}) sw.add<Burning>(es[i]);
    std::vector<uint32_t> order;
    sw.each<Pos, Burning>([&](Entity e, Pos&, Burning&) { order.push_back(e.index); });
    CHECK(order == sw.pool<Burning>().entities());
    order.clear();
    sw.each<Burning, Pos>([&](Entity e, Burning&, Pos&) { order.push_back(e.index); });
    CHECK(order == sw.pool<Burning>().entities());

    ArchetypeWorld<Pos, Vel, Health, Burning, Shield> aw;
    Entity e = aw.create();
    aw.add<Pos>(e), aw.add<Vel>(e);
    uint64_t moved = aw.movedRows();
    aw.remove<Burning>(e), aw.remove<Shield>(e);
    aw.add<Pos>(e, {5, 5});  // 已有的组件：只改值
    CHECK(aw.movedRows() == moved && aw.get<Pos>(e)->x == 5);
}

int main() {
    testQueryOrderAndNoops();
    testEntities();
    testRandom<SparseWorld<Pos, Vel, Health, Burning, Shield>>(1);
    testRandom<ArchetypeWorld<Pos, Vel, Health, Burning, Shield>>(1);
    testRandom<SparseWorld<Pos, Vel, Health, Burning, Shield>>(2);
    testRandom<ArchetypeWorld<Pos, Vel, Health, Burning, Shield>>(2);
    testTables();
    return report("ecs_test");
}
