// ECS 的测量
//   c++ -std=c++20 -O2 -Icpp/common cpp/architecture/ecs/ecs_bench.cpp -o /tmp/ecsb && /tmp/ecsb [section]
//   section: iterate | query | struct | frag（默认全部）

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <vector>

#include "../../foundations/random/random.h"
#include "ecs.h"

using namespace ga;
using namespace ga::ecs;

template <class F>
static double bestMs(int rounds, F&& f) {
    double best = 1e30;
    for (int r = 0; r < rounds; ++r) {
        auto t0 = std::chrono::steady_clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    }
    return best;
}
static volatile float sink;

struct Pos { float x = 0, y = 0; };
struct Vel { float x = 1, y = 1; };
struct Health { float hp = 100; };
struct Burning { float t = 1; };
struct Shield {};
struct Big1 { float v[4] = {}; };
struct Big2 { float v[4] = {}; };
struct Big3 { float v[4] = {}; };
struct Big4 { float v[4] = {}; };
struct Big5 { float v[4] = {}; };
template <int I>
struct Tag {};
#define TAGS Tag<0>, Tag<1>, Tag<2>, Tag<3>, Tag<4>, Tag<5>, Tag<6>, Tag<7>, Tag<8>, Tag<9>, Tag<10>, Tag<11>
using SW = SparseWorld<Pos, Vel, Health, Burning, Shield, Big1, Big2, Big3, Big4, Big5, TAGS>;
using AW = ArchetypeWorld<Pos, Vel, Health, Burning, Shield, Big1, Big2, Big3, Big4, Big5, TAGS>;

// 面向对象的对照：基类 + 虚函数，每个对象单独 new
struct Object {
    float x = 0, y = 0, vx = 1, vy = 1, hp = 100;
    virtual ~Object() = default;
    virtual void update(float dt) = 0;
};
struct Mover : Object { void update(float dt) override { x += vx * dt, y += vy * dt; } };
struct Enemy : Object { float aggro = 0; void update(float dt) override { x += vx * dt, y += vy * dt, aggro += dt; } };
struct Bullet : Object { float life = 1; void update(float dt) override { x += vx * dt, y += vy * dt, life -= dt; } };
struct Plain { Pos p; Vel v; Health h; };

static void iterate() {
    std::printf("== movement system: pos += vel * dt, every entity has Pos, Vel, Health (ns per entity) ==\n");
    std::printf("%-8s %11s %11s %11s %11s %11s %11s %11s\n", "N", "OOP sorted", "OOP types", "OOP shuffle", "AoS array", "SoA arrays", "sparse set", "archetype");
    for (int n : {10000, 100000, 1000000}) {
        const float dt = 0.016f;
        std::vector<std::unique_ptr<Object>> objs;
        for (int i = 0; i < n; ++i) {
            int k = i * 3 / n;  // 按类型排好：先全部 Mover，再 Enemy，再 Bullet
            if (k == 0) objs.push_back(std::make_unique<Mover>());
            else if (k == 1) objs.push_back(std::make_unique<Enemy>());
            else objs.push_back(std::make_unique<Bullet>());
        }
        double oop = bestMs(10, [&] { for (auto& o : objs) o->update(dt); });
        std::shuffle(objs.begin(), objs.end(), std::mt19937(1));  // 类型混在一起（指针也打乱）
        double oopMixed = bestMs(10, [&] { for (auto& o : objs) o->update(dt); });
        // 只打乱类型、不打乱内存：按随机类型依次分配，遍历顺序 = 分配顺序
        std::vector<std::unique_ptr<Object>> objs2;
        Pcg32 tr{9};
        for (int i = 0; i < n; ++i) {
            int k = int(tr.next() % 3);
            if (k == 0) objs2.push_back(std::make_unique<Mover>());
            else if (k == 1) objs2.push_back(std::make_unique<Enemy>());
            else objs2.push_back(std::make_unique<Bullet>());
        }
        double oopTypes = bestMs(10, [&] { for (auto& o : objs2) o->update(dt); });
        std::vector<Pos> sp_(n);
        std::vector<Vel> sv_(n);
        double soa = bestMs(10, [&] { for (int i = 0; i < n; ++i) sp_[i].x += sv_[i].x * dt, sp_[i].y += sv_[i].y * dt; });
        std::vector<Plain> plain(n);
        double pl = bestMs(10, [&] { for (auto& e : plain) e.p.x += e.v.x * dt, e.p.y += e.v.y * dt; });
        SW sw;
        AW aw;
        for (int i = 0; i < n; ++i) {
            Entity a = sw.create(), b = aw.create();
            sw.add<Pos>(a), sw.add<Vel>(a), sw.add<Health>(a);
            aw.add<Pos>(b), aw.add<Vel>(b), aw.add<Health>(b);
        }
        double sp = bestMs(10, [&] { sw.each<Pos, Vel>([dt](Entity, Pos& p, Vel& v) { p.x += v.x * dt, p.y += v.y * dt; }); });
        double ar = bestMs(10, [&] { aw.each<Pos, Vel>([dt](Entity, Pos& p, Vel& v) { p.x += v.x * dt, p.y += v.y * dt; }); });
        auto ns = [n](double ms) { return ms * 1e6 / n; };
        std::printf("%-8d %11.2f %11.2f %11.2f %11.2f %11.2f %11.2f %11.2f\n", n, ns(oop), ns(oopTypes), ns(oopMixed), ns(pl), ns(soa), ns(sp), ns(ar));
    }
}

// 三种组件的查询：Pos 人人都有，Health 50%，Burning 10%（随机）
static void query() {
    std::printf("\n== query <Pos, Health, Burning>: Health on 50%%, Burning on 10%% (random), N = 1M ==\n");
    const int n = 1000000;
    Pcg32 r{2};
    SW sw;
    AW aw;
    for (int i = 0; i < n; ++i) {
        Entity a = sw.create(), b = aw.create();
        sw.add<Pos>(a), aw.add<Pos>(b), sw.add<Vel>(a), aw.add<Vel>(b);
        if (r.next() % 2 == 0) sw.add<Health>(a), aw.add<Health>(b);
        if (r.next() % 10 == 0) sw.add<Burning>(a), aw.add<Burning>(b);
    }
    long matched = 0;
    double sp = bestMs(10, [&] {
        matched = 0;
        sw.each<Pos, Health, Burning>([&](Entity, Pos& p, Health& h, Burning& b) { h.hp -= b.t, p.x += 1, ++matched; });
    });
    double ar = bestMs(10, [&] { aw.each<Pos, Health, Burning>([&](Entity, Pos& p, Health& h, Burning& b) { h.hp -= b.t, p.x += 1; }); });
    std::printf("matched %ld   sparse set %.3f ms (%.2f ns/match)   archetype %.3f ms (%.2f ns/match)   tables %zu\n", matched, sp,
                sp * 1e6 / matched, ar, ar * 1e6 / matched, aw.tableCount());
    // 查询顺序的影响：sparse set 若从最大的集合开始
    double big = bestMs(10, [&] {
        auto& pos = sw.pool<Pos>();
        auto& ents = pos.entities();
        for (size_t k = 0; k < ents.size(); ++k) {
            uint32_t e = ents[k];
            if (Health* h = sw.pool<Health>().get(e))
                if (Burning* b = sw.pool<Burning>().get(e)) h->hp -= b->t, pos.data()[k].x += 1;
        }
    });
    std::printf("sparse set iterating the largest set (Pos) instead: %.3f ms\n", big);
}

// 结构变化：每帧随机给 1% 的实体加上 / 去掉 Burning
template <class W>
static double toggleCost(int n, bool fat) {
    W w;
    std::vector<Entity> es;
    for (int i = 0; i < n; ++i) {
        Entity e = w.create();
        w.template add<Pos>(e), w.template add<Vel>(e), w.template add<Health>(e);
        if (fat) w.template add<Big1>(e), w.template add<Big2>(e), w.template add<Big3>(e), w.template add<Big4>(e), w.template add<Big5>(e);
        es.push_back(e);
    }
    Pcg32 r{3};
    const int ops = 400000;
    std::vector<uint32_t> picks(ops);
    for (auto& p : picks) p = r.next() % uint32_t(n);
    return bestMs(3, [&] {
               for (uint32_t k : picks) {
                   Entity e = es[k];
                   if (w.template has<Burning>(e)) w.template remove<Burning>(e);
                   else w.template add<Burning>(e, {1});
               }
           }) * 1e6 / ops;
}
static void structural() {
    std::printf("\n== structural change: toggle Burning on a random entity (ns per add or remove), N = 100k ==\n");
    std::printf("%-30s %12s %12s\n", "entity has", "sparse set", "archetype");
    std::printf("%-30s %12.1f %12.1f\n", "Pos, Vel, Health (20 B)", toggleCost<SW>(100000, false), toggleCost<AW>(100000, false));
    std::printf("%-30s %12.1f %12.1f\n", "+ 5 x 16 B components (100 B)", toggleCost<SW>(100000, true), toggleCost<AW>(100000, true));
}

// 碎片化：每个实体随机带 k 个标签中的若干个，最多 2^k 张表
template <int K>
static void addTags(AW& aw, SW& sw, Entity a, Entity b, uint32_t bits) {
    if constexpr (K > 0) {
        if (bits & (1u << (K - 1))) aw.add<Tag<K - 1>>(a), sw.add<Tag<K - 1>>(b);
        addTags<K - 1>(aw, sw, a, b, bits);
    }
}
static void frag() {
    std::printf("\n== fragmentation: each entity gets a random subset of k tags, N = 100k, query <Pos, Vel> ==\n");
    std::printf("%-4s %8s %14s %14s %16s\n", "k", "tables", "archetype ns", "sparse ns", "entities/table");
    const int n = 100000;
    for (int k : {0, 2, 4, 6, 8, 10, 12}) {
        Pcg32 r{4};
        AW aw;
        SW sw;
        for (int i = 0; i < n; ++i) {
            Entity a = aw.create(), b = sw.create();
            aw.add<Pos>(a), aw.add<Vel>(a), sw.add<Pos>(b), sw.add<Vel>(b);
            uint32_t bits = k ? r.next() & ((1u << k) - 1) : 0;
            addTags<12>(aw, sw, a, b, bits);
        }
        double ar = bestMs(20, [&] { aw.each<Pos, Vel>([](Entity, Pos& p, Vel& v) { p.x += v.x, p.y += v.y; }); });
        double sp = bestMs(20, [&] { sw.each<Pos, Vel>([](Entity, Pos& p, Vel& v) { p.x += v.x, p.y += v.y; }); });
        size_t tables = 0;
        for (auto& t : aw.tables()) tables += t.size() > 0;
        std::printf("%-4d %8zu %14.2f %14.2f %16.1f\n", k, tables, ar * 1e6 / n, sp * 1e6 / n, double(n) / tables);
    }
}

int main(int argc, char** argv) {
    auto want = [&](const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; };
    if (want("iterate")) iterate();
    if (want("query")) query();
    if (want("struct")) structural();
    if (want("frag")) frag();
}
