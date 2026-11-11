#include "pool.h"

#include <algorithm>
#include <cstdio>
#include <list>
#include <map>
#include <set>
#include <vector>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;
using namespace ga::pool;

// 统计构造 / 析构次数的对象：池必须给每个创建的对象恰好调用一次析构
struct Tracked {
    static inline int live = 0;
    int value;
    explicit Tracked(int v) : value(v) { ++live; }
    Tracked(Tracked&& o) noexcept : value(o.value) { ++live; }
    Tracked& operator=(Tracked&& o) noexcept { value = o.value; return *this; }
    ~Tracked() { --live; }
};

// 与参照（map：句柄 → 值）对照的随机增删
static void testPoolRandom(FreeOrder order) {
    Pcg32 r{order == FreeOrder::Lifo ? 1u : 2u};
    {
        Pool<Tracked> p(200, order);
        std::map<std::pair<uint32_t, uint32_t>, int> ref;
        std::vector<Handle> dead;  // 已经销毁的句柄
        for (int step = 0; step < 50000; ++step) {
            bool doCreate = ref.empty() || (r.next() % 100 < 52 && ref.size() < 200);
            if (doCreate) {
                int v = int(r.next() % 1000000);
                Handle h = p.create(v);
                CHECK(h.index != Pool<Tracked>::kNone);
                CHECK(!ref.count({h.index, h.gen}));
                ref[{h.index, h.gen}] = v;
            } else {
                auto it = ref.begin();
                std::advance(it, r.next() % ref.size());
                Handle h{it->first.first, it->first.second};
                CHECK(p.get(h) && p.get(h)->value == it->second);
                CHECK(p.destroy(h));
                CHECK(!p.destroy(h));  // 重复销毁被拒绝
                dead.push_back(h);
                ref.erase(it);
            }
            CHECK(p.size() == ref.size());
            CHECK(Tracked::live == int(ref.size()));
            if (step % 500 == 0) {
                // 每个旧句柄都失效（32 位代数，不会回绕）
                for (Handle h : dead) CHECK(p.get(h) == nullptr);
                // forEach 恰好访问存活的对象
                std::multiset<int> a, b;
                p.forEach([&](Tracked& t) { a.insert(t.value); });
                for (auto& [k, v] : ref) b.insert(v);
                CHECK(a == b);
            }
        }
        // 满了返回无效句柄
        while (p.size() < p.capacity()) p.create(0);
        CHECK(p.create(1).index == Pool<Tracked>::kNone);
    }
    CHECK(Tracked::live == 0);  // 池析构时销毁剩下的对象
}

// LIFO：刚释放的槽位最先被重用；FIFO：最早释放的最先被重用
static void testFreeOrder() {
    Pool<int> lifo(8, FreeOrder::Lifo), fifo(8, FreeOrder::Fifo);
    std::vector<Handle> a, b;
    for (int i = 0; i < 8; ++i) a.push_back(lifo.create(i)), b.push_back(fifo.create(i));
    for (int i : {2, 5, 1}) lifo.destroy(a[i]), fifo.destroy(b[i]);
    CHECK(lifo.create(0).index == 1 && lifo.create(0).index == 5 && lifo.create(0).index == 2);
    CHECK(fifo.create(0).index == 2 && fifo.create(0).index == 5 && fifo.create(0).index == 1);
    // 从未用过的槽位：代数为 0 的句柄也不能有效（否则会拿到未构造的内存）
    Pool<int> fresh(16);
    fresh.create(1);
    for (uint32_t i = 1; i < 16; ++i) CHECK(!fresh.valid({i, 0}) && fresh.get({i, 0}) == nullptr && !fresh.destroy({i, 0}));
    // 刚构造时按编号顺序分配
    Pool<int> p(4);
    for (uint32_t i = 0; i < 4; ++i) CHECK(p.create(0).index == i);
}

// 代数回绕：8 位代数，同一个槽位重用 256 次后旧句柄"复活"。FIFO 把重用分散到所有槽位，回绕晚得多
static void testWrap() {
    Pool<int, uint8_t> p(1);
    auto old = p.create(7);
    p.destroy(old);
    int revived = -1;
    for (int k = 1; k <= 300 && revived < 0; ++k) {
        auto h = p.create(k);
        if (p.valid(old)) revived = k;
        p.destroy(h);
    }
    CHECK(revived == 256);

    // 容量 64、保持 32 个存活、随机销毁一个再创建一个：旧句柄第一次复活前经过了多少次创建
    for (FreeOrder order : {FreeOrder::Lifo, FreeOrder::Fifo}) {
        Pcg32 r{9};
        Pool<int, uint8_t> q(64, order);
        std::vector<BasicHandle<uint8_t>> liveH;
        for (int i = 0; i < 32; ++i) liveH.push_back(q.create(i));
        auto victim = liveH[0];
        q.destroy(victim);
        liveH.erase(liveH.begin());
        liveH.push_back(q.create(0));
        long creates = 1, first = -1;
        while (first < 0 && creates < 1000000) {
            size_t k = r.next() % liveH.size();
            q.destroy(liveH[k]);
            liveH[k] = q.create(0);
            ++creates;
            if (q.valid(victim)) first = creates;
        }
        std::printf("8-bit wrap, %s: stale handle revived after %ld creates\n", order == FreeOrder::Lifo ? "LIFO" : "FIFO", first);
        if (order == FreeOrder::Fifo) CHECK(first >= 256 * 32);  // FIFO：所有空闲槽位轮流用，每 32 次创建才轮到同一个
        else CHECK(first > 0 && first < 256 * 32);
    }
}

static void testSlotMap() {
    Pcg32 r{3};
    {
        SlotMap<Tracked> m;
        std::map<std::pair<uint32_t, uint32_t>, int> ref;
        std::vector<Handle> dead;
        size_t peak = 0;
        for (int step = 0; step < 50000; ++step) {
            if (ref.empty() || r.next() % 100 < 55) {
                int v = int(r.next() % 1000000);
                Handle h = m.insert(v);
                CHECK(!ref.count({h.index, h.gen}));
                ref[{h.index, h.gen}] = v;
            } else {
                auto it = ref.begin();
                std::advance(it, r.next() % ref.size());
                Handle h{it->first.first, it->first.second};
                CHECK(m.get(h) && m.get(h)->value == it->second);
                CHECK(m.erase(h) && !m.erase(h));
                dead.push_back(h);
                ref.erase(it);
            }
            CHECK(m.size() == ref.size());
            if (step % 500 == 0) {
                for (Handle h : dead) CHECK(m.get(h) == nullptr);
                for (auto& [k, v] : ref) CHECK(m.get({k.first, k.second}) && m.get({k.first, k.second})->value == v);
                // 连续存放：begin..end 恰好是所有存活的对象
                std::multiset<int> a, b;
                for (Tracked* t = m.begin(); t != m.end(); ++t) a.insert(t->value);
                for (auto& [k, v] : ref) b.insert(v);
                CHECK(a == b);
            }
            CHECK(Tracked::live == int(ref.size()));
            peak = std::max(peak, ref.size());
        }
        // 槽位会被重用：槽位数不超过同时存活的峰值
        Handle h = m.insert(0);
        CHECK(h.index <= peak);
        // 空出来的槽位，即使代数"猜对了"也不能有效
        CHECK(m.erase(h));
        CHECK(!m.valid({h.index, h.gen + 1}) && m.get({h.index, h.gen + 1}) == nullptr);
    }
    CHECK(Tracked::live == 0);
}

// 参照：每组一个 std::list 做 LRU
static void testCacheSim() {
    Pcg32 r{4};
    for (auto [line, lines, ways] : {std::tuple{64u, 64u, 4u}, {128u, 32u, 8u}, {64u, 16u, 16u}, {32u, 8u, 1u}}) {
        CacheSim c(line, lines, ways);
        uint32_t sets = lines / ways;
        std::vector<std::list<uint64_t>> ref(sets);
        long hits = 0;
        for (int i = 0; i < 200000; ++i) {
            // 混合：顺序扫描 + 随机地址，命中和不命中都很多
            uint64_t addr = (i % 3 == 0) ? r.next() % (line * lines * 4) : uint64_t(i) * 8 % (line * lines * 2);
            uint64_t b = addr / line;
            auto& l = ref[b % sets];
            bool hit = false;
            for (auto it = l.begin(); it != l.end(); ++it)
                if (*it == b) {
                    l.erase(it), hit = true;
                    break;
                }
            l.push_front(b);
            if (l.size() > ways) l.pop_back();
            hits += hit;
            CHECK(c.access(addr) == hit);
        }
        CHECK(long(c.hits()) == hits && c.hits() + c.misses() == 200000);
        CHECK(hits > 10000 && hits < 190000);
    }
}

int main() {
    testPoolRandom(FreeOrder::Lifo);
    testPoolRandom(FreeOrder::Fifo);
    testFreeOrder();
    testWrap();
    testSlotMap();
    testCacheSim();
    return report("pool_test");
}
