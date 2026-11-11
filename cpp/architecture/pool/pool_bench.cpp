// 对象池与内存布局的测量
//   c++ -std=c++20 -O2 -Icpp/common cpp/architecture/pool/pool_bench.cpp -o /tmp/poolb && /tmp/poolb [section]
//   section: alloc | iterate | lookup | stride | share | wrap（默认全部）

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <list>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include "../../foundations/random/random.h"
#include "pool.h"

using namespace ga;
using namespace ga::pool;

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
static volatile uint64_t sink;

// 64 字节的游戏对象：位置、速度（热）+ 其余（冷）
struct Entity {
    float x = 0, y = 0, vx = 1, vy = 1;
    float cold[12] = {};
    Entity() = default;
    explicit Entity(float v) : x(v), y(v) {}
};
static_assert(sizeof(Entity) == 64);

// 稳态 N 个对象，每次随机销毁一个、再创建一个
static void alloc() {
    std::printf("== churn: destroy one + create one, 64-byte objects (ns per pair) ==\n");
    std::printf("%-8s %10s %10s %10s %10s\n", "N", "new/delete", "Pool LIFO", "Pool FIFO", "SlotMap");
    for (int n : {1000, 100000, 1000000}) {
        const int ops = 2000000;
        Pcg32 r{1};
        std::vector<uint32_t> picks(ops);
        for (auto& p : picks) p = r.next() % uint32_t(n);
        double t[4];
        {
            std::vector<std::unique_ptr<Entity>> v;
            for (int i = 0; i < n; ++i) v.push_back(std::make_unique<Entity>(float(i)));
            t[0] = bestMs(3, [&] {
                for (uint32_t k : picks) v[k] = std::make_unique<Entity>(1.0f);
            });
        }
        for (int m = 0; m < 2; ++m) {
            Pool<Entity> p(uint32_t(n), m == 0 ? FreeOrder::Lifo : FreeOrder::Fifo);
            std::vector<Handle> h;
            for (int i = 0; i < n; ++i) h.push_back(p.create(float(i)));
            t[1 + m] = bestMs(3, [&] {
                for (uint32_t k : picks) p.destroy(h[k]), h[k] = p.create(1.0f);
            });
        }
        {
            SlotMap<Entity> s;
            std::vector<Handle> h;
            for (int i = 0; i < n; ++i) h.push_back(s.insert(float(i)));
            t[3] = bestMs(3, [&] {
                for (uint32_t k : picks) s.erase(h[k]), h[k] = s.insert(1.0f);
            });
        }
        std::printf("%-8d %10.1f %10.1f %10.1f %10.1f\n", n, t[0] * 1e6 / ops, t[1] * 1e6 / ops, t[2] * 1e6 / ops, t[3] * 1e6 / ops);
    }
}

// 遍历所有存活对象，更新位置（x += vx 等）
static void iterate() {
    std::printf("\n== iterate live objects, x += vx (ns per live object) ==\n");
    std::printf("%-8s %-28s %10s\n", "N", "layout", "ns/object");
    for (int n : {10000, 100000, 1000000}) {
        Pcg32 r{2};
        auto row = [&](const char* name, double ms) { std::printf("%-8d %-28s %10.2f\n", n, name, ms * 1e6 / n); };
        {
            std::vector<Entity> v(n);
            row("vector<Entity>", bestMs(10, [&] { for (auto& e : v) e.x += e.vx, e.y += e.vy; }));
        }
        {
            std::vector<std::unique_ptr<Entity>> v;
            for (int i = 0; i < n; ++i) v.push_back(std::make_unique<Entity>());
            row("unique_ptr, allocation order", bestMs(10, [&] { for (auto& e : v) e->x += e->vx, e->y += e->vy; }));
            // 模拟运行一段时间后的堆：随机销毁、重建，指针的顺序和地址的顺序不再一致
            for (int k = 0; k < 4 * n; ++k) v[r.next() % n] = std::make_unique<Entity>();
            row("unique_ptr, after churn", bestMs(10, [&] { for (auto& e : v) e->x += e->vx, e->y += e->vy; }));
            std::shuffle(v.begin(), v.end(), std::mt19937(5));
            row("unique_ptr, shuffled", bestMs(10, [&] { for (auto& e : v) e->x += e->vx, e->y += e->vy; }));
        }
        {
            std::list<Entity> l(n);
            row("std::list", bestMs(10, [&] { for (auto& e : l) e.x += e.vx, e.y += e.vy; }));
        }
        for (int occ : {100, 50, 10}) {
            // 容量 n·100/occ，存活 n 个，随机分布
            uint32_t cap = uint32_t(int64_t(n) * 100 / occ);
            Pool<Entity> p(cap);
            std::vector<Handle> h;
            for (uint32_t i = 0; i < cap; ++i) h.push_back(p.create());
            std::shuffle(h.begin(), h.end(), std::mt19937(7));
            for (uint32_t i = uint32_t(n); i < cap; ++i) p.destroy(h[i]);
            char name[40];
            std::snprintf(name, sizeof name, "Pool, %d%% occupied", occ);
            row(name, bestMs(10, [&] { p.forEach([](Entity& e) { e.x += e.vx, e.y += e.vy; }); }));
        }
        {
            // 同样 50% 存活，但空槽集中在后半段：分支预测几乎总是对的
            Pool<Entity> p{uint32_t(2 * n)};
            std::vector<Handle> h;
            for (int i = 0; i < 2 * n; ++i) h.push_back(p.create());
            for (int i = n; i < 2 * n; ++i) p.destroy(h[i]);
            row("Pool, 50%, holes together", bestMs(10, [&] { p.forEach([](Entity& e) { e.x += e.vx, e.y += e.vy; }); }));
        }
        {
            SlotMap<Entity> s;
            std::vector<Handle> h;
            for (int i = 0; i < 2 * n; ++i) h.push_back(s.insert());
            for (int i = 0; i < 2 * n; i += 2) s.erase(h[i]);  // 删掉一半：dense 仍然连续
            row("SlotMap (dense)", bestMs(10, [&] { for (Entity* e = s.begin(); e != s.end(); ++e) e->x += e->vx, e->y += e->vy; }));
        }
    }
}

// 随机按引用访问：裸指针 / Pool 句柄 / SlotMap 句柄
static void lookup() {
    std::printf("\n== random access by reference (ns per access) ==\n");
    std::printf("%-8s %12s %12s %12s\n", "N", "raw pointer", "Pool handle", "SlotMap");
    for (int n : {1000, 100000, 1000000}) {
        Pcg32 r{3};
        const int q = 4000000;
        std::vector<uint32_t> idx(q);
        for (auto& i : idx) i = r.next() % uint32_t(n);
        Pool<Entity> p{uint32_t(n)};
        SlotMap<Entity> s;
        std::vector<Handle> hp, hs;
        std::vector<Entity*> ptr;
        for (int i = 0; i < n; ++i) hp.push_back(p.create(float(i))), hs.push_back(s.insert(float(i))), ptr.push_back(p.get(hp.back()));
        // SlotMap 删一半再插回去，让 dense 的顺序和槽位的顺序不同（真实的使用状态）
        for (int i = 0; i < n; i += 2) s.erase(hs[i]);
        for (int i = 0; i < n; i += 2) hs[i] = s.insert(float(i));
        double a = bestMs(3, [&] { float t = 0; for (uint32_t i : idx) t += ptr[i]->x; sink = uint64_t(t); });
        double b = bestMs(3, [&] { float t = 0; for (uint32_t i : idx) if (Entity* e = p.get(hp[i])) t += e->x; sink = uint64_t(t); });
        double c = bestMs(3, [&] { float t = 0; for (uint32_t i : idx) if (Entity* e = s.get(hs[i])) t += e->x; sink = uint64_t(t); });
        std::printf("%-8d %12.2f %12.2f %12.2f\n", n, a * 1e6 / q, b * 1e6 / q, c * 1e6 / q);
    }
}

// 每隔 stride 字节读一个字节，读完 256 MB 的缓冲区。stride 不超过缓存行时，读进的缓存行数相同
static void stride() {
    std::printf("\n== stride: touch one byte every S bytes of a 256 MB buffer ==\n");
    std::printf("%-8s %12s %12s %14s\n", "S", "accesses", "total ms", "ns/access");
    const size_t bytes = size_t(256) << 20;
    std::vector<uint8_t> buf(bytes, 1);
    for (size_t s : {4, 8, 16, 32, 64, 128, 256, 512, 1024}) {
        double ms = bestMs(5, [&] {
            uint64_t t = 0;
            for (size_t i = 0; i < bytes; i += s) t += buf[i];
            sink = t;
        });
        std::printf("%-8zu %12zu %12.2f %14.3f\n", s, bytes / s, ms, ms * 1e6 / double(bytes / s));
    }
}

// 伪共享：4 个线程各自累加自己的计数器，计数器之间相隔 spacing 字节
static void share() {
    std::printf("\n== false sharing: 4 threads, each increments its own counter 50M times ==\n");
    std::printf("%-10s %10s\n", "spacing", "ms");
    const int threads = 4;
    const long iters = 50000000;
    for (size_t spacing : {4, 8, 32, 64, 128, 256}) {
        alignas(256) static uint8_t mem[256 * 8];
        double ms = bestMs(3, [&] {
            std::memset(mem, 0, sizeof mem);
            std::vector<std::thread> ts;
            for (int t = 0; t < threads; ++t)
                ts.emplace_back([&, t] {
                    auto* c = reinterpret_cast<std::atomic<uint32_t>*>(mem + t * spacing);
                    for (long i = 0; i < iters; ++i) c->fetch_add(1, std::memory_order_relaxed);
                });
            for (auto& th : ts) th.join();
        });
        std::printf("%-10zu %10.1f\n", spacing, ms);
    }
}

// 代数回绕：旧句柄"复活"前经过多少次创建
static void wrap() {
    std::printf("\n== generation wrap-around: creates until a stale handle validates again ==\n");
    std::printf("%-28s %-6s %14s %14s\n", "workload", "order", "8-bit gen", "16-bit gen");
    auto run = [](auto genTag, FreeOrder order, bool hotBullet) -> long {
        using G = decltype(genTag);
        Pcg32 r{11};
        Pool<int, G> p(1024, order);
        std::vector<BasicHandle<G>> live;
        for (int i = 0; i < 512; ++i) live.push_back(p.create(i));
        BasicHandle<G> stale;
        if (hotBullet) {  // 512 个长寿的对象 + 一颗子弹每帧创建、下一帧销毁
            stale = p.create(0);
            p.destroy(stale);
            for (long c = 1; c < 50000000; ++c) {
                auto b = p.create(0);
                if (p.valid(stale)) return c;
                p.destroy(b);
            }
        } else {  // 随机销毁一个、创建一个
            stale = live[0];
            p.destroy(stale);
            live[0] = p.create(0);
            for (long c = 1; c < 50000000; ++c) {
                size_t k = r.next() % live.size();
                p.destroy(live[k]);
                live[k] = p.create(0);
                if (p.valid(stale)) return c;
            }
        }
        return -1;
    };
    for (bool hot : {false, true})
        for (FreeOrder o : {FreeOrder::Lifo, FreeOrder::Fifo})
            std::printf("%-28s %-6s %14ld %14ld\n", hot ? "512 long-lived + 1 bullet" : "random churn, 512 of 1024",
                        o == FreeOrder::Lifo ? "LIFO" : "FIFO", run(uint8_t{}, o, hot), run(uint16_t{}, o, hot));
}

int main(int argc, char** argv) {
    auto want = [&](const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; };
    if (want("alloc")) alloc();
    if (want("iterate")) iterate();
    if (want("lookup")) lookup();
    if (want("stride")) stride();
    if (want("share")) share();
    if (want("wrap")) wrap();
}
