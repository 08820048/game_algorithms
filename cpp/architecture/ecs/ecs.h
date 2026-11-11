#pragma once

// ECS（实体 - 组件 - 系统）的两种存储
//   - 实体只是一个编号（+ 代数，见 6.1）；组件是纯数据；系统是"对所有同时拥有 A、B 的实体做某件事"的循环
//   - SparseWorld：每种组件一个稀疏集合（sparse set，EnTT 的做法）。
//       sparse[实体编号] → dense 下标；dense 数组紧凑地存放组件。增删组件 O(1)，
//       多组件查询时遍历最小的那个集合，其余的逐个查表（随机访问）
//   - ArchetypeWorld：组件组合相同的实体放在同一张表里（archetype，Unity DOTS / Flecs 的做法）。
//       每张表每种组件一列，查询直接按列连续遍历；代价是增删组件要把整行搬到另一张表
//   - 两者对外的接口相同：create / destroy / add / remove / has / get / each
//   - each 遍历期间的结构变化（add / remove / destroy）先记下来，遍历结束后再执行（延迟命令）：
//     否则交换删除会让遍历漏掉或重复处理实体，archetype 的表还可能在遍历中重新分配

#include <algorithm>
#include <cstdint>
#include <functional>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ga::ecs {

// #region entity
struct Entity {
    uint32_t index = UINT32_MAX, gen = 0;
    friend bool operator==(Entity, Entity) = default;
};

// 实体编号的分配：FIFO 空闲列表 + 代数（6.1 的句柄）
class Entities {
public:
    Entity create() {
        uint32_t i;
        if (freeHead_ < free_.size()) i = free_[freeHead_++];
        else i = uint32_t(gens_.size()), gens_.push_back(0), alive_.push_back(0);
        alive_[i] = 1;
        ++count_;
        if (freeHead_ > 1024 && freeHead_ * 2 > free_.size()) free_.erase(free_.begin(), free_.begin() + freeHead_), freeHead_ = 0;
        return {i, gens_[i]};
    }
    bool destroy(Entity e) {
        if (!alive(e)) return false;
        alive_[e.index] = 0, ++gens_[e.index], --count_;
        free_.push_back(e.index);
        return true;
    }
    bool alive(Entity e) const { return e.index < gens_.size() && alive_[e.index] && gens_[e.index] == e.gen; }
    uint32_t count() const { return count_; }
    uint32_t gen(uint32_t i) const { return gens_[i]; }
    uint32_t capacity() const { return uint32_t(gens_.size()); }

private:
    std::vector<uint32_t> gens_, free_;
    std::vector<uint8_t> alive_;
    size_t freeHead_ = 0;
    uint32_t count_ = 0;
};
// #endregion

// #region defer
// 遍历中的结构变化放进队列，最外层的遍历结束时统一执行
class Deferred {
protected:
    bool deferring() const { return depth_ > 0; }
    void later(std::function<void()> f) { pending_.push_back(std::move(f)); }
    struct Scope {
        Deferred& d;
        explicit Scope(Deferred& x) : d(x) { ++d.depth_; }
        ~Scope() {
            if (--d.depth_ == 0) {
                for (size_t i = 0; i < d.pending_.size(); ++i) d.pending_[i]();  // 执行中可能追加新的命令
                d.pending_.clear();
            }
        }
    };

private:
    int depth_ = 0;
    std::vector<std::function<void()>> pending_;
};
// #endregion

template <class T, class... Ts>
constexpr int indexOf() {
    int i = 0, r = -1;
    ((std::is_same_v<T, Ts> ? (r = i, ++i) : ++i), ...);
    return r;
}

// #region sparse
template <class T>
class SparseSet {
public:
    static constexpr uint32_t kNone = UINT32_MAX;
    bool has(uint32_t e) const { return e < sparse_.size() && sparse_[e] != kNone; }
    T* get(uint32_t e) { return has(e) ? &data_[sparse_[e]] : nullptr; }
    void insert(uint32_t e, T v) {
        if (e >= sparse_.size()) sparse_.resize(e + 1, kNone);
        if (sparse_[e] != kNone) return void(data_[sparse_[e]] = std::move(v));
        sparse_[e] = uint32_t(dense_.size());
        dense_.push_back(e), data_.push_back(std::move(v));
    }
    void erase(uint32_t e) {  // 交换删除
        if (!has(e)) return;
        uint32_t d = sparse_[e], last = uint32_t(dense_.size() - 1);
        if (d != last) dense_[d] = dense_[last], data_[d] = std::move(data_[last]), sparse_[dense_[d]] = d;
        dense_.pop_back(), data_.pop_back();
        sparse_[e] = kNone;
    }
    size_t size() const { return dense_.size(); }
    const std::vector<uint32_t>& entities() const { return dense_; }
    std::vector<T>& data() { return data_; }

private:
    std::vector<uint32_t> sparse_, dense_;
    std::vector<T> data_;
};

template <class... Cs>
class SparseWorld : Deferred {
public:
    Entity create() { return ents_.create(); }
    bool alive(Entity e) const { return ents_.alive(e); }
    void destroy(Entity e) {
        if (deferring()) return later([this, e] { destroy(e); });
        if (!ents_.alive(e)) return;
        (std::get<SparseSet<Cs>>(pools_).erase(e.index), ...);
        ents_.destroy(e);
    }
    template <class C>
    void add(Entity e, C v = {}) {
        if (deferring()) return later([this, e, v] { add<C>(e, v); });
        if (ents_.alive(e)) pool<C>().insert(e.index, std::move(v));
    }
    template <class C>
    void remove(Entity e) {
        if (deferring()) return later([this, e] { remove<C>(e); });
        if (ents_.alive(e)) pool<C>().erase(e.index);
    }
    template <class C>
    bool has(Entity e) const {
        return ents_.alive(e) && std::get<SparseSet<C>>(pools_).has(e.index);
    }
    template <class C>
    C* get(Entity e) {
        return ents_.alive(e) ? pool<C>().get(e.index) : nullptr;
    }
    template <class C>
    SparseSet<C>& pool() {
        return std::get<SparseSet<C>>(pools_);
    }
    uint32_t count() const { return ents_.count(); }

    // 对同时拥有 Qs... 的实体调用 f(Entity, Qs&...)。遍历最小的集合，其余的查表
    template <class... Qs, class F>
    void each(F&& f) {
        const std::vector<uint32_t>* smallest = nullptr;
        ((smallest = (!smallest || pool<Qs>().size() < smallest->size()) ? &pool<Qs>().entities() : smallest), ...);
        Scope scope(*this);
        for (size_t k = 0; k < smallest->size(); ++k) {
            uint32_t e = (*smallest)[k];
            if ((pool<Qs>().has(e) && ...)) f(Entity{e, ents_.gen(e)}, *pool<Qs>().get(e)...);
        }
    }

private:
    Entities ents_;
    std::tuple<SparseSet<Cs>...> pools_;
};
// #endregion

// #region archetype
template <class... Cs>
class ArchetypeWorld : Deferred {
    static_assert(sizeof...(Cs) <= 32);

public:
    using Mask = uint32_t;
    template <class C>
    static constexpr Mask bit() {
        return Mask(1) << indexOf<C, Cs...>();
    }

    struct Table {
        Mask mask = 0;
        std::vector<uint32_t> entities;
        std::tuple<std::vector<Cs>...> columns;  // 只有 mask 里的列有数据
        size_t size() const { return entities.size(); }
        template <class C>
        std::vector<C>& col() {
            return std::get<std::vector<C>>(columns);
        }
    };

    ArchetypeWorld() { tableFor(0); }

    Entity create() {
        Entity e = ents_.create();
        if (e.index >= records_.size()) records_.resize(e.index + 1);
        Table& t = tables_[0];
        records_[e.index] = {0, uint32_t(t.size())};
        t.entities.push_back(e.index);
        return e;
    }
    bool alive(Entity e) const { return ents_.alive(e); }
    void destroy(Entity e) {
        if (deferring()) return later([this, e] { destroy(e); });
        if (!ents_.alive(e)) return;
        removeRow(records_[e.index].table, records_[e.index].row);
        ents_.destroy(e);
    }
    template <class C>
    void add(Entity e, C v = {}) {
        if (deferring()) return later([this, e, v] { add<C>(e, v); });
        if (!ents_.alive(e)) return;
        Record r = records_[e.index];
        if (tables_[r.table].mask & bit<C>()) return void(tables_[r.table].template col<C>()[r.row] = std::move(v));
        uint32_t to = move(e.index, tables_[r.table].mask | bit<C>());
        tables_[to].template col<C>().push_back(std::move(v));
    }
    template <class C>
    void remove(Entity e) {
        if (deferring()) return later([this, e] { remove<C>(e); });
        if (!ents_.alive(e)) return;
        Record r = records_[e.index];
        if (!(tables_[r.table].mask & bit<C>())) return;
        move(e.index, tables_[r.table].mask & ~bit<C>());
    }
    template <class C>
    bool has(Entity e) const {
        return ents_.alive(e) && (tables_[records_[e.index].table].mask & bit<C>());
    }
    template <class C>
    C* get(Entity e) {
        if (!has<C>(e)) return nullptr;
        Record r = records_[e.index];
        return &tables_[r.table].template col<C>()[r.row];
    }
    uint32_t count() const { return ents_.count(); }
    size_t tableCount() const { return tables_.size(); }
    std::vector<Table>& tables() { return tables_; }
    uint64_t movedRows() const { return moved_; }

    // 对所有包含 Qs... 的表，按列连续遍历。匹配的表列表按查询缓存，有新表时更新
    template <class... Qs, class F>
    void each(F&& f) {
        constexpr Mask need = (bit<Qs>() | ...);
        Scope scope(*this);
        for (uint32_t ti : matching(need)) {
            Table& t = tables_[ti];
            auto cols = std::forward_as_tuple(t.template col<Qs>()...);
            const uint32_t* ents = t.entities.data();
            for (size_t row = 0, n = t.size(); row < n; ++row)
                std::apply([&](auto&... c) { f(Entity{ents[row], ents_.gen(ents[row])}, c[row]...); }, cols);
        }
    }

private:
    struct Record {
        uint32_t table = 0, row = 0;
    };

    uint32_t tableFor(Mask m) {
        auto it = index_.find(m);
        if (it != index_.end()) return it->second;
        tables_.push_back({});
        tables_.back().mask = m;
        index_[m] = uint32_t(tables_.size() - 1);
        return uint32_t(tables_.size() - 1);
    }
    const std::vector<uint32_t>& matching(Mask need) {
        auto& q = queries_[need];
        if (q.seen != tables_.size()) {
            q.tables.clear();
            for (uint32_t i = 0; i < tables_.size(); ++i)
                if ((tables_[i].mask & need) == need) q.tables.push_back(i);
            q.seen = tables_.size();
        }
        return q.tables;
    }
    // 把实体的整行搬到 newMask 对应的表，两张表都有的列逐个复制；新增的列由调用方补上
    uint32_t move(uint32_t e, Mask newMask) {
        Record r = records_[e];
        uint32_t to = tableFor(newMask);
        Table& src = tables_[r.table];
        Table& dst = tables_[to];
        Mask common = src.mask & newMask;
        ((common & bit<Cs>() ? (dst.template col<Cs>().push_back(std::move(src.template col<Cs>()[r.row])), 0) : 0), ...);
        dst.entities.push_back(e);
        records_[e] = {to, uint32_t(dst.size() - 1)};
        removeRow(r.table, r.row);
        ++moved_;
        return to;
    }
    // 交换删除表中的一行
    void removeRow(uint32_t ti, uint32_t row) {
        Table& t = tables_[ti];
        uint32_t last = uint32_t(t.size() - 1);
        auto fix = [&](auto& col) {
            if (col.size() <= row) return;  // 这一列不在表里（或者是刚搬走、还没补齐的新列）
            if (row != last) col[row] = std::move(col[last]);
            col.pop_back();
        };
        ((t.mask & bit<Cs>() ? fix(t.template col<Cs>()) : void()), ...);
        if (row != last) t.entities[row] = t.entities[last], records_[t.entities[row]].row = row;
        t.entities.pop_back();
    }

    struct Query {
        size_t seen = 0;
        std::vector<uint32_t> tables;
    };
    Entities ents_;
    std::vector<Record> records_;
    std::vector<Table> tables_;
    std::unordered_map<Mask, uint32_t> index_;
    std::unordered_map<Mask, Query> queries_;
    uint64_t moved_ = 0;
};
// #endregion

}  // namespace ga::ecs
