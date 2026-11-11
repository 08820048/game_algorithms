#pragma once

// 对象池与内存布局
//   - Pool：固定的槽位数组 + 空闲列表。创建 / 销毁都是 O(1)、不调用 new / delete，对象的地址不变。
//     遍历要跳过空槽：存活率低时，遍历的代价取决于容量而不是存活数
//   - Handle = 槽位编号 + 代数（generation）。槽位每被释放一次代数加一；持有旧句柄的人拿它取对象时，
//     代数对不上就返回空，而不是悄悄拿到"住进同一个槽位的新对象"
//   - 代数会回绕：b 位的代数，同一个槽位被重用 2^b 次后，旧句柄又"有效"了。
//     空闲列表用 LIFO（刚释放的槽位最先被重用）时最容易发生；FIFO 让重用分散到所有槽位
//   - SlotMap：对象紧凑地存放在 dense 数组里（删除时交换删除），句柄经过一层间接指向它。
//     遍历是连续的，代价是对象会移动（不能长期持有指针），按句柄访问多一次间接
//   - CacheSim：组相联 LRU 缓存模型，用来数"遍历时读进了多少条缓存行"

#include <cstddef>
#include <cstdint>
#include <new>
#include <utility>
#include <vector>

namespace ga::pool {

// #region handle
template <class Gen = uint32_t>
struct BasicHandle {
    uint32_t index = UINT32_MAX;
    Gen gen = 0;
    friend bool operator==(BasicHandle, BasicHandle) = default;
};
using Handle = BasicHandle<uint32_t>;
// #endregion

enum class FreeOrder { Lifo, Fifo };

// #region pool
template <class T, class Gen = uint32_t>
class Pool {
public:
    using H = BasicHandle<Gen>;
    static constexpr uint32_t kNone = UINT32_MAX;

    explicit Pool(uint32_t capacity, FreeOrder order = FreeOrder::Lifo) : slots_(capacity), order_(order) {
        // 一开始所有槽位都空闲，按编号串成链表
        for (uint32_t i = 0; i < capacity; ++i) slots_[i].next = i + 1 < capacity ? i + 1 : kNone;
        head_ = capacity ? 0 : kNone;
        tail_ = capacity ? capacity - 1 : kNone;
    }
    ~Pool() {
        for (Slot& s : slots_)
            if (s.alive) s.ptr()->~T();
    }
    Pool(const Pool&) = delete;
    Pool& operator=(const Pool&) = delete;

    // 满了返回无效句柄（index = kNone）
    template <class... Args>
    H create(Args&&... args) {
        if (head_ == kNone) return {};
        uint32_t i = head_;
        head_ = slots_[i].next;
        if (head_ == kNone) tail_ = kNone;
        Slot& s = slots_[i];
        new (s.storage) T(std::forward<Args>(args)...);
        s.alive = true;
        ++size_;
        return {i, s.gen};
    }
    bool destroy(H h) {
        if (!valid(h)) return false;
        Slot& s = slots_[h.index];
        s.ptr()->~T();
        s.alive = false;
        ++s.gen;  // 旧句柄从此失效（直到回绕）
        --size_;
        // 放回空闲列表：LIFO 放到队头，FIFO 放到队尾
        if (order_ == FreeOrder::Lifo || head_ == kNone) {
            s.next = head_;
            head_ = h.index;
            if (tail_ == kNone) tail_ = h.index;
        } else {
            s.next = kNone;
            slots_[tail_].next = h.index;
            tail_ = h.index;
        }
        return true;
    }
    bool valid(H h) const { return h.index < slots_.size() && slots_[h.index].alive && slots_[h.index].gen == h.gen; }
    T* get(H h) { return valid(h) ? slots_[h.index].ptr() : nullptr; }
    // 只按编号取，不检查代数：模拟"存下标 / 裸指针"，演示悬空引用
    T* getUnchecked(uint32_t index) { return index < slots_.size() && slots_[index].alive ? slots_[index].ptr() : nullptr; }

    template <class F>
    void forEach(F&& f) {
        for (Slot& s : slots_)
            if (s.alive) f(*s.ptr());
    }
    uint32_t size() const { return size_; }
    uint32_t capacity() const { return uint32_t(slots_.size()); }
    bool alive(uint32_t i) const { return slots_[i].alive; }
    Gen generation(uint32_t i) const { return slots_[i].gen; }
    uint32_t freeHead() const { return head_; }
    uint32_t nextFree(uint32_t i) const { return slots_[i].next; }

private:
    struct Slot {
        alignas(T) unsigned char storage[sizeof(T)];
        uint32_t next = kNone;  // 空闲时：下一个空闲槽位
        Gen gen = 0;
        bool alive = false;
        T* ptr() { return std::launder(reinterpret_cast<T*>(storage)); }
    };
    std::vector<Slot> slots_;
    uint32_t head_ = kNone, tail_ = kNone, size_ = 0;
    FreeOrder order_;
};
// #endregion

// #region slotmap
template <class T>
class SlotMap {
public:
    static constexpr uint32_t kNone = UINT32_MAX;

    template <class... Args>
    Handle insert(Args&&... args) {
        uint32_t s;
        if (freeHead_ != kNone) s = freeHead_, freeHead_ = sparse_[s].next;
        else s = uint32_t(sparse_.size()), sparse_.push_back({});
        sparse_[s].dense = uint32_t(dense_.size());
        dense_.emplace_back(std::forward<Args>(args)...);
        owner_.push_back(s);
        return {s, sparse_[s].gen};
    }
    bool erase(Handle h) {
        if (!valid(h)) return false;
        uint32_t d = sparse_[h.index].dense, last = uint32_t(dense_.size() - 1);
        if (d != last) {  // 交换删除：把最后一个搬到 d，更新它的槽位
            dense_[d] = std::move(dense_[last]);
            owner_[d] = owner_[last];
            sparse_[owner_[d]].dense = d;
        }
        dense_.pop_back(), owner_.pop_back();
        Sparse& sp = sparse_[h.index];
        sp.dense = kNone, ++sp.gen;
        sp.next = freeHead_, freeHead_ = h.index;
        return true;
    }
    bool valid(Handle h) const { return h.index < sparse_.size() && sparse_[h.index].dense != kNone && sparse_[h.index].gen == h.gen; }
    T* get(Handle h) { return valid(h) ? &dense_[sparse_[h.index].dense] : nullptr; }
    // 连续遍历
    T* begin() { return dense_.data(); }
    T* end() { return dense_.data() + dense_.size(); }
    size_t size() const { return dense_.size(); }

private:
    struct Sparse {
        uint32_t dense = kNone, gen = 0, next = kNone;
    };
    std::vector<T> dense_;
    std::vector<uint32_t> owner_;  // dense[i] 属于哪个槽位
    std::vector<Sparse> sparse_;
    uint32_t freeHead_ = kNone;
};
// #endregion

// #region padding
// 成员按声明顺序排列，每个成员要对齐到自己的大小：小成员夹在大成员中间会留下空洞
struct Scattered {
    bool alive;    // 1 + 7 字节空洞
    double x;      // 8
    bool visible;  // 1 + 7 字节空洞
    double y;      // 8
    int id;        // 4 + 4 字节尾部填充（数组里下一个元素的 double 要对齐）
};
struct Sorted {  // 同样的成员，从大到小排
    double x, y;
    int id;
    bool alive, visible;  // 4 + 1 + 1 + 2 字节尾部填充
};
static_assert(sizeof(Scattered) == 40 && sizeof(Sorted) == 24);
// #endregion

// #region cachesim
// 组相联、LRU 替换的缓存模型。access(地址) 返回是否命中
class CacheSim {
public:
    CacheSim(uint32_t lineBytes, uint32_t lines, uint32_t ways) : line_(lineBytes), ways_(ways), sets_(lines / ways) {
        tags_.assign(size_t(sets_) * ways_, UINT64_MAX);
        age_.assign(size_t(sets_) * ways_, 0);
    }
    bool access(uint64_t addr) {
        uint64_t block = addr / line_;
        uint32_t set = uint32_t(block % sets_);
        size_t base = size_t(set) * ways_;
        ++clock_;
        size_t victim = base;
        for (size_t w = base; w < base + ways_; ++w) {
            if (tags_[w] == block) {
                age_[w] = clock_;
                ++hits_;
                return true;
            }
            if (age_[w] < age_[victim]) victim = w;  // 最久没用过的（空位的 age 为 0，优先）
        }
        tags_[victim] = block, age_[victim] = clock_;
        ++misses_;
        return false;
    }
    uint64_t hits() const { return hits_; }
    uint64_t misses() const { return misses_; }
    uint32_t lineBytes() const { return line_; }
    void reset() {
        tags_.assign(tags_.size(), UINT64_MAX), age_.assign(age_.size(), 0);
        hits_ = misses_ = clock_ = 0;
    }

private:
    uint32_t line_, ways_, sets_;
    std::vector<uint64_t> tags_, age_;
    uint64_t hits_ = 0, misses_ = 0, clock_ = 0;
};
// #endregion

}  // namespace ga::pool
