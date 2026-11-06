#pragma once

// 宽相检测：均匀网格与空间哈希
//   - bruteForcePairs：两两测试，O(n²)，作为对照
//   - NaiveGrid：最直接的网格写法（每格一个 vector，结果排序去重），用来对比
//   - UniformGrid / SpatialHash：同一套实现，只是"格子 → 存储位置"的映射不同
//       * 每帧用计数排序重建，所有格子的内容存放在一个连续数组里
//       * 用"参考格子"去重：一对物体只在一个格子里被报告，不需要额外的集合

#include <algorithm>
#include <cmath>
#include <compare>
#include <cstdint>
#include <vector>

#include "../intersect/intersect.h"

namespace ga {

struct Pair {
    int a, b;  // a < b
    friend auto operator<=>(const Pair&, const Pair&) = default;
};

// #region brute
// 两两测试：n 个物体要做 n(n-1)/2 次 AABB 测试。返回做了多少次测试
inline long bruteForcePairs(const std::vector<Aabb>& boxes, std::vector<Pair>& out) {
    out.clear();
    int n = static_cast<int>(boxes.size());
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (overlaps(boxes[i], boxes[j])) out.push_back({i, j});
    return static_cast<long>(n) * (n - 1) / 2;
}
// #endregion

// 一个盒子覆盖的格子范围（闭区间）
struct CellRange {
    int x0, y0, x1, y1;
};

// #region naive
// 第一版网格：每个格子一个列表。物体放进它覆盖的每个格子，再在每个格子内部两两测试。
// 跨越多个格子的两个物体会在多个格子里相遇，所以最后要排序去重
class NaiveGrid {
public:
    NaiveGrid(Aabb world, float cellSize)
        : origin_(world.min), inv_(1.0f / cellSize),
          cols_(std::max(1, static_cast<int>(std::ceil((world.max.x - world.min.x) * inv_)))),
          rows_(std::max(1, static_cast<int>(std::ceil((world.max.y - world.min.y) * inv_)))),
          cells_(static_cast<size_t>(cols_) * rows_) {}

    long findPairs(const std::vector<Aabb>& boxes, std::vector<Pair>& out) {
        for (auto& cell : cells_) cell.clear();
        for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
            CellRange r = range(boxes[i]);
            for (int y = r.y0; y <= r.y1; ++y)
                for (int x = r.x0; x <= r.x1; ++x) cells_[y * cols_ + x].push_back(i);
        }
        out.clear();
        long tests = 0;
        for (const auto& cell : cells_)
            for (size_t k = 0; k < cell.size(); ++k)
                for (size_t l = k + 1; l < cell.size(); ++l, ++tests)
                    if (overlaps(boxes[cell[k]], boxes[cell[l]])) out.push_back({cell[k], cell[l]});
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        return tests;
    }

private:
    int clampCell(float v, int count) const {
        return static_cast<int>(std::clamp(std::floor(v * inv_), 0.0f, static_cast<float>(count - 1)));
    }
    CellRange range(const Aabb& b) const {
        return {clampCell(b.min.x - origin_.x, cols_), clampCell(b.min.y - origin_.y, rows_),
                clampCell(b.max.x - origin_.x, cols_), clampCell(b.max.y - origin_.y, rows_)};
    }

    Vec2 origin_;
    float inv_;
    int cols_, rows_;
    std::vector<std::vector<int>> cells_;
};
// #endregion

// #region indexers
// 均匀网格：世界有边界，格子按行编号。超出边界的部分被"夹"进最外圈的格子，结果仍然正确，只是变慢
struct GridIndexer {
    Vec2 origin;
    float inv;  // 1 / 格子边长
    int cols, rows;

    GridIndexer(Aabb world, float cellSize)
        : origin(world.min), inv(1.0f / cellSize),
          cols(std::max(1, static_cast<int>(std::ceil((world.max.x - world.min.x) * inv)))),
          rows(std::max(1, static_cast<int>(std::ceil((world.max.y - world.min.y) * inv)))) {}

    int slots() const { return cols * rows; }
    CellRange range(const Aabb& b) const {
        return {clampCell(b.min.x - origin.x, cols), clampCell(b.min.y - origin.y, rows),
                clampCell(b.max.x - origin.x, cols), clampCell(b.max.y - origin.y, rows)};
    }
    int slot(int x, int y) const { return y * cols + x; }

private:
    int clampCell(float v, int count) const {
        return static_cast<int>(std::clamp(std::floor(v * inv), 0.0f, static_cast<float>(count - 1)));
    }
};

// 空间哈希：世界没有边界，格子坐标经过哈希落进固定数量的桶里。
// 不同的格子可能落进同一个桶，这只会多出一些候选对，不影响正确性
struct HashIndexer {
    float inv;
    int buckets;

    HashIndexer(float cellSize, int bucketCount) : inv(1.0f / cellSize), buckets(std::max(1, bucketCount)) {}

    int slots() const { return buckets; }
    CellRange range(const Aabb& b) const {
        return {cell(b.min.x), cell(b.min.y), cell(b.max.x), cell(b.max.y)};
    }
    int slot(int x, int y) const {
        // Teschner 等人（2003）使用的哈希：两个大质数相乘后异或
        uint32_t h = (static_cast<uint32_t>(x) * 73856093u) ^ (static_cast<uint32_t>(y) * 19349663u);
        return static_cast<int>(h % static_cast<uint32_t>(buckets));
    }

private:
    int cell(float v) const { return static_cast<int>(std::floor(v * inv)); }
};
// #endregion

template <class Indexer>
class CellBroadphase {
public:
    explicit CellBroadphase(Indexer indexer)
        : idx_(indexer), start_(static_cast<size_t>(indexer.slots()) + 1), stamp_(static_cast<size_t>(indexer.slots())) {}

    const Indexer& indexer() const { return idx_; }
    CellRange range(const Aabb& b) const { return idx_.range(b); }

    // #region build
    // 计数排序：第一遍数出每个格子有几个物体，求前缀和得到每个格子在数组里的位置，
    // 第二遍把物体编号填进去。所有格子的内容连续存放在 entries_ 里，没有一次小块内存分配
    void build(const std::vector<Aabb>& boxes) {
        boxes_ = &boxes;
        int n = static_cast<int>(boxes.size());
        std::fill(start_.begin(), start_.end(), 0);
        std::fill(stamp_.begin(), stamp_.end(), -1);
        for (int i = 0; i < n; ++i) forEachSlot(i, [&](int s) { ++start_[s]; });
        for (size_t s = 1; s < start_.size(); ++s) start_[s] += start_[s - 1];  // start_[s] = 第 s 格的结尾
        entries_.resize(start_.back());
        std::fill(stamp_.begin(), stamp_.end(), -1);
        // 倒着填：每填一个，结尾往前挪一格。填完后 start_[s] 正好是第 s 格的开头，
        // 并且每个格子里的编号从小到大排列
        for (int i = n - 1; i >= 0; --i) forEachSlot(i, [&](int s) { entries_[--start_[s]] = i; });
    }
    // #endregion

    // #region pairs
    // 每个格子内部两两测试。一对物体可能同时出现在好几个格子里，只在"参考格子"里报告它：
    // 参考格子是两个盒子交集的左上角所在的格子，两个物体一定都覆盖它，所以恰好报告一次
    long findPairs(std::vector<Pair>& out) const {
        out.clear();
        const std::vector<Aabb>& b = *boxes_;
        long tests = 0;
        for (int s = 0; s < idx_.slots(); ++s) {
            int begin = start_[s], end = start_[s + 1];
            for (int k = begin; k < end; ++k)
                for (int l = k + 1; l < end; ++l) {
                    int i = entries_[k], j = entries_[l];
                    ++tests;
                    if (overlaps(b[i], b[j]) && refSlot(b[i], b[j]) == s) out.push_back({i, j});
                }
        }
        return tests;
    }
    // #endregion

    // #region query
    // 找出与 region 重叠的所有物体，每个只回调一次（同样用参考格子去重）
    template <class F>
    void query(const Aabb& region, F&& visit) const {
        const std::vector<Aabb>& b = *boxes_;
        CellRange r = idx_.range(region);
        for (int y = r.y0; y <= r.y1; ++y)
            for (int x = r.x0; x <= r.x1; ++x) {
                int s = idx_.slot(x, y);
                for (int k = start_[s]; k < start_[s + 1]; ++k) {
                    int j = entries_[k];
                    if (!overlaps(b[j], region)) continue;
                    CellRange ref = refCell(b[j], region);
                    if (ref.x0 == x && ref.y0 == y) visit(j);
                }
            }
    }
    // #endregion

    // 供演示使用：某个存储位置里的物体编号
    int slotBegin(int s) const { return start_[s]; }
    int slotEnd(int s) const { return start_[s + 1]; }
    int entry(int k) const { return entries_[k]; }
    int entryCount() const { return static_cast<int>(entries_.size()); }

private:
    // 物体覆盖的每个存储位置调用一次 f。空间哈希里，同一个物体的两个格子可能落进同一个桶，
    // stamp_ 记录每个桶最后放进了谁，避免重复放入（那样同一对会被报告两次）
    template <class F>
    void forEachSlot(int i, F&& f) {
        CellRange r = idx_.range((*boxes_)[i]);
        for (int y = r.y0; y <= r.y1; ++y)
            for (int x = r.x0; x <= r.x1; ++x) {
                int s = idx_.slot(x, y);
                if (stamp_[s] == i) continue;
                stamp_[s] = i;
                f(s);
            }
    }
    CellRange refCell(const Aabb& a, const Aabb& b) const {
        Vec2 p{std::max(a.min.x, b.min.x), std::max(a.min.y, b.min.y)};
        return idx_.range({p, p});
    }
    int refSlot(const Aabb& a, const Aabb& b) const {
        CellRange c = refCell(a, b);
        return idx_.slot(c.x0, c.y0);
    }

    Indexer idx_;
    const std::vector<Aabb>* boxes_ = nullptr;
    std::vector<int> start_;    // slots + 1 个：第 s 格的内容是 entries_[start_[s], start_[s+1])
    std::vector<int> entries_;  // 所有格子的物体编号，按格子连续存放
    std::vector<int> stamp_;
};

using UniformGrid = CellBroadphase<GridIndexer>;
using SpatialHash = CellBroadphase<HashIndexer>;

}  // namespace ga
