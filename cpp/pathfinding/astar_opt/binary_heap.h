#pragma once

#include <utility>
#include <vector>

namespace ga {

// #region indexed-heap
// 带索引的二叉小顶堆。元素是 0..capacity-1 的整数 id（这里就是格子下标），
// 每个 id 在堆里至多出现一次。额外记录每个 id 在数组中的位置 pos_，于是可以就地"降低优先级"，
// 不再需要往堆里塞重复条目、出堆时再跳过。
// 上浮 / 下沉时用"挪空位"代替反复交换：先把节点拿出来，其他节点依次移动，最后放回一次。
//
// 数组布局：下标 i 的父节点是 (i-1)/2，子节点是 2i+1 和 2i+2。
template <class Key>
class IndexedMinHeap {
public:
    void reset(int capacity) {
        heap_.clear();
        pos_.assign(capacity, -1);
    }
    // 清空堆，只重置仍在堆里的元素：O(堆大小)，而不是 O(capacity)
    void clear() {
        for (const Node& n : heap_) pos_[n.id] = -1;
        heap_.clear();
    }

    bool empty() const { return heap_.empty(); }
    int size() const { return static_cast<int>(heap_.size()); }
    bool contains(int id) const { return pos_[id] >= 0; }
    const Key& key(int id) const { return heap_[pos_[id]].key; }
    const Key& topKey() const { return heap_[0].key; }
    int idAt(int i) const { return heap_[i].id; }

    void push(int id, const Key& key) {
        heap_.push_back({key, id});
        siftUp(size() - 1);
    }
    // 新的 key 必须不大于原来的 key
    void decreaseKey(int id, const Key& key) {
        int i = pos_[id];
        heap_[i].key = key;
        siftUp(i);
    }
    // 取出 key 最小的 id
    int pop() {
        int top = heap_[0].id;
        pos_[top] = -1;
        Node last = heap_.back();
        heap_.pop_back();
        if (!heap_.empty()) siftDown(0, last);
        return top;
    }

private:
    struct Node {
        Key key;  // key 和 id 放在一起，比较时不必再查一次表
        int id;
    };

    // 上浮：把 i 处的节点和父节点比较，比父节点小就让父节点下移一层
    void siftUp(int i) {
        Node node = heap_[i];
        while (i > 0) {
            int parent = (i - 1) / 2;
            if (!(node.key < heap_[parent].key)) break;
            place(i, heap_[parent]);
            i = parent;
        }
        place(i, node);
    }
    // 下沉：从 i 开始为 node 找位置，较小的子节点逐层上移
    void siftDown(int i, Node node) {
        const int n = size();
        for (;;) {
            int child = 2 * i + 1;
            if (child >= n) break;
            if (child + 1 < n && heap_[child + 1].key < heap_[child].key) ++child;
            if (!(heap_[child].key < node.key)) break;
            place(i, heap_[child]);
            i = child;
        }
        place(i, node);
    }
    void place(int i, const Node& node) {
        heap_[i] = node;
        pos_[node.id] = i;
    }

    std::vector<Node> heap_;  // 堆数组
    std::vector<int> pos_;    // pos_[id] = id 在 heap_ 中的下标，-1 表示不在堆里
};
// #endregion

}  // namespace ga
