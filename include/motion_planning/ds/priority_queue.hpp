// Copyright 2025 Soheil E.nia

/**
 * TODO: Priorityqueue2 and priorityqueue's difference is just using a sharedptr vs raw pointer! unify that later. I guess I use one in some algorithm and the other in some other algs!  * 
 */
#include "motion_planning/pch.hpp"
#include "motion_planning/ds/rrtx_node.hpp"
#include "motion_planning/ds/fmt_node.hpp"
#include "motion_planning/ds/bit_node.hpp"
#include "motion_planning/ds/ifmt_node.hpp"
#include "motion_planning/ds/dstar_lite_node.hpp"
#pragma once

// struct FMTComparator {
//     bool operator()(const std::pair<double, FMTNode*>& a, 
//                    const std::pair<double, FMTNode*>& b) const {
//         return a.first < b.first;  // Only compare min_key
//     }
// };

struct FMTComparator {
    bool operator()(const std::pair<double, FMTNode*>& a, 
                   const std::pair<double, FMTNode*>& b) const {
        // Primary: Compare the priority key (Cost + Heuristic)
        if (std::abs(a.first - b.first) > 1e-9) { 
            return a.first < b.first;
        }
        // Secondary: Compare Node Index to ensure deterministic order
        // This prevents "undefined behavior" when costs are equal
        return a.second->getIndex() < b.second->getIndex();
    }
};


struct IFMTComparator {
    bool operator()(const std::pair<double, std::shared_ptr<IFMTNode>>& a, 
                   const std::pair<double, std::shared_ptr<IFMTNode>>& b) const {
        return a.first < b.first;  // Only compare min_key
    }
};

// struct FMTBITComparator {
//     bool operator()(const std::pair<double, BITNode*>& a,
//                     const std::pair<double, BITNode*>& b) const {
//         return a.first > b.first; // For min-heap
//     }
// };


// This is speicifically for std priorty queue and thats why the ">" is different --> for my custom priority queue you can see i have if(comare_()) so it means if the compare is true which means if a<b then its true which means its min heap by default1
// struct FMTBITComparator {
//     bool operator()(const std::pair<double, std::shared_ptr<BITNode>>& a,
//                     const std::pair<double, std::shared_ptr<BITNode>>& b) const {
//         return a.first > b.first; // Min-heap --> because in std::priority queue max heap is the default!
//     }
// };

// if you wanna use PriorityQueue2 class
struct FMTBITComparator {
    bool operator()(const std::pair<double, std::shared_ptr<BITNode>>& a,
                    const std::pair<double, std::shared_ptr<BITNode>>& b) const {
        return a.first < b.first; 
    }
};


struct RRTxComparator {
    bool operator()(const std::pair<double, RRTxNode*>& a, 
                   const std::pair<double, RRTxNode*>& b) const {
        if (a.first != b.first) 
            return a.first < b.first;  // Primary: min_key (LMC)
        return a.second->getCost() < b.second->getCost();  // Secondary: g_value
    }
};

struct DStarLiteComparator {
    bool operator()(const std::pair<double, DStarLiteNode*>& a, 
                   const std::pair<double, DStarLiteNode*>& b) const {
        DStarLiteNode* nA = a.second;
        DStarLiteNode* nB = b.second;

        // 1. Compare Primary Key (k1)
        // Use an epsilon for floating point comparisons
        if (std::abs(nA->k1 - nB->k1) > 1e-9) {
            return nA->k1 < nB->k1;
        }

        // 2. If k1 is equal, compare Secondary Key (k2)
        if (std::abs(nA->k2 - nB->k2) > 1e-9) {
            return nA->k2 < nB->k2;
        }

        // 3. Tie-breaker (optional, but good for stability)
        return nA->getIndex() < nB->getIndex();
    }
};


template <typename NodeType, typename Comparator>
class PriorityQueue {
private:
    std::vector<std::pair<double, NodeType*>> heap_;  // (priority, node)
    Comparator compare_; 

    void heapifyUp(size_t index) {
        while (index > 0) {
            size_t parent = (index - 1) / 2;
            if (compare_(heap_[index], heap_[parent])) {
                std::swap(heap_[index], heap_[parent]);
                heap_[index].second->heap_index_ = index;
                heap_[parent].second->heap_index_ = parent;
                index = parent;
            } else break;
        }
    }

    void heapifyDown(size_t index) {
        while (true) {
            size_t left = 2 * index + 1;
            size_t right = 2 * index + 2;
            size_t smallest = index;

            if (left < heap_.size() && compare_(heap_[left], heap_[smallest])) 
                smallest = left;
            if (right < heap_.size() && compare_(heap_[right], heap_[smallest])) 
                smallest = right;
            
            if (smallest != index) {
                std::swap(heap_[index], heap_[smallest]);
                heap_[index].second->heap_index_ = index;
                heap_[smallest].second->heap_index_ = smallest;
                index = smallest;
            } else break;
        }
    }


public:

    PriorityQueue() = default;

    void bulkAdd(const std::vector<std::pair<double, NodeType*>>& elements) {
        // Step 1: Add elements without heapifying
        for (const auto& elem : elements) {
            NodeType* node = elem.second;
            // if (node->in_queue_) continue; // Skip duplicates
            // node->in_queue_ = true;
            heap_.emplace_back(elem.first, node);
            node->heap_index_ = heap_.size() - 1;
        }

        // Step 2: Heapify the entire vector in O(k)
        for (int i = static_cast<int>(heap_.size()) / 2 - 1; i >= 0; --i) {
            heapifyDown(static_cast<size_t>(i));
        }
    }

    void clear() {
        for (auto& entry : heap_) {
            assert(entry.second != nullptr && "Null pointer in heap_ — this is a bug!");
            entry.second->in_queue_ = false;
        }
        heap_.clear();
    }

    const std::pair<double, NodeType*>& top() const {
        if (heap_.empty()) throw std::runtime_error("Queue is empty");
        return heap_[0];
    }

    void add(NodeType* node, double priority) {
        if (node->in_queue_) return;
        node->in_queue_ = true;
        heap_.emplace_back(priority, node);
        node->heap_index_ = heap_.size() - 1;
        heapifyUp(node->heap_index_);
    }


    void update(NodeType* node, double new_priority) {
        if (!node->in_queue_) return;
        size_t idx = node->heap_index_;
        double old_priority = heap_[idx].first;
        if (old_priority == new_priority) return;

        heap_[idx].first = new_priority;
        if (compare_(heap_[idx], {old_priority, node})) 
            heapifyUp(idx);
        else 
            heapifyDown(idx);
    }

    NodeType* pop() {
        if (heap_.empty()) return nullptr;
        NodeType* top_node = heap_[0].second;
        remove(top_node);
        return top_node;
    }

    void remove(NodeType* node) {
        if (!node->in_queue_) return;
        size_t idx = node->heap_index_;
        NodeType* last_node = heap_.back().second;

        // Swap with last element
        heap_[idx] = heap_.back();
        last_node->heap_index_ = idx;
        heap_.pop_back();
        node->in_queue_ = false;

        // Restore heap property
        if (idx < heap_.size()) {
            if (idx > 0 && compare_(heap_[idx], heap_[(idx - 1) / 2])) 
                heapifyUp(idx);
            else 
                heapifyDown(idx);
        }
    }
    // Get read-only access to the underlying heap
    const std::vector<std::pair<double, NodeType*>>& getHeap() const {
        return heap_;
    }

    bool empty() const { return heap_.empty(); }
};
//////////////////////////////////

// 1. Define the Key exactly as the paper does
struct DStarLiteKey {
    double k1;
    double k2;

    // Lexicographical comparison: k1 first, then k2
    bool operator<(const DStarLiteKey& other) const {
        if (std::abs(k1 - other.k1) > 1e-9) return k1 < other.k1;
        return k2 < other.k2 - 1e-9;
    }

    bool operator<=(const DStarLiteKey& other) const {
        return (*this < other) || (std::abs(k1 - other.k1) < 1e-9 && std::abs(k2 - other.k2) < 1e-9);
    }
};

// 2. A specialized Queue that uses the DStarLiteKey
class DStarLitePriorityQueue {
private:
    std::vector<std::pair<DStarLiteKey, DStarLiteNode*>> heap_;

    void heapifyUp(size_t idx) {
        while (idx > 0) {
            size_t p = (idx - 1) / 2;
            if (heap_[idx].first < heap_[p].first) {
                swap(idx, p);
                idx = p;
            } else break;
        }
    }

void swapNodes(size_t i, size_t j) {
    std::swap(heap_[i], heap_[j]);
    heap_[i].second->heap_index_ = i;
    heap_[j].second->heap_index_ = j;
}
    void heapifyDown(size_t idx) {
        while (true) {
            size_t l = 2 * idx + 1;
            size_t r = 2 * idx + 2;
            size_t smallest = idx;

            // Check left child
            if (l < heap_.size() && heap_[l].first < heap_[smallest].first) {
                smallest = l;
            }
            
            // Check right child - FIX: use heap_[smallest].first for comparison
            if (r < heap_.size() && heap_[r].first < heap_[smallest].first) {
                smallest = r;
            }

            if (smallest != idx) {
                swapNodes(idx, smallest);
                idx = smallest;
            } else {
                break;
            }
        }
    }

    void swap(size_t i, size_t j) {
        std::swap(heap_[i], heap_[j]);
        heap_[i].second->heap_index_ = i;
        heap_[j].second->heap_index_ = j;
    }

public:
    void add(DStarLiteNode* node, DStarLiteKey key) {
        if (node->in_queue_) { update(node, key); return; }
        node->in_queue_ = true;
        node->k1 = key.k1; node->k2 = key.k2; // Keep node in sync
        heap_.push_back({key, node});
        node->heap_index_ = heap_.size() - 1;
        heapifyUp(node->heap_index_);
    }


void clear() {
        for (auto& pair : heap_) {
            pair.second->in_queue_ = false;
            pair.second->heap_index_ = -1;
        }
        heap_.clear();
    }

    void update(DStarLiteNode* node, DStarLiteKey new_key) {
        if (!node->in_queue_) return;
        size_t idx = node->heap_index_;
        DStarLiteKey old_key = heap_[idx].first;
        heap_[idx].first = new_key;
        node->k1 = new_key.k1; node->k2 = new_key.k2;
        if (new_key < old_key) heapifyUp(idx);
        else heapifyDown(idx);
    }

    void remove(DStarLiteNode* node) {
        if (!node->in_queue_) return;
        size_t idx = node->heap_index_;
        swap(idx, heap_.size() - 1);
        heap_.pop_back();
        node->in_queue_ = false;
        if (idx < heap_.size()) {
            heapifyUp(idx);
            heapifyDown(idx);
        }
    }

    DStarLiteKey topKey() const {
        if (heap_.empty()) return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        return heap_[0].first;
    }

    DStarLiteNode* pop() {
        if (heap_.empty()) return nullptr;
        DStarLiteNode* node = heap_[0].second;
        remove(node);
        return node;
    }
    
    bool empty() const { return heap_.empty(); }
};


//////////////////////////////////
template <typename NodeType, typename Comparator>
class PriorityQueue2 {
private:
    std::vector<std::pair<double, std::shared_ptr<NodeType>>> heap_;  // (priority, node)
    Comparator compare_; 

    void heapifyUp(size_t index) {
        while (index > 0) {
            size_t parent = (index - 1) / 2;
            if (compare_(heap_[index], heap_[parent])) {
                std::swap(heap_[index], heap_[parent]);
                heap_[index].second->heap_index_ = index;
                heap_[parent].second->heap_index_ = parent;
                index = parent;
            } else break;
        }
    }

    void heapifyDown(size_t index) {
        while (true) {
            size_t left = 2 * index + 1;
            size_t right = 2 * index + 2;
            size_t smallest = index;

            if (left < heap_.size() && compare_(heap_[left], heap_[smallest])) 
                smallest = left;
            if (right < heap_.size() && compare_(heap_[right], heap_[smallest])) 
                smallest = right;
            
            if (smallest != index) {
                std::swap(heap_[index], heap_[smallest]);
                heap_[index].second->heap_index_ = index;
                heap_[smallest].second->heap_index_ = smallest;
                index = smallest;
            } else break;
        }
    }


public:

    PriorityQueue2() = default;

    void clear() {
        for (auto& entry : heap_) {
            entry.second->in_queue_ = false;
        }
        heap_.clear();
    }

    const std::pair<double, std::shared_ptr<NodeType>>& top() const {
        if (heap_.empty()) throw std::runtime_error("Queue is empty");
        return heap_[0];
    }

    void add(std::shared_ptr<NodeType> node, double priority) {
        if (node->in_queue_) return;
        // node->in_queue_ = true;
        heap_.emplace_back(priority, node);
        node->heap_index_ = heap_.size() - 1;
        heapifyUp(node->heap_index_);
    }


    void update(std::shared_ptr<NodeType> node, double new_priority) {
        if (!node->in_queue_) return;
        size_t idx = node->heap_index_;
        double old_priority = heap_[idx].first;
        if (old_priority == new_priority) return;

        heap_[idx].first = new_priority;
        if (compare_(heap_[idx], {old_priority, node})) 
            heapifyUp(idx);
        else 
            heapifyDown(idx);
    }

    std::shared_ptr<NodeType> pop() {
        if (heap_.empty()) return nullptr;
        std::shared_ptr<NodeType> top_node = heap_[0].second;
        remove(top_node);
        return top_node;
    }

    void remove(std::shared_ptr<NodeType> node) {
        if (!node->in_queue_) return;
        size_t idx = node->heap_index_;
        std::shared_ptr<NodeType> last_node = heap_.back().second;

        // Swap with last element
        heap_[idx] = heap_.back();
        last_node->heap_index_ = idx;
        heap_.pop_back();
        // node->in_queue_ = false;

        // Restore heap property
        if (idx < heap_.size()) {
            if (idx > 0 && compare_(heap_[idx], heap_[(idx - 1) / 2])) 
                heapifyUp(idx);
            else 
                heapifyDown(idx);
        }
    }
    // Get read-only access to the underlying heap
    const std::vector<std::pair<double, std::shared_ptr<NodeType>>>& getHeap() const {
        return heap_;
    }

    bool empty() const { return heap_.empty(); }
};