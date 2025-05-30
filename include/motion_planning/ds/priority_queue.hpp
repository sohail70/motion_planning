#include "motion_planning/pch.hpp"
#include "motion_planning/ds/rrtx_node.hpp"
#include "motion_planning/ds/fmt_node.hpp"
#include "motion_planning/ds/bit_node.hpp"
#include "motion_planning/ds/ifmt_node.hpp"
#pragma once

struct FMTComparator {
    bool operator()(const std::pair<double, FMTNode*>& a, 
                   const std::pair<double, FMTNode*>& b) const {
        return a.first < b.first;  // Only compare min_key
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