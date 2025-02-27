// Copyright 2025 SOheil E.nia

#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/state_space/state.hpp"
#include "motion_planning/state_space/statespace.hpp"
#include "motion_planning/utils/problem_definition.hpp"
#include "motion_planning/utils/planner_params.hpp"

#include "motion_planning/utils/nano_flann.hpp"
#include "motion_planning/utils/visualization.hpp"

#include "motion_planning/utils/obstacle_checker.hpp"

enum class PlannerType{
    FMTX,
    RRTX,
    RRTSTAR,
    RRT,
};


struct QueueElement {
    double min_key;  // Priority value (key)
    double g_value;  // Additional value used for comparison if needed
    int index;       // A unique index for the element

    // Comparator to define the heap order
    bool operator<(const QueueElement& other) const {
        if (min_key != other.min_key) {
            return min_key < other.min_key;  // Min heap: smaller key has higher priority
        }
        return g_value < other.g_value;  // If keys are equal, compare g_value
    }
};

class UpdatablePriorityQueue {
private:
    std::vector<QueueElement> heap;  // Min-heap (based on std::vector)
    std::vector<int> elementIndex;   // To find element index in O(1)
    void ensureCapacity(int index) {
        if (index >= elementIndex.size()) {
            elementIndex.resize(index + 1, -1);  // Resize and initialize new elements to -1
        }
    }

void heapifyUp(size_t index) {
    while (index > 0) {
        size_t parent = (index - 1) / 2;
        if (heap[index] < heap[parent]) {  // Min-heap: child must be smaller
            std::swap(heap[parent], heap[index]);
            std::swap(elementIndex[heap[parent].index], elementIndex[heap[index].index]);
            index = parent;
        } else {
            break;
        }
    }
}

void heapifyDown(size_t index) {
    size_t left, right, smallest;
    while (true) {
        left = 2 * index + 1;
        right = 2 * index + 2;
        smallest = index;

        if (left < heap.size() && heap[left] < heap[smallest]) {
            smallest = left;
        }
        if (right < heap.size() && heap[right] < heap[smallest]) {
            smallest = right;
        }
        if (smallest != index) {
            std::swap(heap[index], heap[smallest]);
            std::swap(elementIndex[heap[index].index], elementIndex[heap[smallest].index]);
            index = smallest;
        } else {
            break;
        }
    }
}

public:
    UpdatablePriorityQueue(size_t capacity) {
        heap.reserve(capacity);
        elementIndex.resize(capacity, -1);  // Initialize elementIndex with invalid indices
    }

    // Check if an element exists in the queue
    bool contains(int index) const {
        return index >= 0 && index < elementIndex.size() && elementIndex[index] != -1;
    }


    void remove(int index) {
        if (!contains(index)) {
            throw std::runtime_error("Element not found");
        }
        size_t heap_index = elementIndex[index];
        if (heap_index >= heap.size()) {
            throw std::runtime_error("Invalid heap index");
        }

        int last_index = heap.back().index;
        heap[heap_index] = heap.back();
        elementIndex[last_index] = heap_index;
        heap.pop_back();
        elementIndex[index] = -1;

        // Fix: Ensure proper heap restoration
        if (heap_index < heap.size()) {
            heapifyUp(heap_index);
            heapifyDown(heap_index);
        }
    }


    void add(const QueueElement& element) {
        ensureCapacity(element.index);  // Ensure capacity for the new element
        if (contains(element.index)) {
            throw std::runtime_error("Element already exists");
        }
        heap.push_back(element);
        elementIndex[element.index] = heap.size() - 1;
        heapifyUp(heap.size() - 1);
    }

    void update(int index, const QueueElement& new_value) {
        ensureCapacity(index);  // Ensure capacity for the updated element
        if (!contains(index)) {
            throw std::runtime_error("Element not found");
        }
        size_t heap_index = elementIndex[index];
        QueueElement old_value = heap[heap_index];
        heap[heap_index] = new_value;

        // if (new_value < old_value) {
        //     heapifyUp(heap_index);  // New value is smaller, move it up
        // } else {
        //     heapifyDown(heap_index);  // New value is larger, move it down
        // }

        heapifyUp(heap_index);
        heapifyDown(heap_index);

    }

    QueueElement top() const {
        if (heap.empty()) {
            throw std::runtime_error("Priority queue is empty");
        }
        return heap[0];
    }

    void pop() {
        if (heap.empty()) {
            throw std::runtime_error("Priority queue is empty");
        }

        // Get the index of the element being removed
        int removed_index = heap[0].index;

        // Replace the root with the last element in the heap
        heap[0] = heap.back();
        elementIndex[heap[0].index] = 0;  // Update the index of the moved element
        heap.pop_back();  // Remove the last element
        elementIndex[removed_index] = -1;  // Mark the removed element as invalid

        if (!heap.empty()) {
            heapifyDown(0);  // Restore heap property
        }
    }

    bool empty() const {
        return heap.empty();
    }

    size_t size() const {
        return heap.size();
    }
};
////////////////////////////////////////////////////////////////////////////////////
struct QueueElement2 {
    double min_key;  // Priority value (cost)
    int index;       // A unique index for the element

    // Comparator to define the heap order (min-heap)
    bool operator<(const QueueElement2& other) const {
        return min_key < other.min_key;  // Smaller cost has higher priority
    }
};

class PriorityQueue {
private:
    std::vector<QueueElement2> heap;  // Min-heap (based on std::vector)
    std::vector<int> elementIndex;   // To find element index in O(1)

    void ensureCapacity(int index) {
        if (index >= elementIndex.size()) {
            elementIndex.resize(index + 1, -1);  // Resize and initialize new elements to -1
        }
    }

    void heapifyUp(size_t index) {
        while (index > 0) {
            size_t parent = (index - 1) / 2;
            if (heap[index] < heap[parent]) {  // Min-heap: child must be smaller
                std::swap(heap[parent], heap[index]);
                std::swap(elementIndex[heap[parent].index], elementIndex[heap[index].index]);
                index = parent;
            } else {
                break;
            }
        }
    }

    void heapifyDown(size_t index) {
        size_t left, right, smallest;
        while (true) {
            left = 2 * index + 1;
            right = 2 * index + 2;
            smallest = index;

            if (left < heap.size() && heap[left] < heap[smallest]) {
                smallest = left;
            }
            if (right < heap.size() && heap[right] < heap[smallest]) {
                smallest = right;
            }
            if (smallest != index) {
                std::swap(heap[index], heap[smallest]);
                std::swap(elementIndex[heap[index].index], elementIndex[heap[smallest].index]);
                index = smallest;
            } else {
                break;
            }
        }
    }

public:
    PriorityQueue(size_t capacity) {
        heap.reserve(capacity);
        elementIndex.resize(capacity, -1);  // Initialize elementIndex with invalid indices
    }

    // Check if an element exists in the queue
    bool contains(int index) const {
        return index >= 0 && index < elementIndex.size() && elementIndex[index] != -1;
    }

    void remove(int index) {
        if (!contains(index)) {
            throw std::runtime_error("Element not found");
        }
        size_t heap_index = elementIndex[index];
        if (heap_index >= heap.size()) {
            throw std::runtime_error("Invalid heap index");
        }

        int last_index = heap.back().index;
        heap[heap_index] = heap.back();
        elementIndex[last_index] = heap_index;
        heap.pop_back();
        elementIndex[index] = -1;

        // Restore heap property
        if (heap_index < heap.size()) {
            heapifyUp(heap_index);
            heapifyDown(heap_index);
        }
    }

    void add(const QueueElement2& element) {
        ensureCapacity(element.index);  // Ensure capacity for the new element
        if (contains(element.index)) {
            throw std::runtime_error("Element already exists");
        }
        heap.push_back(element);
        elementIndex[element.index] = heap.size() - 1;
        heapifyUp(heap.size() - 1);
    }

    void update(int index, double new_cost) {
        ensureCapacity(index);  // Ensure capacity for the updated element
        if (!contains(index)) {
            throw std::runtime_error("Element not found");
        }
        size_t heap_index = elementIndex[index];
        double old_cost = heap[heap_index].min_key;
        heap[heap_index].min_key = new_cost;

        // Restore heap property
        if (new_cost < old_cost) {
            heapifyUp(heap_index);  // New cost is smaller, move it up
        } else {
            heapifyDown(heap_index);  // New cost is larger, move it down
        }
    }

    QueueElement2 top() const {
        if (heap.empty()) {
            throw std::runtime_error("Priority queue is empty");
        }
        return heap[0];
    }

    void pop() {
        if (heap.empty()) {
            throw std::runtime_error("Priority queue is empty");
        }

        // Get the index of the element being removed
        int removed_index = heap[0].index;

        // Replace the root with the last element in the heap
        heap[0] = heap.back();
        elementIndex[heap[0].index] = 0;  // Update the index of the moved element
        heap.pop_back();  // Remove the last element
        elementIndex[removed_index] = -1;  // Mark the removed element as invalid

        if (!heap.empty()) {
            heapifyDown(0);  // Restore heap property
        }
    }

    bool empty() const {
        return heap.empty();
    }

    size_t size() const {
        return heap.size();
    }
};





class Planner {
 public:
    Planner() = default;
    virtual ~Planner() = default;

    virtual void setStart(const Eigen::VectorXd& start) = 0;
    virtual void setGoal(const Eigen::VectorXd& goal) = 0;
    
    virtual void setup(const PlannerParams& params , std::shared_ptr<Visualization> visualization) = 0;
    virtual void plan() = 0;
    virtual std::vector<int> getPathIndex() const = 0;

 protected:


};


