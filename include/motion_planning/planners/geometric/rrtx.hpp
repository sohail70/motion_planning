// Copyright 2025 Soheil E.nia
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/tree_node.hpp"
#include "motion_planning/utils/visualization.hpp"


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


class RRTX : public Planner {
 public:
    RRTX(std::unique_ptr<StateSpace> statespace, 
        std::unique_ptr<ProblemDefinition> problem_def,
        std::shared_ptr<ObstacleChecker> obs_checker);
    
    void setup(const PlannerParams& params, std::shared_ptr<Visualization> visualization) override;
    void plan() override;
    std::vector<int> getPathIndex() const override;
    void setStart(const Eigen::VectorXd& start) override;
    void setGoal(const Eigen::VectorXd& goal) override;

    void updateRobotPosition(const Eigen::VectorXd& new_position);
    void updateObstacleSamples(const std::vector<Eigen::Vector2d>& obstacles);

    void visualizeTree();
    void visualizePath(std::vector<int> path_indices);
    void setRobotIndex(const Eigen::VectorXd& robot_position);

 private:
    std::vector<std::shared_ptr<TreeNode>> tree_;
    std::shared_ptr<KDTree> kdtree_;
    std::shared_ptr<Visualization> visualization_;
    std::shared_ptr<ObstacleChecker> obs_checker_;
    std::unique_ptr<StateSpace> statespace_;
    std::unique_ptr<ProblemDefinition> problem_;
    std::unordered_set<int> v_indices_;

    std::unordered_set<int> samples_in_obstacles_; 

    std::unordered_map<int, QueueElement> handle_map_; 

    int vbot_index_;
    int vgoal_index_;
    double neighborhood_radius_;
    double epsilon_ = 1e-6; 

    int num_of_samples_;
    double lower_bound_;
    double upper_bound_;
    int root_state_index_;
    int robot_state_index_;
    bool use_kdtree;
    int dimension_;
    double gamma_;
    size_t sample_counter;
    bool cap_samples_ = true;
    double delta = 10.0; // Step size limit


    UpdatablePriorityQueue inconsistency_queue_;







    // Neighbor management
    std::unordered_map<int, std::unordered_set<int>> N0_in_;   // Original incoming neighbors
    std::unordered_map<int, std::unordered_set<int>> N0_out_;  // Original outgoing neighbors
    std::unordered_map<int, std::unordered_set<int>> Nr_in_;   // Running incoming neighbors
    std::unordered_map<int, std::unordered_set<int>> Nr_out_;  // Running outgoing neighbors

    // Distance dictionary
    std::unordered_map<int, std::unordered_map<int, double>> distance_;  // Key1: node index, Key2: neighbor index, Value: distance

    std::unordered_set<int> Vc_T_;




    // RRTX functions
    double shrinkingBallRadius() const;
    void extend(Eigen::VectorXd v);
    void rewireNeighbors(int v_index);
    void reduceInconsistency();
    void findParent(Eigen::VectorXd v, const std::vector<size_t>& candidates);
    void cullNeighbors(int v_index);
    void propagateDescendants();
    void verifyQueue(int v_index);
    void updateLMC(int v_index);
    void handleObstacleChanges();
    void makeParentOf(int child_index, int parent_index);
    void addNewObstacle(const std::vector<int>& added_samples);
    void removeObstacle(const std::vector<int>& removed_samples);
    void verifyOrphan(int v_index);


    std::unordered_set<int> findSamplesNearObstacles(
        const std::vector<Eigen::Vector2d>& obstacles, 
        double obstacle_radius
    );
    // Obstacle management
    std::unordered_set<int> obstacle_samples_;
    std::vector<Eigen::Vector2d> current_obstacles_;
};
