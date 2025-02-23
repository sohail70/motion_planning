// Modified RRTX header (rrtx.hpp)
#pragma once

#include "motion_planning/pch.hpp"
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/tree_node.hpp"
#include "motion_planning/utils/visualization.hpp"



// struct NeighborInfo {
//     int index;
//     double distance;
// };

struct QueueElement {
    double min_key;  // Priority value (key)
    double g_value;  // Additional value used for comparison if needed
    int index;       // A unique index for the element

    // Comparator to define the heap order
    bool operator<(const QueueElement& other) const {
        if (min_key != other.min_key) {
            return min_key > other.min_key;  // Min heap: smaller key is higher priority
        }
        return g_value > other.g_value;  // If keys are equal, compare g_value
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
            if (heap[parent] < heap[index]) {
                std::swap(heap[parent], heap[index]);
                std::swap(elementIndex[heap[parent].index], elementIndex[heap[index].index]);
                index = parent;
            } else {
                break;
            }
        }
    }

    void heapifyDown(size_t index) {
        size_t left, right, largest;
        while (true) {
            left = 2 * index + 1;
            right = 2 * index + 2;
            largest = index;

            if (left < heap.size() && heap[largest] < heap[left]) {
                largest = left;
            }
            if (right < heap.size() && heap[largest] < heap[right]) {
                largest = right;
            }
            if (largest != index) {
                std::swap(heap[index], heap[largest]);
                std::swap(elementIndex[heap[index].index], elementIndex[heap[largest].index]);
                index = largest;
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
        // std::cout << "Removing index: " << index << std::endl;
        if (!contains(index)) {
            throw std::runtime_error("Element not found");
        }
        size_t heap_index = elementIndex[index];
        // std::cout << "Heap index: " << heap_index << std::endl;
        if (heap_index >= heap.size()) {
            throw std::runtime_error("Invalid heap index");
        }

        int last_index = heap.back().index;
        // std::cout << "Last index: " << last_index << std::endl;

        heap[heap_index] = heap.back();
        elementIndex[last_index] = heap_index;
        heap.pop_back();
        elementIndex[index] = -1;

        // std::cout << "Heap size after removal: " << heap.size() << std::endl;

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

        if (new_value < old_value) {
            heapifyUp(heap_index);  // New value is smaller, move it up
        } else {
            heapifyDown(heap_index);  // New value is larger, move it down
        }
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

    // RRTX-specific interface
    void updateRobotPosition(const Eigen::VectorXd& new_position);
    void updateObstacleSamples(const std::vector<Eigen::Vector2d>& obstacles);

    void visualizeTree();


 private:
    // Core data structures
    std::vector<std::shared_ptr<TreeNode>> tree_;
    std::shared_ptr<KDTree> kdtree_;
    std::shared_ptr<Visualization> visualization_;
    std::shared_ptr<ObstacleChecker> obs_checker_;
    std::unique_ptr<StateSpace> statespace_;
    std::unique_ptr<ProblemDefinition> problem_;
    std::unordered_set<int> v_indices_;

    std::unordered_set<int> samples_in_obstacles_; // Current samples in obstacles

    std::unordered_map<int, QueueElement> handle_map_; 

    // Algorithm state
    int vbot_index_;          // Robot's current node
    int vgoal_index_;         // Goal node index
    double neighborhood_radius_;
    double epsilon_ = 1e-6;   // Consistency threshold

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


    // struct QueueComparator {
    //     const RRTX& rrtx;  // Reference to the RRTX instance

    //     // Constructor to initialize the reference
    //     explicit QueueComparator(const RRTX& rrtx_instance) : rrtx(rrtx_instance) {}

    //     bool operator()(const std::pair<double, int>& a, const std::pair<double, int>& b) const {
    //         // Compare min(g(v), lmc(v)) first
    //         if (a.first != b.first) {
    //             return a.first > b.first;  // Smaller min(g(v), lmc(v)) has higher priority
    //         }
    //         // If min(g(v), lmc(v)) is equal, compare g(v)
    //         return rrtx.tree_[a.second]->getCost() > rrtx.tree_[b.second]->getCost();
    //     }
    // };
    // // Priority queue for inconsistency propagation
    // using QueueElement = std::pair<double, int>;  // (min(g(v), lmc(v)), node index)
    // std::priority_queue<QueueElement, std::vector<QueueElement>, QueueComparator> inconsistency_queue_;

    UpdatablePriorityQueue inconsistency_queue_;







    // Neighbor management
    std::unordered_map<int, std::unordered_set<int>> N0_in_;   // Original incoming neighbors
    std::unordered_map<int, std::unordered_set<int>> N0_out_;  // Original outgoing neighbors
    std::unordered_map<int, std::unordered_set<int>> Nr_in_;   // Running incoming neighbors
    std::unordered_map<int, std::unordered_set<int>> Nr_out_;  // Running outgoing neighbors

    // Distance dictionary
    std::unordered_map<int, std::unordered_map<int, double>> distance_;  // Key1: node index, Key2: neighbor index, Value: distance

    std::unordered_set<int> Vc_T_;




    // Helper functions
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
