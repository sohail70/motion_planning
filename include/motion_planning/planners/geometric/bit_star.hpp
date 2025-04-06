#pragma once
#include "motion_planning/planners/planner.hpp"
#include "motion_planning/ds/fmt_node.hpp"
#include "motion_planning/utils/nano_flann.hpp"
#include <queue>

class BITStar : public Planner {
public:
    struct BITNode;
    
    struct QueueComparator {
        bool operator()(const std::pair<double, BITNode*>& a, 
                       const std::pair<double, BITNode*>& b) const {
            return a.first > b.first;
        }
    };

    // Modify queues to use order counters
    struct EdgeComparator {
        bool operator()(const std::tuple<double, int, double, BITNode*, BITNode*>& a,
                    const std::tuple<double, int, double, BITNode*, BITNode*>& b) const {
            return std::get<0>(a) > std::get<0>(b); // First compare cost
        }
    };


    BITStar(std::unique_ptr<StateSpace> statespace,
           std::shared_ptr<ProblemDefinition> problem_def,
           std::shared_ptr<ObstacleChecker> obs_checker);
           
    void setStart(const Eigen::VectorXd& start) override;
    void setGoal(const Eigen::VectorXd& goal) override;
    void setup(const Params& params, std::shared_ptr<Visualization> visualization) override;
    void plan() override;
    void visualizeTree();

    struct BITNode : public FMTNode {
        double h_hat;    // Heuristic estimate to goal
        double g;       // Cost-to-come from start
        bool expanded;
        bool in_unconnected;
        
        BITNode(std::unique_ptr<State> state, int index)
            : FMTNode(std::move(state), index), h_hat(INFINITY), g(INFINITY),
              expanded(false), in_unconnected(true) {}
    };


private:
    // Queues
    std::priority_queue<std::pair<double, BITNode*>, 
                       std::vector<std::pair<double, BITNode*>>,
                       QueueComparator> vertex_queue_;
    
    // In BITStar class
    int qe_order_ = 0;
    int qv_order_ = 0;


    std::priority_queue<std::tuple<double, int, double, BITNode*, BITNode*>,
                    std::vector<std::tuple<double, int, double, BITNode*, BITNode*>>,
                    EdgeComparator> edge_queue_;



    std::vector<std::unique_ptr<BITNode>> nodes_;
    std::vector<BITNode*> unconnected_;
    std::vector<BITNode*> solution_nodes_;

    std::vector<BITNode*> x_reuse_; // Add to class

    
    BITNode* start_node_;
    BITNode* goal_node_;
    double ci_;         // Current best solution cost
    double radius_;     // Connection radius
    int batch_size_;
    int samples_added_;
    
    std::shared_ptr<NanoFlann> kdtree_;
    std::unique_ptr<StateSpace> statespace_;
    std::shared_ptr<ProblemDefinition> problem_;
    std::shared_ptr<Visualization> visualization_;
    std::shared_ptr<ObstacleChecker> obs_checker_;

    void initialize();
    void processBatch();
    void prune();
    void expandVertex(BITNode* vertex);
    void addEdge(BITNode* parent, BITNode* child, double cost);
    void updateSolution();
    void sampleBatch(bool initial_batch = false);
    double heuristic(const Eigen::VectorXd& state) const;
    std::vector<BITNode*> findNear(BITNode* node, const std::vector<BITNode*>& nodes);

};