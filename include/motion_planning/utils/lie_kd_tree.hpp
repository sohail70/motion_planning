// Copyright 2025 Soheil E.nia

#pragma once

#include "motion_planning/state_space/dubins_time_statespace.hpp"
#include "motion_planning/utils/kd_tree.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <utility> // For std::pair

// Forward declaration of the internal Node structure
struct LieNode;

/**
 * LieSplittingKDTree
 * A k-d tree tailored for nonholonomic systems using Lie splitting.
 */
class LieSplittingKDTree: public KDTree {
public:
    /**
     * Constructs the Lie-splitting k-d tree.
     * dimension The state space dimension (e.g., 4 for DubinsTime).
     */
    explicit LieSplittingKDTree(int dimension, std::shared_ptr<StateSpace> space);

    /**
     * Destructor to clean up the tree nodes.
     */
    ~LieSplittingKDTree();

    /**
     * Adds a single point to the tree incrementally.
     * stateValue The state vector to add.
     */
    void addPoint(const Eigen::VectorXd& stateValue) override;

    /**
     * Finds the k nearest neighbors to a query state.
     * return A vector of indices corresponding to the nearest neighbors.
     */
    std::vector<size_t> knnSearch(const Eigen::VectorXd& query, int k) const override;

    /**
     * Finds all neighbors within a given geometric radius of a query state. 
     * A vector of indices of points found within the radius.
     */
    std::vector<size_t> radiusSearch(const Eigen::VectorXd& query, double radius) const override;

    /**
     * Finds neighbors within two different radii simultaneously.
     * return A pair of vectors: the first contains indices within radius1, the second within radius2.
     */
    std::pair<std::vector<size_t>, std::vector<size_t>> radiusSearchDual(const Eigen::VectorXd& query, double radius1, double radius2) const override;



    void clear() override;

    size_t size() const override;

    // USELESS FOR NOW
    void addPoints(const std::vector<Eigen::VectorXd>& statesValues) override;
    void addPoints(const Eigen::MatrixXd& states) override;
    void buildTree() override;
    void printData() const override;
    bool removePoint(const Eigen::VectorXd& query) override;
    bool removeByIndex(size_t index) override;
    bool validateAgainstSamples(const std::vector<std::shared_ptr<IFMTNode>>& samples) const override;
    bool validateAgainstSamples(const std::vector<std::shared_ptr<BITNode>>& samples) const override;
    int getDimension() override {return dimension_;}

private:
    LieNode* root_ = nullptr;
    int dimension_;
    std::vector<Eigen::VectorXd> points_;
    std::shared_ptr<StateSpace> space_;
    const std::vector<double> weights_ = {1.0, 2.0, 1.0, 1.0};
    const std::vector<double> C_constants_ = {1.0, 0.5, 1.0, 1.0};
    const double W_total_ = 5.0;

    // Recursive helper functions
    LieNode* insert_recursive(LieNode* current, const Eigen::VectorXd& point, int depth);
    void knn_search_recursive(LieNode* current, const Eigen::VectorXd& query, int k,
                              std::priority_queue<std::pair<double, size_t>>& pq) const;
    void radius_search_recursive(LieNode* current, const Eigen::VectorXd& query, double radius, std::vector<size_t>& results) const;
    void radius_search_dual_recursive(LieNode* current, const Eigen::VectorXd& query, double r1, double r2, std::vector<size_t>& results1, std::vector<size_t>& results2) const;


    void getPrivilegedAxes(const Eigen::VectorXd& state, std::vector<Eigen::VectorXd>& axes) const;
    Eigen::VectorXd getSplitNormal(int depth, const Eigen::VectorXd& point) const;
    bool ballHyperplaneIntersection(const Eigen::VectorXd& query, double radius,
                                    const Eigen::VectorXd& plane_point, const Eigen::VectorXd& plane_normal) const;

    void delete_recursive(LieNode* node);
};
