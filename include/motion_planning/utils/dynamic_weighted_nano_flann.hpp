// Copyright 2025 Soheil E.nia
#pragma once

#include "motion_planning/utils/kd_tree.hpp"
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

/**
 * Adaptor to make Eigen::MatrixXd compatible with Nanoflann Dynamic Adaptor.
 */
struct EigenMatrixAdaptor {
    const Eigen::MatrixXd& data; // Reference to the data matrix

    EigenMatrixAdaptor(const Eigen::MatrixXd& mat) : data(mat) {}

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return data.rows(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return data(idx, dim);
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

/**
 * DynamicWeightedNanoFlann
 * 
 * Inherits directly from KDTree. Uses nanoflann's KDTreeSingleIndexDynamicAdaptor
 * for O(log N) insertions. Handles weighted distances and wrapping dimensions.
 */
class DynamicWeightedNanoFlann : public KDTree {
public:
    DynamicWeightedNanoFlann(int dimension, 
                             const Eigen::VectorXd& weights,
                             const std::vector<int>& wrap_dims = {},
                             const std::vector<double>& wrap_periods = {});
    
    ~DynamicWeightedNanoFlann() override = default;

    // --- KDTree Interface Implementation ---
    
    void addPoint(const Eigen::VectorXd& stateValue) override;
    void addPoints(const std::vector<Eigen::VectorXd>& statesValues) override;
    void addPoints(const Eigen::MatrixXd& states) override;
    
    // For dynamic tree, this is a no-op (tree is built incrementally)
    void buildTree() override; 
    
    std::vector<size_t> knnSearch(const Eigen::VectorXd& query, int k) const override;
    std::vector<size_t> radiusSearch(const Eigen::VectorXd& query, double radius) const override;
    
    // Dual search not implemented for this dynamic version to keep it simple, 
    // or you can implement it similar to radiusSearch.
    std::pair<std::vector<size_t>, std::vector<size_t>> radiusSearchDual(
        const Eigen::VectorXd& query, double radius1, double radius2) const override;

    void clear() override;
    bool removePoint(const Eigen::VectorXd& query) override;
    bool removeByIndex(size_t index) override;
    void printData() const override;
    size_t size() const override;
    int getDimension() override { return dimension_; }

    // Validation placeholders
    bool validateAgainstSamples(const std::vector<std::shared_ptr<IFMTNode>>& samples) const override { return false; }
    bool validateAgainstSamples(const std::vector<std::shared_ptr<BITNode>>& samples) const override { return false; }

private:
    int dimension_;
    Eigen::VectorXd weights_;
    
    // We store SCALED data in the matrix because the adaptor sees this matrix directly.
    // This makes the tree search efficient (no scaling needed during search).
    Eigen::MatrixXd scaled_data_; 
    
    // We keep unscaled data only if we need to return original values, 
    // but for KDTree interface we mostly return indices. 
    // However, for wrapping logic, we need the original values to compute true distances.
    Eigen::MatrixXd unscaled_data_; 

    std::vector<int> wrap_dims_;
    std::vector<double> wrap_periods_;

    // Nanoflann Dynamic Adaptor Type
    using DynamicKDTreeType = nanoflann::KDTreeSingleIndexDynamicAdaptor<
        nanoflann::L2_Simple_Adaptor<double, EigenMatrixAdaptor>,
        EigenMatrixAdaptor
    >;

    std::unique_ptr<EigenMatrixAdaptor> adaptor_;
    std::unique_ptr<DynamicKDTreeType> dynamicTree_;

    // Helper to get unscaled point
    Eigen::VectorXd getUnscaledPoint(size_t index) const;
};