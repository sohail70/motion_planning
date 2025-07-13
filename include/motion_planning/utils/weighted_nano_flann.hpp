#pragma once

#include "motion_planning/utils/nano_flann.hpp"
#include "motion_planning/utils/kd_tree.hpp"
#include <vector>
#include <map>

/**
 * @brief Helper function to normalize an angle to the [-PI, PI] range.
 * This is useful for distance calculations involving orientation.
 */
inline double normalizeAngle(double angle) {
    angle = fmod(angle, 2.0 * M_PI);
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}


/**
 * @class WeightedNanoFlann
 * @brief A KD-tree that correctly handles weighted distances and wrapping (toroidal) dimensions.
 *
 * This class implements the "ghost point" search strategy. For dimensions that wrap
 * (like orientation), it performs multiple searches: one for the original query and
 * others for "ghost" queries shifted by the wrap-around period. It stores both the
 * original and a scaled version of the data to perform these operations efficiently.
 */
class WeightedNanoFlann : public KDTree {
public:
    /**
     * @brief Construct a new WeightedNanoFlann object.
     * @param dimension The dimensionality of the state space.
     * @param weights A vector of weights, one for each dimension.
     * @param wrap_dims A vector of indices for dimensions that wrap (e.g., {2} for theta).
     * @param wrap_periods A vector of periods for each wrapped dim (e.g., {2*M_PI}).
     */
    WeightedNanoFlann(int dimension, const Eigen::VectorXd& weights,
                      const std::vector<int>& wrap_dims = {}, 
                      const std::vector<double>& wrap_periods = {});

    ~WeightedNanoFlann() override = default;

    // --- Override all virtual methods from the KDTree interface ---

    void addPoint(const Eigen::VectorXd& stateValue) override;
    void addPoints(const std::vector<Eigen::VectorXd>& statesValues) override;
    void addPoints(const Eigen::MatrixXd& states) override;

    void buildTree() override;

    std::vector<size_t> knnSearch(const Eigen::VectorXd& query, int k) const override;
    std::vector<size_t> radiusSearch(const Eigen::VectorXd& query, double radius) const override;
    std::pair<std::vector<size_t>, std::vector<size_t>> radiusSearchDual(
        const Eigen::VectorXd& query, double radius1, double radius2) const override;
    
    bool removePoint(const Eigen::VectorXd& query) override;

    void printData() const override;
    void clear() override;
    bool removeByIndex(size_t index) override;
    size_t size() const override;

    // Validation is complex and marked as not fully supported
    bool validateAgainstSamples(const std::vector<std::shared_ptr<IFMTNode>>& samples) const override;
    bool validateAgainstSamples(const std::vector<std::shared_ptr<BITNode>>& samples) const override;
    int getDimension() override {return dimension_;}

private:
    int dimension_;
    Eigen::VectorXd weights_;
    
    // --- Data Storage ---
    Eigen::MatrixXd unscaled_data_; // Stores the ORIGINAL, unscaled points
    NanoFlann rawTree_;             // Operates on a SCALED copy of the data

    // --- Members for handling wrapping dimensions ---
    std::vector<int> wrap_dims_;
    std::vector<double> wrap_periods_;

    // Private helper to get the original, unscaled data point by its index
    Eigen::VectorXd getUnscaledPoint(size_t index) const;
};
