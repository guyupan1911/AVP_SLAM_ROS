#ifndef FAST_CORRELATIVE_SCAN_MATCHER_H
#define FAST_CORRELATIVE_SCAN_MATCHER_H

#include "gridmap.h"
#include "correlative_scan_matcher.h"
#include <ros/console.h>
#include "io.h"
#include <image_transport/image_transport.h>
#include <unistd.h>
#include <opencv2/highgui.hpp>

namespace AVP
{
namespace mapping
{
    
class PrecomputationGrid2D 
{
 public:
  PrecomputationGrid2D(const GridMap& grid, const CellLimits& limits, int width,
                       std::vector<float>* reusable_intermediate_grid);

    // Returns a value between 0 and 255 to represent probabilities between
    // min_score and max_score.
    int GetValue(const Eigen::Array2i& xy_index) const {
        const Eigen::Array2i local_xy_index = xy_index - offset_;
        // The static_cast<unsigned> is for performance to check with 2 comparisons
        // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
        // local_xy_index.x() >= wide_limits_.num_x_cells ||
        // local_xy_index.y() >= wide_limits_.num_y_cells
        // instead of using 4 comparisons.
        if (static_cast<unsigned>(local_xy_index.x()) >=
                static_cast<unsigned>(wide_limits_.num_x_cells) ||
            static_cast<unsigned>(local_xy_index.y()) >=
                static_cast<unsigned>(wide_limits_.num_y_cells)) {
        return 0;
        }
        const int stride = wide_limits_.num_x_cells;
        return cells_[local_xy_index.x() + local_xy_index.y() * stride];
    }

    // Maps values from [0, 255] to [min_score, max_score].
    float ToScore(float value) const {
        return min_score_ + value * ((max_score_ - min_score_) / 255.f);
    }

    private:
    uint8 ComputeCellValue(float probability) const;

    // Offset of the precomputation grid in relation to the 'grid'
    // including the additional 'width' - 1 cells.
    const Eigen::Array2i offset_;

    // Size of the precomputation grid.
    const CellLimits wide_limits_;

    const float min_score_;
    const float max_score_;

    // Probabilites mapped to 0 to 255. 
    std::vector<uint8> cells_; 

    ros::NodeHandle node_handle_;
    image_transport::Publisher grid_map_image_pub_;

};

class PrecomputationGridStack2D 
{
    public:
    PrecomputationGridStack2D(const GridMap& grid);

    const PrecomputationGrid2D& Get(int index) {
        return precomputation_grids_[index];
    }

    int max_depth() const { return precomputation_grids_.size() - 1; }

    private:
    std::vector<PrecomputationGrid2D> precomputation_grids_;
};


// An implementation of "Real-Time Correlative Scan Matching" by Olson.
class FastCorrelativeScanMatcher2D 
{ 
    public:
    FastCorrelativeScanMatcher2D(const GridMap& grid);
    ~FastCorrelativeScanMatcher2D();

    FastCorrelativeScanMatcher2D(const FastCorrelativeScanMatcher2D&) = delete;
    FastCorrelativeScanMatcher2D& operator=(const FastCorrelativeScanMatcher2D&) = delete;

    // Aligns 'point_cloud' within the 'grid' given an
    // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
    // is possible, true is returned, and 'score' and 'pose_estimate' are updated
    // with the result.
    bool Match(const Eigen::Vector3d& initial_pose_estimate,
                const pcl::PointCloud<pcl::PointXYZ>& point_cloud, float min_score,
                float* score, Eigen::Vector3d* pose_estimate) const;

    // Aligns 'point_cloud' within the full 'grid', i.e., not
    // restricted to the configured search window. If a score above 'min_score'
    // (excluding equality) is possible, true is returned, and 'score' and
    // 'pose_estimate' are updated with the result.
    bool MatchFullSubmap(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, float min_score,
                        float* score, Eigen::Vector3d* pose_estimate) const;

    private:
    // The actual implementation of the scan matcher, called by Match() and
    // MatchFullSubmap() with appropriate 'initial_pose_estimate' and
    // 'search_parameters'.
    bool MatchWithSearchParameters(
        SearchParameters search_parameters,
        const Eigen::Vector3d& initial_pose_estimate,
        const pcl::PointCloud<pcl::PointXYZ>& point_cloud, float min_score, float* score,
        Eigen::Vector3d* pose_estimate) const;
    std::vector<Candidate2D> ComputeLowestResolutionCandidates(
        const std::vector<DiscreteScan2D>& discrete_scans,
        const SearchParameters& search_parameters) const;
    std::vector<Candidate2D> GenerateLowestResolutionCandidates(
        const SearchParameters& search_parameters) const;
    void ScoreCandidates(const PrecomputationGrid2D& precomputation_grid,
                        const std::vector<DiscreteScan2D>& discrete_scans,
                        const SearchParameters& search_parameters,
                        std::vector<Candidate2D>* const candidates) const;
    Candidate2D BranchAndBound(const std::vector<DiscreteScan2D>& discrete_scans,
                                const SearchParameters& search_parameters,
                                const std::vector<Candidate2D>& candidates,
                                int candidate_depth, float min_score) const;

    MapLimits limits_; // 栅格地图的参数 
    std::unique_ptr<PrecomputationGridStack2D> precomputation_grid_stack_; // 预先计算好的低分辨率地图stack
};

} // namespace mapping
} // namespace AVP


#endif