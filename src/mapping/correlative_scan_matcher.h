#ifndef CORRELATIVE_SCAN_MATCHER_H
#define CORRELATIVE_SCAN_MATCHER_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "gridmap.h"
#include "transform.h"

namespace AVP
{
namespace mapping
{
    
typedef std::vector<Eigen::Array2i> DiscreteScan2D; // 保存一个点云中所有的点对应的栅格位置索引

// Describes the search space.
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(double linear_search_window, double angular_search_window,
                   const pcl::PointCloud<pcl::PointXYZ>& point_cloud, double resolution);

  // For testing.
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // Tightens the search window as much as possible.
  void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                   const CellLimits& cell_limits);

  int num_angular_perturbations;            // 个数
  double angular_perturbation_step_size;    // 角度分辨率
  double resolution;
  int num_scans;                            // 旋转后的点云集合的个数
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

// Generates a collection of rotated scans.
std::vector<pcl::PointCloud<pcl::PointXYZ>> GenerateRotatedScans(
    const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<pcl::PointCloud<pcl::PointXYZ>>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
struct Candidate2D {
  Candidate2D(const int init_scan_index, const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0; 

  // Linear offset from the initial pose. 以栅格为单位
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate2D relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};

} // namespace mapping
} // namespace AVP


#endif