#include "fast_correlative_scan_matcher.h"

namespace AVP
{  
namespace mapping
{
namespace {

/************** SlidingWindowMaximum **************/

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1). // 摊派
// 滑动窗口算法
class SlidingWindowMaximum 
{
    public:
    // 添加值, 会将小于填入值的其他值删掉, 再将这个值放到最后
    void AddValue(const float value) {
        while (!non_ascending_maxima_.empty() &&
            value > non_ascending_maxima_.back()) {
        non_ascending_maxima_.pop_back();
        }
        non_ascending_maxima_.push_back(value);
    }

    // 删除值, 如果第一个值等于要删除的这个值, 则将这个值删掉
    void RemoveValue(const float value) {
        // DCHECK for performance, since this is done for every value in the
        // precomputation grid.
        if (value == non_ascending_maxima_.front()) {
        non_ascending_maxima_.pop_front();
        }
    }

    // 获取最大值, 因为是按照顺序存储的, 第一个值是最大的
    float GetMaximum() const {
        // DCHECK for performance, since this is done for every value in the
        // precomputation grid.
        return non_ascending_maxima_.front();
    }

    void CheckIsEmpty() const { }

    private:
    // Maximum of the current sliding window at the front. Then the maximum of the
    // remaining window that came after this values first occurrence, and so on.
    std::deque<float> non_ascending_maxima_; // 下降的最大值，第一个总是最大的
};

}  // namespace

PrecomputationGrid2D::PrecomputationGrid2D(
    const GridMap& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1), 
      wide_limits_(limits.num_x_cells + width - 1, 
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.GetMaxCorrespondenceCost()), 
      max_score_(1.f - grid.GetMinCorrespondenceCost()), 
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) 
{ 

    image_transport::ImageTransport it(node_handle_);
    grid_map_image_pub_ = it.advertise("submap_grid_map_image_different_resolution",1);

    const int stride = wide_limits_.num_x_cells; 
    // First we compute the maximum probability for each (x0, y) achieved in the
    // span defined by x0 <= x < x0 + width.
    std::vector<float>& intermediate = *reusable_intermediate_grid;
    intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
    
    for (int y = 0; y != limits.num_y_cells; ++y) 
    { 
        SlidingWindowMaximum current_values;
        current_values.AddValue(
            1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y)))); 

        for (int x = -width + 1; x != 0; ++x) 
        { 
            intermediate[x + width - 1 + y * stride] = current_values.GetMaximum(); 
            if (x + width < limits.num_x_cells) 
            {
                current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost( 
                                                Eigen::Array2i(x + width, y))));
            }
        }

        for (int x = 0; x < limits.num_x_cells - width; ++x) 
        {
            intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
            current_values.RemoveValue(
                1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
            current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                                Eigen::Array2i(x + width, y))));
        }

        for (int x = std::max(limits.num_x_cells - width, 0);
            x != limits.num_x_cells; ++x) 
        {
            intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
            current_values.RemoveValue(
                1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
        }
        current_values.CheckIsEmpty();
    }

    // For each (x, y), we compute the maximum probability in the width x width
    // region starting at each (x, y) and precompute the resulting bound on the
    // score.
    for (int x = 0; x != wide_limits_.num_x_cells; ++x) 
    {
        SlidingWindowMaximum current_values;

        current_values.AddValue(intermediate[x]);
        for (int y = -width + 1; y != 0; ++y) 
        {
            cells_[x + (y + width - 1) * stride] =
                ComputeCellValue(current_values.GetMaximum());
            if (y + width < limits.num_y_cells) 
            {
                current_values.AddValue(intermediate[x + (y + width) * stride]);
            }
        }
        for (int y = 0; y < limits.num_y_cells - width; ++y) 
        {
            cells_[x + (y + width - 1) * stride] =
                ComputeCellValue(current_values.GetMaximum());
            current_values.RemoveValue(intermediate[x + y * stride]);
            current_values.AddValue(intermediate[x + (y + width) * stride]);
        }
        for (int y = std::max(limits.num_y_cells - width, 0);
            y != limits.num_y_cells; ++y) 
        {
            cells_[x + (y + width - 1) * stride] =
                ComputeCellValue(current_values.GetMaximum());
            current_values.RemoveValue(intermediate[x + y * stride]);
        }
        current_values.CheckIsEmpty();
    }

    // if(1)
    // {
    //     static int index = 0;
    //     int width = wide_limits_.num_x_cells;
    //     int height = wide_limits_.num_y_cells;
    //     cv::Mat grid_map_image = cv::Mat::zeros(width, height, CV_8UC1);

    //     for (size_t i = 0; i <  width; i++)
    //     {
    //         for (size_t j = 0; j < height; j++)
    //         {
    //             grid_map_image.at<unsigned char>(j,i) = cells_[j*width+i];
    //             if (cells_[j*width+i]!=0)
    //             {
    //                 ROS_INFO("value: %d", cells_[j*width+i]);
    //             }
    //         }
    //     }
        
    //     std::string path_to_image = "/home/catkin_ws/src/avp/data/gridImage" + std::to_string(index) + ".png";
    //     cv::imwrite(path_to_image, grid_map_image);
    //     index++;
    // }
}

uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const 
{
    const int cell_value = common::RoundToInt(
        (probability - min_score_) * (255.f / (max_score_ - min_score_)));
    return cell_value;
}

PrecomputationGridStack2D::PrecomputationGridStack2D(const GridMap& grid) 
{
  const int max_width = 1 << (7 - 1);
  precomputation_grids_.reserve(7);
  
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits(); 

  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);

  for (int i = 0; i != 7; ++i) 
  {
    const int width = 1 << i; 
    precomputation_grids_.emplace_back(grid, limits, width, &reusable_intermediate_grid); 
  }
}

FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(const GridMap& grid)
    :limits_(grid.limits()),
    precomputation_grid_stack_(std::unique_ptr<PrecomputationGridStack2D>(new PrecomputationGridStack2D(grid))){}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

bool FastCorrelativeScanMatcher2D::Match(
    const Eigen::Vector3d& initial_pose_estimate,
    const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const float min_score, float* score,
    Eigen::Vector3d* pose_estimate) const 
{
  const SearchParameters search_parameters(7,
                                           M_PI/6,
                                           point_cloud, limits_.resolution());

  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const Eigen::Vector3d& initial_pose_estimate,
    const pcl::PointCloud<pcl::PointXYZ>& point_cloud, float min_score, float* score,
    Eigen::Vector3d* pose_estimate) const 
{
    // 将tracking frame下的点云用初始的node位姿变换到世界坐标系中
    const pcl::PointCloud<pcl::PointXYZ> rotated_point_cloud = 
        TransformPointCloudAndReturn(point_cloud, initial_pose_estimate);

    const std::vector<pcl::PointCloud<pcl::PointXYZ>> rotated_scans = // 按照不同角度旋转后的点云
        GenerateRotatedScans(rotated_point_cloud, search_parameters);

    ROS_INFO("rotated_scans size: %d", rotated_scans.size());

    const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans( // 计算旋转后的点云，经初始位姿变换后在栅格地图中的索引
        limits_, rotated_scans,
        Eigen::Translation2f(initial_pose_estimate[0],
                             initial_pose_estimate[1]));
    
    search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits()); //？

    const std::vector<Candidate2D> lowest_resolution_candidates =
        ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
    
    ROS_INFO("lowest_resolution_candidates size: %d", lowest_resolution_candidates.size());
    // for (size_t i = 0; i < lowest_resolution_candidates.size(); i++)
    // {
    //     if (lowest_resolution_candidates[i].score > 0.1)
    //     {
    //         ROS_INFO("score: %f", lowest_resolution_candidates[i].score);        
    //     }        
    // }
    

    const Candidate2D best_candidate = BranchAndBound(
        discrete_scans, search_parameters, lowest_resolution_candidates,
        precomputation_grid_stack_->max_depth(), min_score);
    
    // ROS_INFO("best_candidate x: %f, y: %f, z: %f, score: %f",
    //         best_candidate.x, best_candidate.y, best_candidate.orientation, best_candidate.score);

    if (best_candidate.score > min_score) 
    {
        *score = best_candidate.score;

        (*pose_estimate)[0] = initial_pose_estimate[0] + best_candidate.x;
        (*pose_estimate)[1] = initial_pose_estimate[1] + best_candidate.y;
        (*pose_estimate)[2] = initial_pose_estimate[2] + best_candidate.orientation;

        ROS_INFO("best score: %f, dx: %f dy: %f dz: %f", 
            *score, best_candidate.x, best_candidate.y, best_candidate.orientation);

        return true;
    }
    return false;
}

// 在最低分辨率地图上计算候选位置
std::vector<Candidate2D> FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const 
{
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);

  ScoreCandidates( // 计算候选解的得分
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

// 生产最低分辨率地图上的候选位置
std::vector<Candidate2D> FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const 
{
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) { // 遍历所有角度

    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;

    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;

    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }

  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);

  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  return candidates;
}

void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
    

      sum += precomputation_grid.GetValue(proposed_xy_index);
    //   if (sum > 0)
    //   {
    //       ROS_INFO("sum: %d", sum);
    //   }  
    }

    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
        
  }

  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans, // 旋转后的点云在栅格中的索引
    const SearchParameters& search_parameters, // 搜索参数
    const std::vector<Candidate2D>& candidates, const int candidate_depth, // 候选解， 候选解所在的层数
    float min_score) const 
{ // 设置的最低得分

    // ROS_INFO("candidate_depth: %d", candidate_depth); 

  if (candidate_depth == 0) { // 若候选解都在最后一层，返回最高得分
    return *candidates.begin();
  }

  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;

  for (const Candidate2D& candidate : candidates) {

    if (candidate.score <= min_score) {
      break;
    }

    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);

    for (int x_offset : {0, half_width}) { 
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }

        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }

    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);

    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
//   ROS_INFO("score: %f, depth: %d", best_high_resolution_candidate.score, candidate_depth);
  return best_high_resolution_candidate;
}

} // namespace mapping
} // namespace AVP
