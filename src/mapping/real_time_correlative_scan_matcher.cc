#include "real_time_correlative_scan_matcher.h"

namespace AVP
{

namespace mapping
{

    float ComputeCandidateScore(const GridMap& probability_grid,
                                const DiscreteScan2D& discrete_scan,
                                int x_index_offset, int y_index_offset) {
    float candidate_score = 0.f;
    for (const Eigen::Array2i& xy_index : discrete_scan) {
        // 对每个点都加上像素坐标的offset, 相当于对点云进行平移
        const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                            xy_index.y() + y_index_offset);
        // 获取占用的概率
        const float probability = CorrespondenceCostToProbability(
            probability_grid.GetCorrespondenceCost(proposed_xy_index)
        );
        // 以概率为得分
        candidate_score += probability;
    }
    // 计算平均得分
    candidate_score /= static_cast<float>(discrete_scan.size());
    return candidate_score;
    }

    RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher()
    {
        image_transport::ImageTransport it(node_handle_);
        grid_map_image_pub_ = it.advertise("submap_grid_map_image",1);
    }


    double RealTimeCorrelativeScanMatcher::Match(const Eigen::Vector3d& predict_pose, 
                                                 const pcl::PointCloud<pcl::PointXYZ>& semantics_in_tracking_frame,                             
                                                 const GridMap& grid, Eigen::Vector3d& pose_estimated)
    {
        // if(1)        
        // {   
        //     grid_map_image_pub_.publish(GridMapToImage(grid));
        // }

        // transform point cloud in tracking from using predicted rotation
        pcl::PointCloud<pcl::PointXYZ> rotated_point_cloud = 
            TransformPointCloudAndReturn(semantics_in_tracking_frame, Eigen::Vector3d{0,0,predict_pose[2]});

        const SearchParameters search_parameters(5, M_PI / 6, rotated_point_cloud, grid.limits().resolution());

        const std::vector<pcl::PointCloud<pcl::PointXYZ>> rotated_scans =
            GenerateRotatedScans(rotated_point_cloud, search_parameters);

        const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
                                    grid.limits(), rotated_scans,
                                    Eigen::Translation2f(predict_pose[0],predict_pose[1])
                                    );

        std::vector<Candidate2D> candidates = GenerateExhaustiveSearchCandidates(search_parameters);

        ScoreCandidates(grid, discrete_scans, search_parameters, &candidates);

        const Candidate2D& best_candidate = *std::max_element(candidates.begin(), candidates.end());

        pose_estimated[0] = predict_pose[0] + best_candidate.x;
        pose_estimated[1] = predict_pose[1] + best_candidate.y;
        pose_estimated[2] = predict_pose[2] + best_candidate.orientation;


        // ROS_INFO("best score: %f", best_candidate.score);
        return best_candidate.score;

    }

    std::vector<Candidate2D> RealTimeCorrelativeScanMatcher::GenerateExhaustiveSearchCandidates(
        const SearchParameters& search_parameters) const {
    int num_candidates = 0;
    // 计算候选解的个数
    for (int scan_index = 0; scan_index != search_parameters.num_scans;
        ++scan_index) {
        const int num_linear_x_candidates =
            (search_parameters.linear_bounds[scan_index].max_x -
            search_parameters.linear_bounds[scan_index].min_x + 1);
        const int num_linear_y_candidates =
            (search_parameters.linear_bounds[scan_index].max_y -
            search_parameters.linear_bounds[scan_index].min_y + 1);
        num_candidates += num_linear_x_candidates * num_linear_y_candidates;
    }

    std::vector<Candidate2D> candidates;
    candidates.reserve(num_candidates);

    // 生成候选解, 候选解是由像素坐标的偏差组成的
    for (int scan_index = 0; scan_index != search_parameters.num_scans;
        ++scan_index) {
        for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
            x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
            ++x_index_offset) {
        for (int y_index_offset =
                search_parameters.linear_bounds[scan_index].min_y;
            y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
            ++y_index_offset) {
            candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                    search_parameters);
        }
        }
    }
    return candidates;
    }

    void RealTimeCorrelativeScanMatcher::ScoreCandidates(
        const GridMap& grid, const std::vector<DiscreteScan2D>& discrete_scans,
        const SearchParameters& search_parameters,
        std::vector<Candidate2D>* const candidates) const 
    {
        for (Candidate2D& candidate : *candidates)
        {
            candidate.score = ComputeCandidateScore(
                static_cast<const GridMap&>(grid),
                discrete_scans[candidate.scan_index], candidate.x_index_offset,
                candidate.y_index_offset);

        // 对得分进行加权
            candidate.score *=
                std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                        0.1 +
                                    std::abs(candidate.orientation) *
                                        0.1));
        }      
    }
}
}