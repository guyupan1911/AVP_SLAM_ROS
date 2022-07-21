#ifndef REAL_TIME_CORRELATIVE_SCAN_MATCHER_H
#define REAL_TIME_CORRELATIVE_SCAN_MATCHER_H

#include "submap.h"
#include "transform.h"
#include "correlative_scan_matcher.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "io.h"
#include <image_transport/image_transport.h>

namespace AVP
{
    
namespace mapping
{
    
class RealTimeCorrelativeScanMatcher
{
    public:
        RealTimeCorrelativeScanMatcher();
        double Match(const Eigen::Vector3d& predict_pose, const pcl::PointCloud<pcl::PointXYZ>& semantics_in_tracking_frame, const GridMap& grid, Eigen::Vector3d& pose_estimated);
    

        void ScoreCandidates(const GridMap& grid,
                             const std::vector<DiscreteScan2D>& discrete_scans,
                             const SearchParameters& search_parameters,
                             std::vector<Candidate2D>* candidates) const;


    private:

        std::vector<Candidate2D> GenerateExhaustiveSearchCandidates(const SearchParameters& search_parameters) const;

        ros::NodeHandle node_handle_;

        image_transport::Publisher grid_map_image_pub_;

};

} // namespace mapping


} // namespace AVP


#endif