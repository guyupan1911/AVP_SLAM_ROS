#ifndef HD_MAP_H
#define HD_MAP_

#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <mutex>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <boost/shared_ptr.hpp>
#include <unordered_set>


namespace AVP{
namespace simulating{

struct MapParam
{
    float OriginX; //原点的像素位置
    float OriginY;
    float Res; // 分辨率
    //uchar CentreCost;
    unsigned char CentreCost;
    int SizeX;
    int SizeY;
};

class HDMAP{

public:
    HDMAP();


    sensor_msgs::ImageConstPtr drawPath(geometry_msgs::PoseStamped pose_true, 
    geometry_msgs::PoseStamped pose_noised);

    void readSemanticData(const cv::Mat& semantic);

    sensor_msgs::PointCloud2::ConstPtr getAllSemanticPoints()
    {
        return ros_semantic_points;
    }

    void savePath()
    {
        std::unique_lock<std::mutex> lock(mMutexMap);
        cv::imwrite("/home/data/AVPSLAM/map/path.png", mImageMap);
        //ROS_DEBUG("path saved");
    }

    sensor_msgs::PointCloud2ConstPtr getCurrentSemanticPoints(const geometry_msgs::PoseStamped& carpose_true);

private:

    std::pair<int,int> pose_to_cell(geometry_msgs::PoseStamped& pose)
    {
        int cell_x = (pose.pose.position.x - mMapParam.OriginX) / mMapParam.Res;
        int cell_y = (-1*pose.pose.position.y - mMapParam.OriginY) / mMapParam.Res;
        return std::make_pair(cell_x, cell_y);
    }

    void drawPoint(std::pair<int, int>& cell_point, int radius, cv::Scalar color)
    {
        cv::circle(mImageMap, cv::Point(cell_point.first, cell_point.second), radius, color, -1);
    }

    cv::Mat mImageMap;
    MapParam mMapParam;
    std::vector<geometry_msgs::PoseStamped> mvPoses_true;
    std::vector<geometry_msgs::PoseStamped> mvPoses_noised;

    geometry_msgs::PoseStamped last_pose_true;

    std::mutex mMutexMap;

    pcl::PointCloud<pcl::PointXYZ> all_semantic_points;
    pcl::search::KdTree<pcl::PointXYZ> kdTree;
    const float search_radius = 5.f;
    sensor_msgs::PointCloud2::Ptr ros_semantic_points;

    std::unordered_set<int> set_corners = {16,46,49,52,55,58,61,64};
    std::unordered_set<int> set_edges = {};
    
};

} // namespace simulating
} // namespace AVP

#endif