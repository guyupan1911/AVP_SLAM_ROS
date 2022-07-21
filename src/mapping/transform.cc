#include "transform.h"
namespace AVP
{
namespace mapping
{
    
void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Vector3d& P_w_vehicle)
{
    Eigen::Matrix4d T_w_vehicle;
    T_w_vehicle << cos(P_w_vehicle[2]), -1*sin(P_w_vehicle[2]), 0, P_w_vehicle[0],
                    sin(P_w_vehicle[2]), cos(P_w_vehicle[2]), 0, P_w_vehicle[1],
                    0,0,1,0,
                    0,0,0,1;

    TransformPointCloud(cloud, T_w_vehicle);
}

void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Matrix4d& T)
{
    pcl::transformPointCloud(cloud, cloud, T);
}

pcl::PointCloud<pcl::PointXYZ> TransformPointCloudAndReturn(const pcl::PointCloud<pcl::PointXYZ>& cloud,const Eigen::Vector3d& P_w_tracking)
{
    pcl::PointCloud<pcl::PointXYZ> ret;
    pcl::copyPointCloud(cloud, ret);


    Eigen::Vector3d P = P_w_tracking;
    TransformPointCloud(ret, P);

    return ret;
}


} // namespace mapping
} // namespace AVP