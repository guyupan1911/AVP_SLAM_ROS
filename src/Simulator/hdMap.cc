#include "hdMap.h"
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>

namespace AVP{
    namespace simulating{

        HDMAP::HDMAP()
        {
            std::string path_to_map = "/home/data/AVPSLAM/map2.png";
            mImageMap = cv::imread(path_to_map, cv::IMREAD_COLOR);

            std::string path_to_map_param = "/home/data/AVPSLAM/map_param.bin";
            std::ifstream fin0(path_to_map_param.c_str(), std::ifstream::in | std::ifstream::binary);
            if (fin0)
            {
                fin0.read((char*)&mMapParam, sizeof(mMapParam));
                fin0.close();
            }

            //ROS_DEBUG("map origin: %f %f", mMapParam.OriginX, mMapParam.OriginY);

            // load semantic points
            std::string path_to_semantic = "/home/data/AVPSLAM/map.png";
            cv::Mat tempImSemantic = cv::imread(path_to_semantic, cv::IMREAD_UNCHANGED);
            
            ros_semantic_points.reset(new sensor_msgs::PointCloud2);

            readSemanticData(tempImSemantic);

        }

        sensor_msgs::ImageConstPtr HDMAP::drawPath(geometry_msgs::PoseStamped pose_true, 
    geometry_msgs::PoseStamped pose_noised)
        {
            std::unique_lock<std::mutex> lock(mMutexMap);

            if (std::pow((pose_true.pose.position.x - last_pose_true.pose.position.x),2)
            +std::pow((pose_true.pose.position.y-last_pose_true.pose.position.y),2)>0.1)
            {
                mvPoses_true.emplace_back(pose_true);
                mvPoses_noised.emplace_back(pose_noised);
            }
            //draw pose true
            std::pair<int, int> cell_true = pose_to_cell(pose_true);
            std::pair<int, int> cell_noised = pose_to_cell(pose_noised);

            drawPoint(cell_true, 2, cv::Scalar(0,255,0));
            drawPoint(cell_noised, 2, cv::Scalar(255,0,0));                
            
            last_pose_true = pose_true;
            
            cv::Mat imLocal;

            mImageMap.colRange(std::max(cell_true.first-400, 0), std::min(cell_true.first+400, mImageMap.cols))
            .rowRange(std::max(cell_true.second-400,0),std::min(cell_true.second+400,mImageMap.rows)).copyTo(imLocal);
            
            //draw car direction

            float x_head = 1.5*cos(pose_true.pose.orientation.z);
            float y_head = -1.5*sin(pose_true.pose.orientation.z);

            //ROS_DEBUG("xhead: %f, yhead: %f", x_head, y_head);

            int col_head = (x_head / mMapParam.Res) + imLocal.cols/2;
            int row_head = (y_head / mMapParam.Res) + imLocal.rows/2;

            //ROS_DEBUG("col head: %d, row head: %d", col_head, row_head);

            cv::arrowedLine(imLocal, cv::Point2f(imLocal.cols/2, imLocal.rows/2), cv::Point2f(col_head, row_head), cv::Scalar(255,0,0));

            sensor_msgs::ImageConstPtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imLocal).toImageMsg();
            return msg;
        }

        void HDMAP::readSemanticData(const cv::Mat& semantic)
        {
            for (int col = 0; col < semantic.cols; col++)
            {
                for (int row = 0; row < semantic.rows; row++)
                {
                    int value = semantic.at<unsigned char>(row,col);
                    if ((set_corners.find(value)!=set_corners.end()) || (set_edges.find(value)!=set_edges.end()))
                    {
                        pcl::PointXYZ point;
                        point.x = col * mMapParam.Res + mMapParam.OriginX;
                        point.y = -1 *(row * mMapParam.Res + mMapParam.OriginY);  
                        point.z = 0.f; 
                        all_semantic_points.points.emplace_back(point);
                    }
                }
            }
            kdTree.setInputCloud(all_semantic_points.makeShared());
            pcl::toROSMsg(all_semantic_points, *ros_semantic_points);
            ros_semantic_points->header.frame_id = "world";
            //ROS_DEBUG("finish loading %ld semantic points", all_semantic_points.points.size());
        }

        sensor_msgs::PointCloud2ConstPtr HDMAP::getCurrentSemanticPoints
        (const geometry_msgs::PoseStamped& carpose_true)
        {
            std::vector<int> indices;
            std::vector<float> distance;
            pcl::PointXYZ carpose = {static_cast<float>(carpose_true.pose.position.x), 
                                     static_cast<float>(carpose_true.pose.position.y),
                                     static_cast<float>(carpose_true.pose.orientation.z)};

            int size = kdTree.radiusSearch(carpose, search_radius, indices, distance);
            pcl::PointCloud<pcl::PointXYZ> semantic_points;
            pcl::copyPointCloud(all_semantic_points, indices, semantic_points);

            // transform to vehicle frame
            for (auto it = semantic_points.begin(); it != semantic_points.end(); it++)
            {
                // calculate T_w_vehicle;
                Eigen::Matrix3f T_w_vehicle;
                T_w_vehicle << cos(carpose_true.pose.orientation.z), -1*sin(carpose_true.pose.orientation.z), carpose_true.pose.position.x,
                               sin(carpose_true.pose.orientation.z), cos(carpose_true.pose.orientation.z), carpose_true.pose.position.y,
                               0,0,1;
                Eigen::Vector3f Pw;
                Pw << it->x, it->y, 1;

                Eigen::Vector3f Pvehicle = T_w_vehicle.inverse() * Pw;
                it->x = Pvehicle[0];
                it->y = Pvehicle[1]; 
            }
            

            sensor_msgs::PointCloud2Ptr ret(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(semantic_points, *ret);
            ret->header.frame_id = "vehicle";
            ret->header.stamp = carpose_true.header.stamp;
            return ret;
        }
    }
}