#include "io.h"

namespace AVP
{
    
namespace mapping
{
    
sensor_msgs::ImagePtr GridMapToImage(const GridMap& grid)
{
    int width = grid.limits().cell_limits().num_x_cells;
    int height = grid.limits().cell_limits().num_y_cells;
    cv::Mat grid_map_image = cv::Mat::zeros(width, height, CV_8UC1);

    double max_value = grid.GetMaxCorrespondenceCost();
    double min_value = grid.GetMinCorrespondenceCost();

    for (size_t i = 0; i <  width; i++)
    {
        for (size_t j = 0; j < height; j++)
        {
            Eigen::Array2i cell{i,j};
            float grid_value = 1 - grid.GetCorrespondenceCost(cell);
            grid_map_image.at<unsigned char>(i,j) = (grid_value - min_value) / (max_value - min_value) * 255;
        }
    }
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grid_map_image).toImageMsg();
    return msg;
}

} // namespace mapping


} // namespace AVP
