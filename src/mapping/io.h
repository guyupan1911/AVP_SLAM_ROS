#ifndef IO_H
#define IO_H

#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include "gridmap.h"
#include <cv_bridge/cv_bridge.h>



namespace AVP
{
    
namespace mapping
{
    
sensor_msgs::ImagePtr GridMapToImage(const GridMap& grid);

} // namespace mapping


} // namespace AVP


#endif