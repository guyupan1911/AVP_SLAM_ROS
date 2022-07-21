#ifndef PROBABILITY_GRID_RANGE_DATA_INSERTER_H
#define PROBABILITY_GRID_RANGE_DATA_INSERTER_H

#include "probability_values.h"
#include "gridmap.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

namespace AVP
{
    
namespace mapping
{
    
class ProbabilityGridRangeDataInserter
{
    public:
    ProbabilityGridRangeDataInserter();

    void Insert(const pcl::PointCloud<pcl::PointXYZ>& range_data, GridMap* grid);


    private:
        const std::vector<uint16> hit_table_;
        const std::vector<uint16> miss_table_;

};

} // namespace mapping
} // namespace AVP


#endif