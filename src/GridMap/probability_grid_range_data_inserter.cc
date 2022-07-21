#include "probability_grid_range_data_inserter.h"

namespace AVP
{
    
namespace mapping
{

constexpr int kSubpixelScale = 1000;

void GrowAsNeeded(const pcl::PointCloud<pcl::PointXYZ>& range_data, GridMap* const grid) 
{
    Eigen::AlignedBox2f bounding_box; // Eigenvalue::AlignedBox与坐标轴对齐的box
    constexpr float kPadding = 1e-6f;
    for (const pcl::PointXYZ& point : range_data.points) 
    {
        Eigen::Vector3f p{point.x, point.y, point.z};
        bounding_box.extend(p.head<2>()); // 扩展box包含点hit
    }
    // 是否对地图进行扩张
    grid->GrowLimits(bounding_box.min() -
                                kPadding * Eigen::Vector2f::Ones());
    grid->GrowLimits(bounding_box.max() +
                                kPadding * Eigen::Vector2f::Ones());
}

void CastRays(const pcl::PointCloud<pcl::PointXYZ>& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              GridMap* grid) 
{
    GrowAsNeeded(range_data, grid);

    const MapLimits& limits = grid->limits(); // 概率网格的参数
    const double superscaled_resolution = limits.resolution() / kSubpixelScale; //亚像素的分辨率
    const MapLimits superscaled_limits(
        superscaled_resolution, limits.max(),
        CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                    limits.cell_limits().num_y_cells * kSubpixelScale)); // 亚像素概率网格的参数

    std::vector<Eigen::Array2i> ends;
    ends.reserve(range_data.points.size());
    for (const pcl::PointXYZ& point : range_data.points) 
    {
        Eigen::Vector3f p{point.x, point.y, point.z};
        ends.push_back(superscaled_limits.GetCellIndex(p.head<2>())); // 计算亚像素的网格位置
        // 更新hit点的栅格值
        grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table); // 更新网格的值
    }
}
    
ProbabilityGridRangeDataInserter::ProbabilityGridRangeDataInserter()
:hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.55))), // p(hit)/P(miss)
    miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.49)))
{}

void ProbabilityGridRangeDataInserter::Insert(const pcl::PointCloud<pcl::PointXYZ>& range_data, GridMap* grid)
{
    CastRays(range_data, hit_table_, miss_table_, grid);
    grid->FinishUpdate();
}



} // namespace mapping


} // namespace AVP
