#include "gridmap.h"

namespace AVP
{
    
namespace mapping
{
    
GridMap::GridMap(const MapLimits& limits, ValueConversionTables* conversion_tables)
    : limits_(limits), conversion_tables_(conversion_tables),
    correspondence_cost_cells_(
          limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
          kUnknownCorrespondenceValue)
{
    value_to_correspondence_cost_table_ = conversion_tables->GetConversionTable(
          max_correspondence_cost_, min_correspondence_cost_,
          max_correspondence_cost_);
}

void GridMap::GrowLimits(const Eigen::Vector2f& point) { // 输入的是boundingbox
  GrowLimits(point, {mutable_correspondence_cost_cells()}, // 使用{}初始化vector
             {kUnknownCorrespondenceValue});
}

// 根据坐标决定是否对地图进行扩大
void GridMap::GrowLimits(const Eigen::Vector2f& point, // bounding box的点
                        const std::vector<std::vector<uint16>*>& grids, //
                        const std::vector<uint16>& grids_unknown_cell_values) {

  // 判断该点是否在地图坐标系内
  while (!limits_.Contains(limits_.GetCellIndex(point))) {
    const int x_offset = limits_.cell_limits().num_x_cells / 2;
    const int y_offset = limits_.cell_limits().num_y_cells / 2;
    // 将xy扩大至2倍, 中心点不变, 向四周扩大
    const MapLimits new_limits(
        limits_.resolution(),
        limits_.max() +
            limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
        CellLimits(2 * limits_.cell_limits().num_x_cells,
                   2 * limits_.cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    // 老坐标系的原点在新坐标系下的一维像素坐标
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;

    // grids.size()为1
    for (size_t grid_index = 0; grid_index < grids.size(); ++grid_index) {
      std::vector<uint16> new_cells(new_size,
                                    grids_unknown_cell_values[grid_index]);
      // 将老地图的栅格值复制到新地图上
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
              (*grids[grid_index])[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      // 将新地图替换老地图, 拷贝
      *grids[grid_index] = new_cells;
    } // end for
    // 更新地图尺寸
    limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      // 将known_cells_box_的x与y进行平移到老地图的范围上
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

bool GridMap::ApplyLookupTable(const Eigen::Array2i& cell_index, const std::vector<uint16>& table)
{
    const int flat_index = ToFlatIndex(cell_index); // 转换为内部的网格索引
    // 获取对应栅格的指针
    uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
    // 对处于更新状态的栅格, 不再进行更新了
    if (*cell >= kUpdateMarker) {
        return false;
    }
    // 标记这个索引的栅格已经被更新过
    mutable_update_indices()->push_back(flat_index);
    // 更新栅格值
    *cell = table[*cell];
    // 更新bounding_box
    mutable_known_cells_box()->extend(cell_index.matrix());
    return true;
}

void GridMap::FinishUpdate() 
{
  while (!update_indices_.empty()) 
  {
      correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
      update_indices_.pop_back();
  }
}

} // namespace mapping
} // namespace AVP
