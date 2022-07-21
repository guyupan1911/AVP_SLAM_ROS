#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "port.h"
#include "value_conversion_tables.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "probability_values.h"

namespace AVP
{

namespace mapping
{

struct CellLimits 
{
    CellLimits() = default;
    CellLimits(int init_num_x_cells, int init_num_y_cells)
        : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

    int num_x_cells = 0;
    int num_y_cells = 0;
};

class MapLimits 
{
    public:

    MapLimits(const double resolution, const Eigen::Vector2d& max,
                const CellLimits& cell_limits)
        : resolution_(resolution), max_(max), cell_limits_(cell_limits) {}

    double resolution() const { return resolution_; }

    const Eigen::Vector2d& max() const { return max_; }

    const CellLimits& cell_limits() const { return cell_limits_; }


    Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const 
    {
        return Eigen::Array2i
        (
            common::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
            common::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5)
        );
    }


    Eigen::Vector2f GetCellCenter(const Eigen::Array2i cell_index) const 
    {
        return {max_.x() - resolution() * (cell_index[1] + 0.5),
                max_.y() - resolution() * (cell_index[0] + 0.5)};
    }


    bool Contains(const Eigen::Array2i& cell_index) const 
    {
        return (Eigen::Array2i(0, 0) <= cell_index).all() &&
            (cell_index <
                Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
                .all();
    }

    private:
    double resolution_; 
    Eigen::Vector2d max_;   
    CellLimits cell_limits_;
};

class GridMap
{
    public:

    GridMap(const MapLimits& limits, ValueConversionTables* conversion_tables);

    void FinishUpdate();

    const MapLimits& limits() const { return limits_; }

    bool ApplyLookupTable(const Eigen::Array2i& cell_index, const std::vector<uint16>& table);

    void GrowLimits(const Eigen::Vector2f& point);

    void GrowLimits(const Eigen::Vector2f& point,
                    const std::vector<std::vector<uint16>*>& grids,
                    const std::vector<uint16>& grids_unknown_cell_values);

    std::vector<int>* mutable_update_indices() { return &update_indices_; }
    Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }
    std::vector<uint16>* mutable_correspondence_cost_cells() 
    {
        return &correspondence_cost_cells_;
    }

    int ToFlatIndex(const Eigen::Array2i& cell_index) const {
        return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
    }

    const std::vector<uint16>& correspondence_cost_cells() const {
        return correspondence_cost_cells_;
    }

    float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const {
        if (!limits().Contains(cell_index)) return max_correspondence_cost_;
        return (*value_to_correspondence_cost_table_)
            [correspondence_cost_cells()[ToFlatIndex(cell_index)]];
    }

    float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

    // Returns the maximum possible correspondence cost.
    float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

    public:

        MapLimits limits_;  
        
        std::vector<uint16> correspondence_cost_cells_; 
        float min_correspondence_cost_ = 0.1;
        float max_correspondence_cost_ = 0.9;
        std::vector<int> update_indices_;  
        Eigen::AlignedBox2i known_cells_box_;   

        ValueConversionTables* conversion_tables_;
        const std::vector<float>* value_to_correspondence_cost_table_;
};

} // namespace mapping

} // namespace AVP


#endif