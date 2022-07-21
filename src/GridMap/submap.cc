#include "submap.h"

namespace AVP
{
namespace mapping
{
    
    Submap::Submap(const Eigen::Vector3d& local_pose, std::unique_ptr<GridMap> grid, 
            ValueConversionTables* conversion_tables):local_pose_(local_pose),conversion_tables_(conversion_tables)
    {
        grid_ = std::move(grid);
    }

    void Submap::InsertSemanticData(const pcl::PointCloud<pcl::PointXYZ>& semantic_data,
                                    ProbabilityGridRangeDataInserter* inserter)
    {
        inserter->Insert(semantic_data, grid_.get());

        data_ += semantic_data;
        set_num_accumulated_semantic_data(num_accumulated_semantic_data()+1);
    }

    std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::InsertSemanticData(
        const pcl::PointCloud<pcl::PointXYZ>& semantic_data, const Eigen::Vector3d& pose_estimated)
    {
        if (submaps_.empty() || submaps_.back()->num_accumulated_semantic_data() == max_semantic_data_in_submap)
        {
            AddSubmap(pose_estimated);
        }

        for(auto elem : submaps_)
        {
            elem->InsertSemanticData(semantic_data, range_data_inserter.get());
        }

        if (submaps_.front()->num_accumulated_semantic_data() == 2*max_semantic_data_in_submap)
        {
            submaps_.front()->set_insertion_finished(true);
        }

        return submaps();
    }

    void ActiveSubmaps::AddSubmap(const Eigen::Vector3d& origin)
    {
        if (submaps_.size()>=2)
        {
            submaps_.erase(submaps_.begin());
        }
        submaps_.push_back(std::unique_ptr<Submap>(new Submap{origin, std::unique_ptr<GridMap>(CreateGrid(origin).release()),
         &conversion_tables_}));
    }

    std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::submaps() const
    {
        return std::vector<std::shared_ptr<const Submap>>(submaps_.begin(),submaps_.end());
    }

    std::unique_ptr<GridMap> ActiveSubmaps::CreateGrid(const Eigen::Vector3d& origin)
    {
        constexpr int kInitialSubmapSize = 100;
        float resolution = 0.05; // param: grid_options_2d.resolution
      
        MapLimits limit{resolution, 
                    origin.head<2>().cast<double>() + 0.5 * kInitialSubmapSize *resolution * Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)};

        return std::unique_ptr<GridMap>(
            new GridMap(limit, &conversion_tables_)
            );      
    }



} // namespace mapping
} // namespace AVP
