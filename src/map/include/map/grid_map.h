
#ifndef MAP_GRID_MAP_H
#define MAP_GRID_MAP_H

#include <map>
#include <set>
#include <shared_mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <mutex/shared_recursive_mutex.h>

#include "config/config_manager.h"

#include "layer.h"
#include "layer_map.h"
#include "map_cost_value.h"

using planning_utils::ConfigManager;

namespace planning_map
{
    using LayerNames = std::set<std::string>;

    class GridMap
    {
    public:
        GridMap();
        virtual ~GridMap();

        virtual void init(ConfigManager &cfg_mgr);
        MapInfo &getMapInfo() const;

        template <typename TLayer, typename TCell>
        bool addLayer(std::string name, TCell &&default_value,
                      std::string log_name);
        void deleteLayer(LayerNames &&names);
        void resetLayer(LayerNames &&names);
        cv::Mat saveMapToMat(std::string name, MapSaveOption opt,
                             RobotGridRect region, uint8_t channel = CV_8UC1, uint32_t padding = 5);
        bool saveMapToFile(std::string file_name, std::string name,
                           MapSaveOption opt, RobotGridRect region, uint8_t channel = CV_8UC1,
                           uint32_t padding = 5);
        bool loadMapFromMat(std::string name, RobotGridRect region,
                            const cv::Mat &map, uint8_t channel = CV_8UC1);
        bool loadMapFromFile(std::string name, RobotGridRect region,
                             std::string file_name, uint8_t channel = CV_8UC1);
        bool loadMapFromFilePgm(std::string name, RobotGridRect region,
                                std::string file_name, uint8_t channel = CV_8UC1);
        void resetMap();

        template <typename TCell>
        bool get(std::string name, const RobotGrid &grid, TCell &cell) const;
        template <typename TCell>
        void set(std::string name, const RobotGrid &grid, const TCell &cell);

        bool inRange(const Point &pt) const;
        bool inRange(const RobotGrid &grid) const;
        RobotGrid poseToGrid(const RobotPose &pose) const;
        RobotGrid poseToGridWithoutBoundary(const RobotPose &pose) const;
        RobotPose gridToPose(const RobotGrid &grid) const;

        LayerPtr getLayer(std::string name) const;

    protected:
        void initMapInfo(float resolution, MapSize size,
                         const Point &origin_pt, std::string log_name);
        Layer &operator[](std::string name);

        bool m_initialized;
        MapInfoPtr m_map_info;
        std::string m_log_name;
        std::map<std::string, LayerPtr> m_layers;

        mutable shared_recursive_mutex m_map_mtx;
        mutable shared_recursive_mutex m_map_info_mtx;

        void initMapInfoLocal(float resolution, float localResolution, MapSize size, MapSize localSize,
                         const Point &origin_pt, const Point &local_origin_pt, std::string log_name);
       
    };

    template <typename TLayer, typename TCell>
    bool GridMap::addLayer(std::string name, TCell &&default_value,
                           std::string log_name)
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        if (m_layers.find(name) != m_layers.end())
            return false;
        Layer *layer = reinterpret_cast<Layer *>(new TLayer(name, m_map_info,
                                                            default_value, log_name));
        m_layers.insert({name, LayerPtr(layer)});
        return true;
    }

    template <typename TCell>
    inline bool GridMap::get(std::string name, const RobotGrid &grid, TCell &cell) const
    {
        std::shared_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        std::shared_ptr<LayerMap<TCell>> layer = std::reinterpret_pointer_cast<LayerMap<TCell>>(getLayer(name));
        if (!layer)
        {
            MAP_WARN_LOG(m_log_name, "%s layer is not found", name.c_str());
            return false;
        }

        cell = (*layer)[grid];
        return true;
    }

    template <typename TCell>
    inline void GridMap::set(std::string name, const RobotGrid &grid, const TCell &cell)
    {
        std::shared_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        std::shared_ptr<LayerMap<TCell>> layer = std::reinterpret_pointer_cast<LayerMap<TCell>>(getLayer(name));
        if (!layer)
        {
            MAP_WARN_LOG(m_log_name, "%s layer is not found", name.c_str());
            return;
        }

        layer->set(grid, cell);
    }

    inline bool GridMap::inRange(const Point &pt) const
    {
        std::shared_lock<shared_recursive_mutex> lock(m_map_info_mtx);
        return m_map_info->range.inRange(pt);
    }

    inline bool GridMap::inRange(const RobotGrid &grid) const
    {
        std::shared_lock<shared_recursive_mutex> lock(m_map_info_mtx);
        return m_map_info->grid_range.inRange(grid);
    }

    inline RobotGrid GridMap::poseToGrid(const RobotPose &pose) const
    {
        std::shared_lock<shared_recursive_mutex> lock(m_map_info_mtx);
        Point diff = pose.pt - m_map_info->origin_pt;
        diff = diff / m_map_info->resolution;
        RobotGrid grid_diff{
            static_cast<int32_t>(roundf(diff.x)),
            static_cast<int32_t>(roundf(diff.y))};

        RobotGrid grid = m_map_info->origin_grid + grid_diff;
        if (!inRange(grid))
        {
            std::string err_str = pose.toString() + " => " + grid.toString() + " is out of map range, origin_pt = " + m_map_info->origin_pt.toString() + "map range = %s" + m_map_info->grid_range.toString();
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            TRIG_SEGFAULT();
            throw std::out_of_range(err_str.c_str());
        }

        return grid;
    }

    inline RobotGrid GridMap::poseToGridWithoutBoundary(const RobotPose &pose) const
    {
        std::shared_lock<shared_recursive_mutex> lock(m_map_info_mtx);
        Point diff = pose.pt - m_map_info->origin_pt;
        diff = diff / m_map_info->resolution;
        RobotGrid grid_diff{
            static_cast<int32_t>(roundf(diff.x)),
            static_cast<int32_t>(roundf(diff.y))};

        RobotGrid grid = m_map_info->origin_grid + grid_diff;
        if (!inRange(grid))
            MAP_WARN_LOG(m_log_name, "%s => %s  is out of map range.",
                         pose.toString().c_str(), grid.toString().c_str());

        return grid;
    }

    inline RobotPose GridMap::gridToPose(const RobotGrid &grid) const
    {
        std::shared_lock<shared_recursive_mutex> lock(m_map_info_mtx);
        RobotGrid grid_diff = grid - m_map_info->origin_grid;
        Point diff{grid_diff.x * m_map_info->resolution,
                     grid_diff.y * m_map_info->resolution};

        return {m_map_info->origin_pt + diff, 0.0f};
    }

    inline LayerPtr GridMap::getLayer(std::string name) const
    {
        // std::shared_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        auto it = m_layers.find(name);
        if (it == m_layers.end())
        {
            return nullptr;
        }
        return it->second;
    }

    inline Layer &GridMap::operator[](std::string name)
    {
        auto it = m_layers.find(name);
        if (it == m_layers.end())
        {
            std::string err_str = name + "layer is not found.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::out_of_range(err_str.c_str());
        }

        return *(it->second);
    }
} // namespace planning_map

#endif // MAP_GRID_MAP_H