
#include "map/grid_map.h"

namespace planning_map
{
    GridMap::GridMap()
        : m_initialized(false)
    {}

    GridMap::~GridMap() {}

    void GridMap::init(ConfigManager &cfg_mgr)
    {
        return;
    }

    MapInfo& GridMap::getMapInfo() const
    {
        std::shared_lock<shared_recursive_mutex> lock(m_map_info_mtx);
        return *m_map_info;
    }
    

    void GridMap::deleteLayer(LayerNames &&names)
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        for (auto &name : names)
        {
            auto it = m_layers.find(name);
            if (it != m_layers.end())
            {
                m_layers.erase(it);
            }
        }
    }

    void GridMap::resetLayer(LayerNames &&names)
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        for (auto &name : names)
        {
            auto it = m_layers.find(name);
            if (it != m_layers.end())
            {
                it->second->reset();
            }
        }
    }

    cv::Mat GridMap::saveMapToMat(std::string name, MapSaveOption opt,
        RobotGridRect region, uint8_t channel, uint32_t padding)
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        auto it = m_layers.find(name);
        if (it == m_layers.end())
        {
            MAP_WARN_LOG(m_log_name, "%s layer is not found", name.c_str());
            return cv::Mat{};
        }

        return it->second->saveMapToMat(region, opt, channel, padding);
    }

    bool GridMap::saveMapToFile(std::string file_name, std::string name,
        MapSaveOption opt, RobotGridRect region, uint8_t channel, uint32_t padding)
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        cv::Mat &&map = saveMapToMat(name, opt, region, channel, padding);
        if (map.size() == cv::Size(0, 0))
        {
            return false;
        }

        cv::imwrite(file_name, map);
        return true;
    }

    bool GridMap::loadMapFromMat(std::string name, RobotGridRect region,
        const cv::Mat &map, uint8_t channel)
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        auto it = m_layers.find(name);
        if (it == m_layers.end())
        {
            MAP_WARN_LOG(m_log_name, "%s layer is not found", name.c_str());
            return false;
        }
        if (map.rows != (region.max_grid.x - region.min_grid.x + 1)
            || map.cols != (region.max_grid.y - region.min_grid.y + 1))
        {
            MAP_WARN_LOG(m_log_name, "region size %s is incompatible with "
                "input map size { %d, %d }", region.toString().c_str(),
                map.rows, map.cols);
            return false;
        }

        return it->second->loadMapFromMat(region, map, channel);
    }

    bool GridMap::loadMapFromFile(std::string name, RobotGridRect region,
        std::string file_name, uint8_t channel)
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        cv::Mat map = cv::imread(file_name);;
        // cv::cvtColor(map, map, CV_BGR2GRAY);

        return loadMapFromMat(name, region, map, channel);
    }

    bool GridMap::loadMapFromFilePgm(std::string name, RobotGridRect region,
                                     std::string file_name, uint8_t channel)
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);

        cv::Mat map = cv::imread(file_name, 0);
        cv::Mat srcCopy = cv::Mat(map.rows, map.cols, map.depth());
        cv::transpose(map, srcCopy);

        cv::flip(srcCopy, srcCopy, 0);

        for (int i = 0; i < srcCopy.rows; i++)
        {
            for (int j = 0; j < srcCopy.cols; j++)
            {
                if (srcCopy.at<unsigned char>(i, j) <= 128)
                    srcCopy.at<unsigned char>(i, j) = 0;
                else
                    srcCopy.at<unsigned char>(i, j) = 255;
            }
        }
        return loadMapFromMat(name, region, srcCopy, channel);
    }

    void GridMap::resetMap()
    {
        std::unique_lock<shared_recursive_mutex> map_lock(m_map_mtx);
        for (auto & [name, layer] : m_layers)
        {
            layer->reset();
        } 
    }

    void GridMap::initMapInfo(float resolution, MapSize size,
        const Point &origin_pt, std::string log_name)
    {
        if (size.x <= 0 || size.y <= 0)
        {
            std::string err_str = "map size " + size.toString()
                + " is not valid";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::invalid_argument(err_str.c_str());
        }

        m_map_info = std::make_shared<MapInfo>();
        m_map_info->setInfo(resolution, size, origin_pt);
    }
    void GridMap::initMapInfoLocal(float resolution, float localResolution, MapSize size, MapSize localSize,
                              const Point &origin_pt, const Point &local_origin_pt, std::string log_name)
    {
        if (size.x <= 0 || size.y <= 0)
        {
            std::string err_str = "map size " + size.toString() + " is not valid";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::invalid_argument(err_str.c_str());
        }
        if (localSize.x <= 0 || localSize.y <= 0)
        {
            std::string err_str = "map localSize " + localSize.toString() + " is not valid";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::invalid_argument(err_str.c_str());
        }
        m_map_info = std::make_shared<MapInfo>();
        m_map_info->setInfoLocal(resolution, localResolution, size, localSize, origin_pt, local_origin_pt);
    }
}//planning_map