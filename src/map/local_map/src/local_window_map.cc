#include "local_map/include/local_window_map.h"


namespace planning_map
{
    void LocalWindowMap::init(ConfigLocalMap *cfg_local_map)
    {
        MapSize window_size{
            cfg_local_map->window_size_x,
            cfg_local_map->window_size_y
        };

        if (window_size.x <= 0 || window_size.y <= 0)
        {
            std::string err_str = "window size " + window_size.toString()
                + " is not valid";
            LMAP_ERROR_LOG("%s", err_str.c_str());
            throw std::invalid_argument(err_str.c_str());
        }

        if (window_size.x % 2 == 0)
            window_size.x += 1;
        if (window_size.y % 2 == 0)
            window_size.y += 1;

        initMapInfo(cfg_local_map->resolution,
            MapSize{window_size.x, window_size.y},
            Point{cfg_local_map->origin_pt_x, cfg_local_map->origin_pt_x},
            LOG_LOCAL_MAP);

        addLayer<LocalBaseLayer, LocalCell>("local_window", LocalCell(),
            m_log_name);
        m_local_window_layer
            = std::dynamic_pointer_cast<LocalBaseLayer>(getLayer("local_window"));

        addLayer<LocalBaseLayer, LocalCell>("local_new_received", LocalCell(),
            m_log_name);
        m_local_new_received_layer
            = std::dynamic_pointer_cast<LocalBaseLayer>(getLayer("local_new_received"));

        addLayer<InflateLayer, InflateCell>("inflate_window", InflateCell{
            InflateLayer::MASK_NONE, {0, 0}, {0.0f, 0.0f}, 0.0f}, m_log_name);
        m_inflate_window_layer
            = std::dynamic_pointer_cast<InflateLayer>(getLayer("inflate_window"));
        m_inflate_window_layer->setMaxInflateRange(cfg_local_map->max_inflate_range);
        m_inflate_window_layer->setMaskRange(std::vector<float>{
            0.0f,
            cfg_local_map->path_inflate_radius_offset
        });
    }
    
    void LocalWindowMap::inflateMap(std::vector<std::pair<RobotGrid, float>> &obs)
    {
        m_inflate_window_layer->inflateMap(std::vector<RobotGrid>{}, obs);
    }

}//planning_map