#ifndef MAP_LOCAL_WINDOW_MAP_H
#define MAP_LOCAL_WINDOW_MAP_H

#include "map/grid_map.h"
#include "map/inflate_layer.h"
#include "local_map/local_base_layer.h"
#include "local_map/include/local_map_log.h"
#include "local_map/include/local_map_config.h"

namespace planning_map
{
    class LocalWindowMap : public GridMap
    {
      public:

        void init(ConfigLocalMap *cfg_local_map);
        using GridMap::getLayer;
        // void updateLocalNewReceivedMap(LocalCell *new_map);
        template <typename TCell>
        TCell* getLayerMapAddr(std::string name);
        void setOriginPt(const Point &pt);
        void inflateMap(std::vector<std::pair<RobotGrid, float>> &obs);
        // VirtualBumpInfo detectVirtualBumpSignal(const RobotPose &cur_pose);

      private:

        LocalBaseLayerPtr m_local_window_layer;
        LocalBaseLayerPtr m_local_new_received_layer;
        InflateLayerPtr m_inflate_window_layer;
    };


    template <typename TCell>
    inline TCell* LocalWindowMap::getLayerMapAddr(std::string name)
    {
        auto it = m_layers.find(name);
        if (it != m_layers.end())
        {
            std::shared_ptr<LayerMap<TCell>> layer =
                std::dynamic_pointer_cast<LayerMap<TCell>>(it->second);
            return layer->getMapAddr();
        }
        return nullptr;
    }

    inline void LocalWindowMap::setOriginPt(const Point &pt)
    {
        m_map_info->setOriginPt(pt);
    }


}

#endif // MAP_WINDOW_MAP_H
