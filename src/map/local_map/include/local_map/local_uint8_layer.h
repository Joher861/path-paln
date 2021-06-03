#ifndef MAP_LOCAL_UINT8_BASE_LAYER_H
#define MAP_LOCAL_UINT8_BASE_LAYER_H

#include "map/layer_map.h"
#include "misc/footprint.h"
#include "map/map_cost_value.h"

namespace planning_map
{


    class LocalCharBaseLayer : public LayerMap<uint8_t>
    {
      public:

        using LayerMap<uint8_t>::LayerMap;

        std::vector<std::pair<RobotGrid, float>> getInflateObstacles(
            const RobotGridRect &range, float inflate_radius);

      protected:

        virtual bool isDefaultValueZero() const;
        virtual bool isOccupied(const RobotGrid &grid) const;
        virtual void setMapGridToMatCell(const RobotGrid &grid,
            uint8_t *mat_cell, uint8_t channel) const;
        virtual void setMapGridFromMatCell(const RobotGrid &grid,
            const uint8_t *mat_cell, uint8_t channel) const;
    };

    inline bool LocalCharBaseLayer::isDefaultValueZero() const
    {
        return (m_default_value == 0);
    }

    inline bool LocalCharBaseLayer::isOccupied(const RobotGrid &grid) const
    {
        return (m_map[getIndex(grid)] != m_default_value);
    }

    inline void LocalCharBaseLayer::setMapGridToMatCell(const RobotGrid &grid,
        uint8_t *mat_cell, uint8_t channel) const
    {
        if (channel == CV_8UC1)
        {
            *mat_cell = m_map[getIndex(grid)];
        }
        else if (channel == CV_8UC3)
        {
            *mat_cell = m_map[getIndex(grid)];
            *(mat_cell + 1) = m_map[getIndex(grid)];
            *(mat_cell + 2) = m_map[getIndex(grid)];
        }
    }

    inline void LocalCharBaseLayer::setMapGridFromMatCell(const RobotGrid &grid,
        const uint8_t *mat_cell, uint8_t channel) const
    {
        if (channel == CV_8UC1)
        {
            m_map[getIndex(grid)] = *mat_cell;
        }
        else if (channel == CV_8UC3)
        {
            m_map[getIndex(grid)] = *mat_cell;
            m_map[getIndex(grid)] = *(mat_cell + 1);
            m_map[getIndex(grid)] = *(mat_cell + 2);
        }
    }

    using LocalCharBaseLayerPtr = std::shared_ptr<LocalCharBaseLayer>;
}

#endif // MAP_LOCAL_BASE_LAYER_H