#ifndef MAP_LOCAL_BASE_LAYER_H
#define MAP_LOCAL_BASE_LAYER_H

#include "map/layer_map.h"
#include "misc/footprint.h"
#include "map/map_cost_value.h"

namespace planning_map
{
    struct LocalCell
    {
        LocalCell() {}
        LocalCell(uint8_t _status,
                  uint8_t _type,
                  int32_t _cluster_id,
                  uint8_t _facing,
                  float _valid_range_min,
                  float _valid_range_max,
                  uint64_t _timestamp)
            : status(_status),
              type(_type),
              cluster_id(_cluster_id),
              facing(_facing),
              valid_range_min(_valid_range_min),
              valid_range_max(_valid_range_max),
              timestamp(_timestamp)
        {}

        uint8_t status          = 0;
        uint8_t type            = 0;
        int32_t cluster_id      = 0;
        uint8_t facing          = 0;
        float valid_range_min   = 0.0f;
        float valid_range_max   = 0.0f;
        uint64_t timestamp      = 0;
    };

    class LocalBaseLayer : public LayerMap<LocalCell>
    {
      public:

        using LayerMap<LocalCell>::LayerMap;

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

    inline bool LocalBaseLayer::isDefaultValueZero() const
    {
        return (m_default_value.status == 0
            && m_default_value.type == 0
            && m_default_value.cluster_id == 0
            && m_default_value.facing == 0
            && m_default_value.valid_range_min == 0.0f
            && m_default_value.valid_range_max == 0.0f
            && m_default_value.timestamp == 0);
    }

    inline bool LocalBaseLayer::isOccupied(const RobotGrid &grid) const
    {
        return (m_map[getIndex(grid)].status != m_default_value.status);
    }

    inline void LocalBaseLayer::setMapGridToMatCell(const RobotGrid &grid,
        uint8_t *mat_cell, uint8_t channel) const
    {
        if (channel == CV_8UC1)
        {
            *mat_cell = m_map[getIndex(grid)].status;
        }
        else if (channel == CV_8UC3)
        {
            *mat_cell = m_map[getIndex(grid)].status;
            *(mat_cell + 1) = m_map[getIndex(grid)].status;
            *(mat_cell + 2) = m_map[getIndex(grid)].status;
        }
    }

    inline void LocalBaseLayer::setMapGridFromMatCell(const RobotGrid &grid,
        const uint8_t *mat_cell, uint8_t channel) const
    {
        if (channel == CV_8UC1)
        {
            m_map[getIndex(grid)].status = *mat_cell;
        }
        else if (channel == CV_8UC3)
        {
            m_map[getIndex(grid)].status = *mat_cell;
            m_map[getIndex(grid)].status = *(mat_cell + 1);
            m_map[getIndex(grid)].status = *(mat_cell + 2);
        }
    }

    using LocalBaseLayerPtr = std::shared_ptr<LocalBaseLayer>;
}

#endif // MAP_LOCAL_BASE_LAYER_H