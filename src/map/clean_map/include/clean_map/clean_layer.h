#ifndef MAP_CLEAN_LAYER_H
#define MAP_CLEAN_LAYER_H

#include "map/layer_map.h"

namespace planning_map
{
    class CleanLayer : public LayerMap<uint8_t>
    {
      public:

        enum CleanStatus
        {
            UNKNOWN = 0,
            COVERED,
            CLEANED,
            SIDE_CLEANED,
            CLEANED_PATH,
            SIDE_CLEANED_PATH,
            OBSTACLE,
            CLEAN_STATUS_MAX
        };

        using LayerMap<uint8_t>::LayerMap;

        void set(int32_t x, int32_t y, const uint8_t &cell);
        void set(const RobotGrid &grid, const uint8_t &cell);
    
      private:

        virtual bool isDefaultValueZero() const;
        virtual bool isOccupied(const RobotGrid &grid) const;
        virtual void setMapGridToMatCell(const RobotGrid &grid,
            uint8_t *mat_cell, uint8_t channel) const;
        virtual void setMapGridFromMatCell(const RobotGrid &grid,
            const uint8_t *mat_cell, uint8_t channel) const;
    };

    inline void CleanLayer::set(int32_t x, int32_t y, const uint8_t &cell)
    {
        uint8_t& target_cell = m_map[getIndex(x, y)];
        if (target_cell == UNKNOWN)
        {
            target_cell = cell;
        }
        else if (target_cell == CLEANED)
        {
            if (cell != UNKNOWN)
                target_cell = cell;
        }
        else if (target_cell == OBSTACLE)
        {
            return;
        }
    }

    inline void CleanLayer::set(const RobotGrid &grid, const uint8_t &cell)
    {
        set(grid.x, grid.y, cell);
    }

    inline bool CleanLayer::isDefaultValueZero() const
    {
        return m_default_value == 0;
    }

    inline bool CleanLayer::isOccupied(const RobotGrid &grid) const
    {
        return m_map[getIndex(grid)] != UNKNOWN;
    }

    inline void CleanLayer::setMapGridToMatCell(const RobotGrid &grid,
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

    inline void CleanLayer::setMapGridFromMatCell(const RobotGrid &grid,
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

    using CleanLayerPtr = std::shared_ptr<CleanLayer>;
}

#endif // MAP_CLEAN_LAYER_H