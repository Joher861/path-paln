#ifndef MAP_SLAM_LAYER_H
#define MAP_SLAM_LAYER_H

#include "string.h"

#include "map/layer_map.h"

namespace planning_map
{
    class SlamLayer : public LayerMap<uint8_t>
    {
      public:

        enum MapStatus
        {
            UNKNOWN     = 1,
            FREE        = 255,
            OCCUPIED    = 0,
        };

        using LayerMap<uint8_t>::LayerMap;

        void updateMap(uint8_t *src);
        void getFreesAndObstacles(std::vector<RobotGrid> &frees,
            std::vector<RobotGrid> &obs);

      private:

        using LayerMap<uint8_t>::set;
        virtual bool isDefaultValueZero() const;
        virtual bool isOccupied(const RobotGrid &grid) const;
        virtual void setMapGridToMatCell(const RobotGrid &grid,
            uint8_t *mat_cell, uint8_t channel) const;
        virtual void setMapGridFromMatCell(const RobotGrid &grid,
            const uint8_t *mat_cell, uint8_t channel) const;
    };

    void SlamLayer::updateMap(uint8_t *src)
    {
        if (!src)
            return;
        
        memcpy(m_map, src, m_total_size * m_cell_size);
    }

    void SlamLayer::getFreesAndObstacles(std::vector<RobotGrid> &frees,
        std::vector<RobotGrid> &obs)
    {
        frees.clear();
        obs.clear();
        for (size_t i = 0; i < m_total_size; ++i)
        {
            uint8_t status = m_map[i];
            if (status == FREE)
                frees.push_back(parseIndex(i));
            else if (status == OCCUPIED)
                obs.push_back(parseIndex(i));
        }
    }

    inline bool SlamLayer::isDefaultValueZero() const
    {
        return m_default_value == 0;
    }

    inline bool SlamLayer::isOccupied(const RobotGrid &grid) const
    {
        return m_map[getIndex(grid)] != UNKNOWN;
    }

    inline void SlamLayer::setMapGridToMatCell(const RobotGrid &grid,
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

    inline void SlamLayer::setMapGridFromMatCell(const RobotGrid &grid,
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

    using SlamLayerPtr = std::shared_ptr<SlamLayer>;
}

#endif // MAP_SLAM_LAYER_H