#ifndef MAP_OBSTACLE_MARK_LAYER_H
#define MAP_OBSTACLE_MARK_LAYER_H

#include "map/layer_map.h"

namespace planning_map
{
    class ObMarkLayer : public LayerMap<uint8_t>
    {
      public:

        enum ObstacleType
        {
            NONE        = 0,
            BUMP        = 1,
            CLIFF       = 2,
            MAGNET      = 4,
            TRAP        = 8,
            CARPET      = 16,
            THRESHOLD   = 32,
            MAX_OBSTACLE_TYPE   = 33,
        };

        using LayerMap<uint8_t>::LayerMap;

        void set(int32_t x, int32_t y, ObstacleType type);
        void set(const RobotGrid &grid, ObstacleType type);
        void unset(int32_t x, int32_t y, ObstacleType type);
        void unset(const RobotGrid &grid, ObstacleType type);
        void set(int32_t x, int32_t y, const uint8_t &cell);
        void set(const RobotGrid &grid, const uint8_t &cell);
        bool isMarked(int32_t x, int32_t y, ObstacleType type);
        bool isMarked(const RobotGrid &grid, ObstacleType type);
        bool isAllMarked(int32_t x, int32_t y, ObstacleType type);
        bool isAllMarked(const RobotGrid &grid, ObstacleType type);
        ObstacleType getType(uint8_t cell);
        std::pair<cv::Mat, RobotGridRect> saveObMarkMapToMat(RobotGridRect region,
            MapSaveOption opt, uint32_t padding = 5);
        bool loadObMarkMapFromMat(const RobotGridRect &region, const cv::Mat &map);

      private:

        virtual bool isDefaultValueZero() const;
        virtual bool isOccupied(const RobotGrid &grid) const;
        virtual void setMapGridToMatCell(const RobotGrid &grid,
            uint8_t *mat_cell, uint8_t channel) const;
        virtual void setMapGridFromMatCell(const RobotGrid &grid,
            const uint8_t *mat_cell, uint8_t channel) const;
    };

    inline void ObMarkLayer::set(int32_t x, int32_t y, ObstacleType type)
    {
        m_map[getIndex(x, y)] |= type;
    }

    inline void ObMarkLayer::set(const RobotGrid &grid, ObstacleType type)
    {
        m_map[getIndex(grid)] |= type;
    }

    inline void ObMarkLayer::unset(int32_t x, int32_t y, ObstacleType type)
    {
        m_map[getIndex(x, y)] &= ~type;
    }
    
    inline void ObMarkLayer::unset(const RobotGrid &grid, ObstacleType type)
    {
        m_map[getIndex(grid)] &= ~type;
    }

    inline bool ObMarkLayer::isMarked(int32_t x, int32_t y, ObstacleType type)
    {
        return m_map[getIndex(x, y)] & type;
    }

    inline bool ObMarkLayer::isMarked(const RobotGrid &grid, ObstacleType type)
    {
        return m_map[getIndex(grid)] & type;
    }

    inline bool ObMarkLayer::isAllMarked(int32_t x, int32_t y, ObstacleType type)
    {
        return (m_map[getIndex(x, y)] & type) == type;
    }

    inline bool ObMarkLayer::isAllMarked(const RobotGrid &grid, ObstacleType type)
    {
        return (m_map[getIndex(grid)] & type) == type;
    }

    inline ObMarkLayer::ObstacleType ObMarkLayer::getType(uint8_t cell)
    {
        for (uint8_t i = 1; i <= 8; ++i)
        {
            ObstacleType type = static_cast<ObstacleType>(1 << i);
            if (cell & type)
                return type;
        }

        return NONE;
    }

    inline bool ObMarkLayer::isDefaultValueZero() const
    {
        return m_default_value == 0;
    }

    inline bool ObMarkLayer::isOccupied(const RobotGrid &grid) const
    {
        return m_map[getIndex(grid)] != NONE;
    }

    inline void ObMarkLayer::setMapGridToMatCell(const RobotGrid &grid,
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

    inline void ObMarkLayer::setMapGridFromMatCell(const RobotGrid &grid,
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

    std::pair<cv::Mat, RobotGridRect> ObMarkLayer::saveObMarkMapToMat(
        RobotGridRect region, MapSaveOption opt, uint32_t padding)
    {
        if (opt == SAVE_WHOLE)
        {
            region = m_map_info->grid_range;
        }
        else if (opt == SAVE_OCCUPIED)
        {
            RobotGridRect occupied_region = getOccupiedRegion();
            region = getPaddedRegion(occupied_region, padding);
            if (outOfRange(region.min_grid))
                return std::make_pair(cv::Mat{}, RobotGridRect{});
        }
        else if (opt == SAVE_FIXED)
        {
            if (outOfRange(region.min_grid) || outOfRange(region.max_grid))
                return std::make_pair(cv::Mat{}, RobotGridRect{});
        }
        else
        {
            return std::make_pair(cv::Mat{}, RobotGridRect{});
        }

        int32_t rows = region.max_grid.x - region.min_grid.x + 1;
        int32_t cols = region.max_grid.y - region.min_grid.y + 1;
        cv::Mat map(rows, cols, CV_8UC1);

        uint8_t *row;
        for (int32_t i = 0; i < rows; i++)
        {
            row = map.ptr<uint8_t>(i);
            for (int32_t j = 0; j < cols; j++)
            {
                uint8_t *ptr = row + j;
                RobotGrid grid = region.max_grid - RobotGrid{i, j};
                *ptr = getType(m_map[getIndex(grid)]);
            }
        }

        return std::make_pair(map, region);
    }

    bool ObMarkLayer::loadObMarkMapFromMat(const RobotGridRect &region, const cv::Mat &map)
    {
        const uint8_t *row;
        for (int32_t i = 0; i < map.rows; i++)
        {
            row = map.ptr<uint8_t>(i);
            for (int32_t j = 0; j < map.cols; j++)
            {
                const uint8_t *ptr = row + j * 3;
                RobotGrid grid = region.max_grid - RobotGrid{i, j};
                set(grid, static_cast<ObstacleType>(*ptr));
            }
        }
        return true;
    }

    using ObMarkLayerPtr = std::shared_ptr<ObMarkLayer>;
}

#endif // MAP_OBSTACLE_MARK_LAYER_H