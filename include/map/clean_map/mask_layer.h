#ifndef MAP_MASK_LAYER_H
#define MAP_MASK_LAYER_H

#include "map/layer_map.h"

namespace planning_map
{
    class MaskLayer : public LayerMap<uint8_t>
    {
      public:

        enum MaskType
        {
            MASK_NONE               = 0,
            MASK_CLEAN              = 1,
            MASK_STATIC_FORBIDDEN   = 2,
            MASK_DYNAMIC_FORBIDDEN  = 4
        };

        using LayerMap<uint8_t>::LayerMap;

        void set(int32_t x, int32_t y, MaskType type);
        void set(const RobotGrid &grid, MaskType type);
        void unset(int32_t x, int32_t y, MaskType type);
        void unset(const RobotGrid &grid, MaskType type);
        void set(int32_t x, int32_t y, const uint8_t &cell);
        void set(const RobotGrid &grid, const uint8_t &cell);
        bool isMasked(int32_t x, int32_t y, MaskType type);
        bool isMasked(const RobotGrid &grid, MaskType type);
        bool isAllMasked(const RobotGrid &grid, MaskType type);
        bool isAllMasked(int32_t x, int32_t y, MaskType type);
      
      private:

        virtual bool isDefaultValueZero() const;
        virtual bool isOccupied(const RobotGrid &grid) const;
        virtual void setMapGridToMatCell(const RobotGrid &grid,
            uint8_t *mat_cell, uint8_t channel) const;
        virtual void setMapGridFromMatCell(const RobotGrid &grid,
            const uint8_t *mat_cell, uint8_t channel) const;
    };

    inline void MaskLayer::set(int32_t x, int32_t y, MaskType type)
    {
        m_map[getIndex(x, y)] |= type;
    }

    inline void MaskLayer::set(const RobotGrid &grid, MaskType type)
    {
        m_map[getIndex(grid)] |= type;
    }

    inline void MaskLayer::unset(int32_t x, int32_t y, MaskType type)
    {
        m_map[getIndex(x, y)] &= ~type;
    }
    
    inline void MaskLayer::unset(const RobotGrid &grid, MaskType type)
    {
        m_map[getIndex(grid)] &= ~type;
    }

    inline bool MaskLayer::isMasked(int32_t x, int32_t y, MaskType type)
    {
        return m_map[getIndex(x, y)] & type;
    }

    inline bool MaskLayer::isMasked(const RobotGrid &grid, MaskType type)
    {
        return m_map[getIndex(grid)] & type;
    }

    inline bool MaskLayer::isAllMasked(int32_t x, int32_t y, MaskType type)
    {
        return (m_map[getIndex(x, y)] & type) == type;
    }

    inline bool MaskLayer::isAllMasked(const RobotGrid &grid, MaskType type)
    {
        return (m_map[getIndex(grid)] & type) == type;
    }

    inline bool MaskLayer::isDefaultValueZero() const
    {
        return m_default_value == 0;
    }

    inline bool MaskLayer::isOccupied(const RobotGrid &grid) const
    {
        return m_map[getIndex(grid)] != MASK_NONE;
    }

    inline void MaskLayer::setMapGridToMatCell(const RobotGrid &grid,
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

    inline void MaskLayer::setMapGridFromMatCell(const RobotGrid &grid,
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

    using MaskLayerPtr = std::shared_ptr<MaskLayer>; 
}

#endif // MAP_MASK_LAYER_H