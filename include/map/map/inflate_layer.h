#ifndef MAP_INFLATE_LAYER_H
#define MAP_INFLATE_LAYER_H

#include <vector>
#include <map>
#include <algorithm>
#include <string.h>

#include "layer_map.h"
#include "map_cost_value.h"

namespace planning_map
{
    struct InflateCell
    {
        InflateCell() {}
        InflateCell(uint8_t _mask, const RobotGrid &_src_grid,
                    const Point &_src_pt, float _dist_to_src)
            : mask(_mask),
              src_grid(_src_grid),
              src_pt(_src_pt),
              dist_to_src(_dist_to_src)
        {
        }

        uint8_t mask;
        RobotGrid src_grid;
        Point src_pt;
        float dist_to_src;
    };

    struct InflateHelpingCell
    {
        InflateHelpingCell(RobotGrid _grid, RobotGrid _src_grid, Point _src_pt,
                           float _inflate_range)
            : grid(_grid),
              src_grid(_src_grid),
              src_pt(_src_pt),
              inflate_range(_inflate_range)
        {
        }

        RobotGrid grid;
        RobotGrid src_grid;
        Point src_pt;
        float inflate_range;
    };

    class InflateLayer : public LayerMap<InflateCell>
    {
    public:
        enum InflateMask
        {
            MASK_NONE = 0,
            MASK_FREE = 1,
            MASK_SRC = 2,
            MASK_1 = 4,
            MASK_2 = 8,
            MASK_3 = 16,
            MASK_4 = 32,
            MASK_5 = 64,
            MASK_6 = 128,
        };

        using LayerMap<InflateCell>::LayerMap;

        void setMaxInflateRange(float max_inflate_range);
        void setMaskRange(std::vector<float> mask_range);
        void setBaseMask(int32_t x, int32_t y, uint8_t mask);
        void setBaseMask(const RobotGrid &grid, uint8_t mask);
        void setBaseMask(InflateCell &cell, uint8_t mask);
        void setInflateMask(int32_t x, int32_t y, uint8_t mask);
        void setInflateMask(const RobotGrid &grid, uint8_t mask);
        void setInflateMask(InflateCell &cell, uint8_t mask);
        void unsetBaseMask(int32_t x, int32_t y, uint8_t mask);
        void unsetBaseMask(const RobotGrid &grid, uint8_t mask);
        void unsetBaseMask(InflateCell &cell, uint8_t mask);
        void unsetInflateMask(int32_t x, int32_t y, uint8_t mask);
        void unsetInflateMask(const RobotGrid &grid, uint8_t mask);
        void unsetInflateMask(InflateCell &cell, uint8_t mask);
        bool isMasked(int32_t x, int32_t y, uint8_t mask) const;
        bool isMasked(const RobotGrid &grid, uint8_t mask) const;
        bool isMasked(const InflateCell &cell, uint8_t mask) const;
        bool isAllMasked(int32_t x, int32_t y, uint8_t mask) const;
        bool isAllMasked(const RobotGrid &grid, uint8_t mask) const;
        bool isAllMasked(const InflateCell &cell, uint8_t mask) const;
        bool isMaskedOccupied(int32_t x, int32_t y) const;
        bool isMaskedOccupied(const RobotGrid &grid) const;
        bool isMaskedOccupied(const InflateCell &cell) const;
        void inflateMap(const std::vector<RobotGrid> &frees,
                        const std::vector<std::pair<RobotGrid, float>> &obs);
        virtual void getMaskMap(uint8_t mask, uint8_t **dest);
        virtual void getMaskCostMap(uint8_t mask, float inflate_radius,
                                    uint8_t **dest);
        virtual bool isOccupied(const RobotGrid &grid) const;

    protected:
        virtual bool isDefaultValueZero() const;
        virtual void setMapGridToMatCell(const RobotGrid &grid, uint8_t *mat_cell,
                                         uint8_t channel) const;
        virtual void setMapGridFromMatCell(const RobotGrid &grid,
                                           const uint8_t *mat_cell, uint8_t channel) const;
        uint8_t computeCachedDistance(float max_inflate_range);
        void computeMaskRange(std::vector<float> &mask_range);
        void inflateHelper(const RobotGrid &grid, const RobotGrid &src_grid,
                           const Point &src_pt, float inflate_range);

        uint8_t m_max_inflate_range;
        std::vector<std::vector<float>> m_cached_distance;
        std::map<float, std::vector<InflateHelpingCell>> m_inflate_cells;
        std::vector<float> m_mask_range;
        std::vector<uint8_t> m_mask;
    };

    inline void InflateLayer::setBaseMask(int32_t x, int32_t y, uint8_t mask)
    {
        InflateCell &target_cell = m_map[getIndex(x, y)];
        if (mask == MASK_NONE)
        {
            target_cell.mask &= ~(MASK_FREE | MASK_SRC);
        }
        else if (mask == MASK_FREE)
        {
            target_cell.mask &= ~MASK_SRC;
            target_cell.mask |= MASK_FREE;
        }
        else if (mask == MASK_SRC)
        {
            target_cell.mask &= ~MASK_FREE;
            target_cell.mask |= MASK_SRC;
        }
    }

    inline void InflateLayer::setBaseMask(const RobotGrid &grid, uint8_t mask)
    {
        setBaseMask(grid.x, grid.y, mask);
    }

    inline void InflateLayer::setBaseMask(InflateCell &cell, uint8_t mask)
    {
        if (mask == MASK_NONE)
        {
            cell.mask &= ~(MASK_FREE | MASK_SRC);
        }
        else if (mask == MASK_FREE)
        {
            cell.mask &= ~MASK_SRC;
            cell.mask |= MASK_FREE;
        }
        else if (mask == MASK_SRC)
        {
            cell.mask &= ~MASK_FREE;
            cell.mask |= MASK_SRC;
        }
    }

    inline void InflateLayer::setInflateMask(int32_t x, int32_t y, uint8_t mask)
    {
        InflateCell &target_cell = m_map[getIndex(x, y)];
        target_cell.mask |= mask;
    }

    inline void InflateLayer::setInflateMask(const RobotGrid &grid, uint8_t mask)
    {
        setInflateMask(grid.x, grid.y, mask);
    }

    inline void InflateLayer::setInflateMask(InflateCell &cell, uint8_t mask)
    {
        cell.mask |= mask;
    }

    inline void InflateLayer::unsetBaseMask(int32_t x, int32_t y, uint8_t mask)
    {
        InflateCell &target_cell = m_map[getIndex(x, y)];
        target_cell.mask &= ~(MASK_FREE | MASK_SRC);
    }

    inline void InflateLayer::unsetBaseMask(const RobotGrid &grid, uint8_t mask)
    {
        unsetBaseMask(grid.x, grid.y, mask);
    }

    inline void InflateLayer::unsetBaseMask(InflateCell &cell, uint8_t mask)
    {
        cell.mask &= ~(MASK_FREE | MASK_SRC);
    }

    inline void InflateLayer::unsetInflateMask(int32_t x, int32_t y, uint8_t mask)
    {
        InflateCell &target_cell = m_map[getIndex(x, y)];
        target_cell.mask &= ~mask;
    }

    inline void InflateLayer::unsetInflateMask(const RobotGrid &grid, uint8_t mask)
    {
        unsetInflateMask(grid.x, grid.y, mask);
    }

    inline void InflateLayer::unsetInflateMask(InflateCell &cell, uint8_t mask)
    {
        cell.mask &= ~mask;
    }

    inline bool InflateLayer::isDefaultValueZero() const
    {
        return (m_default_value.mask == MASK_NONE && m_default_value.src_grid == RobotGrid{0, 0} && m_default_value.src_pt == Point{0.0f, 0.0f} && m_default_value.dist_to_src == 0.0f);
    }

    inline bool InflateLayer::isOccupied(const RobotGrid &grid) const
    {
        return (m_map[getIndex(grid)].mask != m_default_value.mask);
    }

    inline void InflateLayer::setMapGridToMatCell(const RobotGrid &grid,
                                                  uint8_t *mat_cell, uint8_t channel) const
    {
        if (channel == CV_8UC1)
        {
            *mat_cell = m_map[getIndex(grid)].mask;
        }
        else if (channel == CV_8UC3)
        {
            *mat_cell = m_map[getIndex(grid)].mask;
            *(mat_cell + 1) = m_map[getIndex(grid)].mask;
            *(mat_cell + 2) = m_map[getIndex(grid)].mask;
        }
    }

    inline void InflateLayer::setMapGridFromMatCell(const RobotGrid &grid,
                                                    const uint8_t *mat_cell, uint8_t channel) const
    {
        if (channel == CV_8UC1)
        {
            m_map[getIndex(grid)].mask = *mat_cell;
        }
        else if (channel == CV_8UC3)
        {
            m_map[getIndex(grid)].mask = *mat_cell;
            m_map[getIndex(grid)].mask = *(mat_cell + 1);
            m_map[getIndex(grid)].mask = *(mat_cell + 2);
        }
    }

    inline bool InflateLayer::isMasked(int32_t x, int32_t y, uint8_t mask) const
    {
        if (mask == MASK_NONE)
            return m_map[getIndex(x, y)].mask == MASK_NONE;
        return m_map[getIndex(x, y)].mask & mask;
    }

    inline bool InflateLayer::isMasked(const RobotGrid &grid, uint8_t mask) const
    {
        return isMasked(grid.x, grid.y, mask);
    }

    inline bool InflateLayer::isMasked(const InflateCell &cell, uint8_t mask) const
    {
        if (mask == MASK_NONE)
            return cell.mask == MASK_NONE;
        return cell.mask & mask;
    }

    inline bool InflateLayer::isAllMasked(int32_t x, int32_t y, uint8_t mask) const
    {
        return (m_map[getIndex(x, y)].mask & mask) == mask;
    }

    inline bool InflateLayer::isAllMasked(const RobotGrid &grid, uint8_t mask) const
    {
        return isAllMasked(grid.x, grid.y, mask);
    }

    inline bool InflateLayer::isAllMasked(const InflateCell &cell, uint8_t mask) const
    {
        return (cell.mask & mask) == mask;
    }

    inline bool InflateLayer::isMaskedOccupied(int32_t x, int32_t y) const
    {
        static uint8_t occupied_mask = MASK_SRC | MASK_1 | MASK_2 | MASK_3 | MASK_4 | MASK_5 | MASK_6;
        return m_map[getIndex(x, y)].mask & occupied_mask;
    }

    inline bool InflateLayer::isMaskedOccupied(const RobotGrid &grid) const
    {
        static uint8_t occupied_mask = MASK_SRC | MASK_1 | MASK_2 | MASK_3 | MASK_4 | MASK_5 | MASK_6;
        return m_map[getIndex(grid)].mask & occupied_mask;
    }

    inline bool InflateLayer::isMaskedOccupied(const InflateCell &cell) const
    {
        static uint8_t occupied_mask = MASK_SRC | MASK_1 | MASK_2 | MASK_3 | MASK_4 | MASK_5 | MASK_6;
        return cell.mask & occupied_mask;
    }

    using InflateLayerPtr = std::shared_ptr<InflateLayer>;
} // namespace planning_map

#endif // MAP_INFLATE_LAYER_H