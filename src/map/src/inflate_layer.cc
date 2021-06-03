
#include "map/inflate_layer.h"

namespace planning_map
{
    void InflateLayer::setMaxInflateRange(float max_inflate_range)
    {
        m_max_inflate_range = computeCachedDistance(max_inflate_range);
    }

    void InflateLayer::setMaskRange(std::vector<float> mask_range)
    {
        computeMaskRange(mask_range);
    }

    void InflateLayer::inflateMap(const std::vector<RobotGrid> &frees,
        const std::vector<std::pair<RobotGrid, float>> &obs)
    {
        m_inflate_cells.clear();

        LayerMap<InflateCell>::reset();

        MAP_DEBUG_LOG(m_log_name, "obs pts = %lu, free pts = %lu", obs.size(),
            frees.size());

        for (auto & grid : frees)
        {
            InflateCell &cell = m_map[getIndex(grid)];
            cell.mask = MASK_FREE;
            cell.src_grid = grid;
            cell.src_pt = LayerMap<InflateCell>::gridToPose(grid).pt;
            cell.dist_to_src = 0.0f;
        }
        
        for (auto & [grid, inflate_range] : obs)
        {
            Point pt = LayerMap<InflateCell>::gridToPose(grid).pt;
            InflateCell &cell = m_map[getIndex(grid)];
            cell.mask = MASK_SRC;
            cell.src_grid = grid;
            cell.src_pt = pt;
            cell.dist_to_src = 0.0f;
            m_inflate_cells[0.0f].push_back(InflateHelpingCell(grid,
                grid, pt, inflate_range));
        }

        RobotGrid min_grid = m_map_info->grid_range.min_grid;
        RobotGrid max_grid = m_map_info->grid_range.max_grid;

        for (auto & [distance, helping_cells] : m_inflate_cells)
        {
            for (auto &cell : helping_cells)
            {
                RobotGrid grid = cell.grid;
                RobotGrid src_grid = cell.src_grid;
                Point src_pt = cell.src_pt;
                float inflate_range = cell.inflate_range;

                if (grid.x - 1 >= min_grid.x)
                {
                    RobotGrid g = grid + RobotGrid{-1, 0};
                    inflateHelper(g, src_grid, src_pt, inflate_range);
                }
                if (grid.y + 1 <= max_grid.y)
                {
                    RobotGrid g = grid + RobotGrid{0, 1};
                    inflateHelper(g, src_grid, src_pt, inflate_range);
                }
                if (grid.x + 1 <= max_grid.x)
                {
                    RobotGrid g = grid + RobotGrid{1, 0};
                    inflateHelper(g, src_grid, src_pt, inflate_range);
                }
                if (grid.y - 1 >= min_grid.y)
                {
                    RobotGrid g = grid + RobotGrid{0, -1};
                    inflateHelper(g, src_grid, src_pt, inflate_range);
                }
            }
        }

        return;
    }

    void InflateLayer::getMaskMap(uint8_t mask, uint8_t **dest)
    {
        for (size_t i = 0; i < m_total_size; ++i)
        {
            RobotGrid grid = parseIndex(i);
            InflateCell &cell = m_map[i];
            if (isMasked(cell, MASK_SRC))
            {
                dest[grid.x][grid.y] = OBSTACLE;
            }
            else if (isMasked(cell, mask))
            {
                dest[grid.x][grid.y] = INFLATE;
            }
            else if (isMasked(cell, MASK_FREE))
            {
                dest[grid.x][grid.y] = FREE;
            }
            else
            {
                dest[grid.x][grid.y] = UNKNOWN;
            }
        }
    }

    void InflateLayer::getMaskCostMap(uint8_t mask, float inflate_radius,
        uint8_t **dest)
    {
        uint8_t diff = INFLATE - FREE - 1;
        for (size_t i = 0; i < m_total_size; ++i)
        {
            RobotGrid grid = parseIndex(i);
            InflateCell &cell = m_map[i];
            if (isMasked(cell, MASK_SRC))
            {
                dest[grid.x][grid.y] = OBSTACLE;
            }
            else if (isMasked(cell, mask))
            {
                if (cell.dist_to_src <= inflate_radius)
                {
                    dest[grid.x][grid.y] = INFLATE;
                }
                else
                {
                    float factor
                        = exp(-1.0f * (cell.dist_to_src - inflate_radius));
                    dest[grid.x][grid.y]
                        = static_cast<uint8_t>(diff * factor); 
                }
            }
            else if (isMasked(cell, MASK_FREE))
            {
                dest[grid.x][grid.y] = FREE;
            }
        }
    }


    uint8_t InflateLayer::computeCachedDistance(float max_inflate_range)
    {
        float resolution = m_map_info->resolution;
        uint8_t max_range = static_cast<uint8_t>(ceilf(max_inflate_range
            / resolution));
        for (uint8_t i = 0; i < max_range; ++i)
        {
            std::vector<float> cached_distance;
            for (int j = 0; j < max_range; ++j)
            {
                cached_distance.push_back(hypot(i, j) * resolution);
            }
            m_cached_distance.push_back(cached_distance);
        }
        return max_range;
    }

    void InflateLayer::computeMaskRange(std::vector<float> &mask_range)
    {
        m_mask_range.clear();
        if (mask_range.empty())
        {
            return;                
        }

        std::sort(mask_range.begin(), mask_range.end());

        for (size_t i = 0; i < mask_range.size() && i < 6; i++)
        {
            m_mask_range.push_back(mask_range[i]);
        }

        m_mask.push_back(MASK_1);
        m_mask.push_back(MASK_2);
        m_mask.push_back(MASK_3);
        m_mask.push_back(MASK_4);
        m_mask.push_back(MASK_5);
        m_mask.push_back(MASK_6);
    }


    void InflateLayer::inflateHelper(const RobotGrid &grid, const RobotGrid &src_grid,
        const Point &src_pt, float inflate_range)
    {
        RobotGrid delta = grid - src_grid;
        delta.x = abs(delta.x);
        delta.y = abs(delta.y);
        if (delta.x >= m_max_inflate_range
            || delta.y >= m_max_inflate_range)
        {
            return;
        }

        float distance = m_cached_distance[delta.x][delta.y];
        InflateCell &target_inflate_cell = m_map[getIndex(grid)];
        bool inflated = isMasked(target_inflate_cell, ~(MASK_FREE | MASK_SRC));

        if (inflated
            && distance > target_inflate_cell.dist_to_src - FLOAT_EPS)
        {
            return;
        }

        uint8_t mask = MASK_NONE;
        bool stop_inflate = true;
        for (size_t i = 0; i < m_mask_range.size(); i++)
        {
            float range = m_mask_range[i] + inflate_range;
            if (distance <= range)
            {
                stop_inflate = false;
                mask |= m_mask[i];
            }
        }

        if (stop_inflate)
            return;

        setInflateMask(target_inflate_cell, mask);
        target_inflate_cell.src_grid = src_grid;
        target_inflate_cell.src_pt = src_pt;
        target_inflate_cell.dist_to_src = distance;

        m_inflate_cells[distance].push_back(
            InflateHelpingCell{grid, src_grid, src_pt, inflate_range});
    }

}//planning_map