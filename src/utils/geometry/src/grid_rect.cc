#include "grid_rect.h"

namespace planning_utils
{
    GridRect::GridRect()
        : min_grid({0, 0}), max_grid({0, 0})
    {}

    GridRect::GridRect(const Grid &_min_grid, const Grid &_max_grid)
        : min_grid(_min_grid), max_grid(_max_grid)
    {}

    bool operator==(const GridRect &r1, const GridRect &r2)
    {
        return (r1.min_grid == r2.min_grid && r1.max_grid == r2.max_grid);
    }
}