
#include "local_map/local_base_layer.h"
#include <vector>


namespace planning_map
{


std::vector<std::pair<RobotGrid, float>> LocalBaseLayer::getInflateObstacles(
    const RobotGridRect &range, float inflate_radius)
{
    std::vector<std::pair<RobotGrid, float>> inflate_obs;

    RobotGrid min_grid = range.min_grid;
    RobotGrid max_grid = range.max_grid;

    for (int32_t x = min_grid.x; x <= max_grid.x; x++)
    {
        for (int32_t y = min_grid.y; y <= max_grid.y; ++y)
        {
            if (m_map[getIndex(x, y)].status == OCCUPIED)
            {
                inflate_obs.emplace_back(RobotGrid{x, y}, inflate_radius);
            }
        }
    }

    return inflate_obs;
}




} //planning_map