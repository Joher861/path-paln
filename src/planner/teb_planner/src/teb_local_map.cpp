

#include "teb_local_planner/teb_local_map.h"
#include <stdio.h>
using namespace costmap_2d;

TebLocalMap::TebLocalMap(/* args */)
{
    // memset(&m_local_map_data, 0, sizeof(DataLocalMap));
    m_local_map_data.x = 0;
    m_local_map_data.y = 0;
    m_local_map_data.theta = 0;
    m_local_map_data.ts = 0;
    m_local_map_data.obstacleInFront = 0;
    m_local_map_data.distanceToFrontObstacle = 0;

    for (size_t i = 0; i < TEB_LOCAL_MAP_WINDOW_SIZE; i++)
    {
        for (size_t j = 0; j < TEB_LOCAL_MAP_WINDOW_SIZE; j++)
        {

            m_local_map_data.grid[i][j] = 0;
        }
    }
}

TebLocalMap::~TebLocalMap()
{
}

void TebLocalMap::update()
{
    g_dc.getData<DataLocalMap>(m_local_map_data);
}
//local_map_data_.grid[col][row] 中的row对应mx，col对应my
bool TebLocalMap::world2map(float wx, float wy, unsigned int* mx, unsigned int* my) const
{
    RobotPose world_pose(wx,wy,0);
    RobotPose origin_pose(m_local_map_data.x, m_local_map_data.y, m_local_map_data.theta);
    RobotPose diff = reverseTransformFrame(origin_pose,world_pose);
    *my = (unsigned int)(diff.pt.x / m_resolution + m_origin_grid.x) + 1;//用map2world转换，再用world2map转换回来，实测少1,此处补偿
    *mx = (unsigned int)(diff.pt.y / m_resolution + m_origin_grid.y);

    return (*mx < m_size_x && *my < m_size_y);
}

//用法对应关系
//local_map_data_.grid[col][row]   x->row  y->col
//teb_local_map_.map2world(row,col,world_x,world_y);
bool TebLocalMap::map2world(unsigned int mx, unsigned int my, float* wx, float* wy) const
{
    RobotGrid grid(my,mx);
    RobotGrid grid_diff = grid - m_origin_grid;
    RobotPose diff { grid_diff.x * m_resolution, grid_diff.y * m_resolution, 0};//在机器人坐标系下的坐标
    RobotPose origin_pose(m_local_map_data.x, m_local_map_data.y, m_local_map_data.theta);
    RobotPose world_pose = transformFrame(diff, origin_pose);
    *wx = world_pose.pt.x;
    *wy = world_pose.pt.y;
    return false;
}

//local_map_data_.grid[col][row] 中的row填入mx，col填入my
unsigned char TebLocalMap::getCost(unsigned int mx, unsigned int my) const
{
    if(m_local_map_data.grid[my][mx] != FREE_SPACE)
    {
        return LETHAL_OBSTACLE;
    }
    return FREE_SPACE;
}

