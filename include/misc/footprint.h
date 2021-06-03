#ifndef MAP_FOOTPRINT_H
#define MAP_FOOTPRINT_H

#include <vector>
#include <map>

#include <opencv2/opencv.hpp>

#include "misc/planning_typedefs.h"
#include "geometry/geometry_func.h"

using planning_utils::getDistance;

struct Footprint
{
    float resolution;
    float inner_radius;
    float outer_radius;
    float width;
    float length;
    RobotGridRect rect;
    std::set<RobotGrid> grids;
    std::vector<RobotGrid> contour_grids;
    ContourPtr contour;
    std::pair<size_t, RobotGrid> edge_left_most;
    std::pair<size_t, RobotGrid> edge_right_most;

    Footprint() : contour(nullptr)
    {}

    void setFootprint(float res, cv::Mat &desc)
    {
        grids.clear();

        resolution = res;
        
        int32_t row = desc.rows;
        int32_t col = desc.cols;

        RobotGrid src{0, 0};
        RobotGrid src_mat{row / 2, col / 2};

        // 获取机器人坐标系下footprint所占大小
        rect.min_grid = src_mat + RobotGrid{1, 1} - RobotGrid{row, col};
        rect.max_grid = src_mat;

        width = (rect.max_grid.y - rect.min_grid.y) * resolution;
        length = (rect.max_grid.x - rect.min_grid.x) * resolution;

        // 获取内、外接半径
        float max_grid_radius = 0.0f;
        float min_grid_radius = 1000000.0f;

        std::map<RobotGrid, bool> edge_grids;

        for (int32_t x = 0; x < row; ++x)
        {
            uint8_t *ptr = desc.ptr(x);
            for (int32_t y = 0; y < col; ++y)
            {
                uint8_t elem = *(ptr + y);
                
                if (elem == 0)
                    continue;

                RobotGrid cur_mat{x, y};
                RobotGrid fp_grid = src_mat - cur_mat;
                
                bool on_edge = false;
                uint8_t prev_elem, next_elem, up_elem, down_elem;

                if (y == 0 || y == col - 1 || x == 0 || x == row - 1)
                    on_edge = true;

                prev_elem = *(ptr + y - 1);
                next_elem = *(ptr + y + 1);
                up_elem = *(ptr + y - col);
                down_elem = *(ptr + y + col);

                if (prev_elem == 0 || next_elem == 0
                    || up_elem == 0 || down_elem == 0)
                    on_edge = true;

                if (on_edge)
                {
                    edge_grids.emplace(fp_grid, false);
                    float dist = getDistance(src, fp_grid);
                    if (dist > max_grid_radius)
                    {
                        max_grid_radius = dist;
                    }
                    if (dist < min_grid_radius)
                    {
                        min_grid_radius = dist;
                    }
                }

                grids.insert(fp_grid);
            }
        }

        inner_radius = min_grid_radius * resolution;
        outer_radius = max_grid_radius * resolution;

        // 获取边界起始点
        RobotGrid contour_start_grid{-1, -1};
        for (auto & [grid, visited] : edge_grids)
        {
            if (grid.y == 0 && grid.x > 0)
            {
                contour_start_grid = grid;
                break;
            }
        }

        if (contour_start_grid == RobotGrid{-1, -1})
            return;


        contour_grids.push_back(contour_start_grid);
        size_t visited_cnt = 1;
        while (visited_cnt < edge_grids.size())
        {
            auto findNext = [&](const RobotGrid &offset) mutable {
                RobotGrid grid = offset + contour_start_grid;
                auto it = edge_grids.find(grid);
                if (edge_grids.find(grid) == edge_grids.end())
                    return false;
                if (it->second)
                    return false;
                
                it->second = true;
                visited_cnt++;
                contour_grids.push_back(grid);
                contour_start_grid = grid;
                return true;
            };
            
            RobotGrid offset1 = RobotGrid{1, 0};
            if (findNext(offset1))
                continue;
            RobotGrid offset2 = RobotGrid{-1, 0};
            if (findNext(offset2))
                continue;
            RobotGrid offset3 = RobotGrid{0, 1};
            if (findNext(offset3))
                continue;
            RobotGrid offset4 = RobotGrid{0, -1};
            if (findNext(offset4))
                continue;
            RobotGrid offset5 = RobotGrid{1, 1};
            if (findNext(offset5))
                continue;
            RobotGrid offset6 = RobotGrid{-1, 1};
            if (findNext(offset6))
                continue;
            RobotGrid offset7 = RobotGrid{1, -1};
            if (findNext(offset7))
                continue;
            RobotGrid offset8 = RobotGrid{-1, -1};
            if (findNext(offset8))
                continue;
        }

        contour = std::make_shared<Contour>(contour_grids);

        contour_grids.clear();
        size_t contour_size = contour->size();
        for (size_t i = 0; i < contour_size; ++i)
        {
            RobotGrid grid = contour->at(i).grid;
            contour_grids.push_back(grid);
            if (grid.x == 0)
            {
                if (grid.y > 0)
                    edge_left_most = std::make_pair(i, grid);
                if (grid.y < 0)
                    edge_right_most = std::make_pair(i, grid);
            }
        }
    }
};

#endif // MAP_FOOTPRINT_H