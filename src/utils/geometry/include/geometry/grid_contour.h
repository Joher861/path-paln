#ifndef UTILS_GRID_CONTOUR_H
#define UTILS_GRID_CONTOUR_H

#include <vector>
#include <algorithm>

#include "grid.h"

namespace planning_utils
{
    struct GridContour
    {
        enum ClockwiseType
        {
            CW = 0,
            CCW
        };

        GridContour()
            : cw_type(CW)
        {}

        // GridContour(ClockwiseType _cw_type, std::vector<Grid> _grids)
        // {
        //     std::sort(_grids.begin(), grids.end(),
        //         [](const Grid &g1, const Grid &g2)
        //         {
        //             return g1.x < g2.x;
        //         });
            

        // }
        GridContour(ClockwiseType _cw_type, const std::vector<Grid> &_grids)
            : cw_type(_cw_type), grids(_grids)
        {}


        // bool addInnerContour(const GridContour &inner_contour)
        // {
        //     inner_contours.push_back(inner_contour);
        // }

        // bool addInnerContour(ClockwiseType _cw_type,
        //     const std::vector<Grid> &_grids)
        // {
        //     inner_contours.emplace_back(GridContour{_cw_type, _grids});
        // }

        friend bool operator==(const GridContour &c1, const GridContour &c2)
        {
            if (c1.cw_type != c2.cw_type)
                return false;
            
            size_t c1_grids_size = c1.grids.size();
            size_t c2_grids_size = c2.grids.size();
            if (c1_grids_size != c2_grids_size)
                return false;

            if (c1_grids_size == 0)
                return true;
            if (c1_grids_size == 1 && c1.grids.front() == c2.grids.front())
                return true;

            const Grid &c1_front = c1.grids.front();
            bool find_same = false;
            size_t c1_start = 1;
            size_t c2_start;
            for (size_t i = 0; i < c2_grids_size; i++)
            {
                if (c1_front  == c2.grids[i])
                {
                    find_same = true;
                    if (i != c2_grids_size - 1)
                        c2_start = i + 1;
                    else
                        c2_start = 0;
                    break;
                }
            }
            if (!find_same)
                return false;

            for (size_t i = c1_start; i < c1_grids_size; ++i)
            {
                size_t c2_idx = c2_start + i;
                if (c2_idx >= c2_grids_size)
                    c2_idx -= c2_grids_size;
                if (c1.grids[i] != c2.grids[c2_idx])
                    return false;
            }

            return true;
        }

        ClockwiseType cw_type;
        std::vector<Grid> grids;
        // std::vector<GridContour> inner_contours;
    };
}

#endif // UTILS_GRID_CONTOUR_H