#ifndef UTILS_AREA_H
#define UTILS_AREA_H

#include "contour.h"

#include <unordered_set>

namespace planning_utils
{
    using AreaGrids = std::unordered_set<Grid, GridHash>;

    struct Area
    {
        Area(const AreaContour &contour);
        Area(std::vector<Grid> &outer_contour_grids);

        Grid getCenter();
        Grid getCenter() const;
        std::pair<cv::Mat, GridRect> getFilledMat(int padding = 0);
        std::vector<Area> erode(int dist);
        Area dilate(int dist);

        AreaContour contour;
        AreaGrids grids;
        GridRect rect;
    };
}

#endif // UTILS_AREA_H