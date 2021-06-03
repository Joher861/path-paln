#ifndef UTILS_GEOMETRY_FUNC_H
#define UTILS_GEOMETRY_FUNC_H

#include <cmath>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>

#include <opencv2/core.hpp>

#include "point.h"
#include "pose.h"
#include "grid.h"
#include "line.h"
#include "rect.h"
#include "grid_rect.h"
#include "grid_contour.h"
#include "contour.h"
#include "area.h"

namespace planning_utils
{
    #define M_2PI   (M_PI * 2.0)

    using GridSet = std::unordered_set<Grid, GridHash>;
    using GridMap = std::unordered_map<Grid, GridHash>;
    using AreaPtr = std::shared_ptr<Area>;

    /**
      * @brief  角度转弧度
      * @param  deg     角度
      * @retval float   弧度
      */
    float deg2rad(float deg);

    /**
      * @brief  弧度转角度
      * @param  rad     弧度
      * @retval deg     角度
      */
    float rad2deg(float rad);

    /**
      * @brief  获取两point之间距离
      * @param  p1      第一个point
      *         p2      第二个point
      * @retval float   距离
      */
    float getDistance(const Point &p1, const Point &p2);

    /**
      * @brief  获取两grid之间距离
      * @param  grid1   第一个grid
      *         grid2   第二个grid
      * @retval float   距离
      */
    float getDistance(const Grid &grid1, const Grid &grid2);

    /**
      * @brief  获取两个grid之前代表的实际距离
      * @param  grid1       第一个grid
      *         grid2       第二个grid
      *         resolution  栅格点的分辨率
      * @retval float       距离
      */
    float getDistance(const Grid &grid1, const Grid &grid2, float resolution);
    float getAngleR(const Point &p1, const Point &p2);
    float getAngleR(const Grid &grid1, const Grid &grid2);
    Point getPoint(const Point &pt, float distance, float angle_r);

    float toStdAngleRangeR(float angle_r);
    float toNPPiAngleRangeR(float angle_r);
        
    float normalizeAngle(float angle_r);//normalize angle into the range(-pi ~ +pi)

    bool withinAngleRangeR(float angle_r, float range_min_r, float range_max_r);
    /* angle between angle2 and angle1 (angle2 - angle1) */
    float angleBetweenR(float angle1_r, float angle2_r);

    bool getGridRectIntersection(const GridRect &rect1, const GridRect &rect2,
        GridRect &intersect_rect);

    /* transform represent the transform from the original frame to target form */
    Pose transformFrame(const Pose &transform, const Pose &origin_pose);
    Grid transformFrame(const Pose &transform, const Grid &origin_grid,
        const std::function<Pose(const Grid &)> &grid_to_pose = {},
        const std::function<Grid(const Pose &)> &pose_to_grid = {});
    GridSet preciseTransformGrid(const Pose &transform,
        const Grid &origin_grid, float resolution,
        const std::function<Pose(const Grid &)> &grid_to_pose = {},
        const std::function<Grid(const Pose &)> &pose_to_grid = {});
    Pose reverseTransformFrame(const Pose &transform, const Pose &target_pose);
    Grid reverseTransformFrame(const Pose &transform, const Grid &target_grid,
        const std::function<Pose(const Grid &)> &grid_to_pose = {},
        const std::function<Grid(const Pose &)> &pose_to_grid = {});
    GridSet preciseReverseTransformGrid(const Pose &transform,
        const Grid &target_grid, float resolution,
        const std::function<Pose(const Grid &)> &grid_to_pose = {},
        const std::function<Grid(const Pose &)> &pose_to_grid = {});

    /* get the transformation of the frames (origin to target) */
    Pose getTransform(const Pose &pose_in_origin_frame,
        const Pose &pose_in_target_frame);
    Pose reverseTransform(const Pose &transform);

    std::vector<Point> getLine(const Point &start_pos, const Point &end_pos,
        const float resolution);
    std::vector<Point> getLine(Line &line, const float resolution);
    std::vector<Grid> getLine(const Grid &start_grid, const Grid &end_grid);

    std::vector<Point> getRangeIntersection(const Point &src, const Line &line,
        float range);

    bool isBetweenLine(const Point &pt, const Line &l1, const Line &l2);

    inline Grid matPtToGrid(const cv::Point &pt, const GridRect &rect)
    {
        return rect.max_grid - Grid{pt.y, pt.x};
    }

    inline cv::Point gridToMatPt(const Grid &grid, const GridRect &rect)
    {
        cv::Point pt;
        pt.y = rect.max_grid.x - grid.x;
        pt.x = rect.max_grid.y - grid.y;
        return pt;
    }

    bool generateContourFromMat(cv::Mat &mat,
        std::function<bool(uint8_t)> is_occupied, const GridRect &grid_rect,
        std::vector<AreaContour> &area_contours);
    
    bool fillContour(const Contour &contour, GridSet &inner_grids);
    bool fillAreaContour(const AreaContour &area_contour, GridSet &inner_grids);

    Contour transformContour(const Pose &transform, const Contour &origin_contour,
        const std::function<Pose(const Grid &)> &grid_to_pose = {},
        const std::function<Grid(const Pose &)> &pose_to_grid = {});
    AreaContour transformAreaContour(const Pose &transform,
        const AreaContour &origin_area_contour,
        const std::function<Pose(const Grid &)> &grid_to_pose = {},
        const std::function<Grid(const Pose &)> &pose_to_grid = {});
    Area transformArea(const Pose &transform, const Area &origin_area,
        const std::function<Pose(const Grid &)> &grid_to_pose = {},
        const std::function<Grid(const Pose &)> &pose_to_grid = {});
    AreaPtr transformArea(const Pose &transform, AreaPtr origin_area,
        const std::function<Pose(const Grid &)> &grid_to_pose = {},
        const std::function<Grid(const Pose &)> &pose_to_grid = {});

    Rect GridRectToRect(const GridRect &rect,
        const std::function<Pose(const Grid &)> &grid_to_pose = {});
}

#endif