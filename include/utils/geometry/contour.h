#ifndef UTILS_CONTOUR_H
#define UTILS_CONTOUR_H

#include <vector>
#include <tuple>
#include <list>
#include <functional>

#include <opencv2/core.hpp>

#include "path.h"
#include "grid_rect.h"

namespace planning_utils
{
    struct ContourGrid
    {
        Grid grid;
        float tangential    = 0.0f;
        float normal        = 0.0f;
        float curvature     = 0.0f;

        ContourGrid() {};
        ContourGrid(const Grid &_grid)
            : grid(_grid)
        {}

        friend bool operator==(const ContourGrid &grid1, const ContourGrid &grid2)
        {
            return (grid1.grid.x == grid2.grid.x)
                && (grid1.grid.y == grid2.grid.y)
                && (grid1.tangential == grid2.tangential)
                && (grid1.normal == grid2.normal)
                && (grid1.curvature == grid2.curvature);
        }

        friend bool operator!=(const ContourGrid &grid1, const ContourGrid &grid2)
        {
            return (grid1.grid.x != grid2.grid.x)
                || (grid1.grid.y != grid2.grid.y)
                || (grid1.tangential != grid2.tangential)
                || (grid1.normal != grid2.normal)
                || (grid1.curvature != grid2.curvature);
        }
    };

    class Contour
    {
      public:

        enum ClockwiseType
        {
            CW = 0,
            CCW,
            MAX_CW_TYPE
        };

        enum ContourType
        {
            OUTER_CONTOUR = 0,
            INNER_CONTOUR
        };

        Contour(const std::vector<Grid> &&grids,
            ContourType contour_type = OUTER_CONTOUR);
        Contour(const Path<Grid> &&path,
            ContourType contour_type = OUTER_CONTOUR);
        Contour(const Path<Point> &&path,
            std::function<Grid(const Pose&)> poseToGrid,
            ContourType contour_type = OUTER_CONTOUR);
        Contour(const Path<Pose> &&path,
            std::function<Grid(const Pose&)> poseToGrid,
            ContourType contour_type = OUTER_CONTOUR);
        Contour(const Trajectory<Grid> &&traj,
            ContourType contour_type = OUTER_CONTOUR);
        Contour(const Trajectory<Point> &&traj,
            std::function<Grid(const Pose&)> poseToGrid,
            ContourType contour_type = OUTER_CONTOUR);
        Contour(const Trajectory<Pose> &&traj,
            std::function<Grid(const Pose&)> poseToGrid,
            ContourType contour_type = OUTER_CONTOUR);
        Contour(const Contour &contour);
        Contour(const Contour &&contour);

        ClockwiseType getClockwiseType() const;
        ContourType getContourType() const;
        GridRect getRect() const;

        bool empty() const;
        size_t size() const;
        const ContourGrid& at(size_t n) const;
        ContourGrid& operator[](size_t n);
        const ContourGrid& operator[](size_t n) const;

        std::tuple<size_t, ContourGrid, float> getNearestGrid(const Grid &src_grid);
        int32_t getGridIndex(const Grid &grid);
        std::pair<int32_t, ContourGrid> prev(size_t cur_idx, size_t n);
        std::pair<int32_t, ContourGrid> next(size_t cur_idx, size_t n);

        cv::Mat getMat() const;
        void print() const;
        std::vector<ContourGrid> getPart(size_t start_idx, size_t end_idx,
            ClockwiseType cw_type = MAX_CW_TYPE);
        std::vector<ContourGrid> getPart(const Grid &start_grid, const Grid &end_grid,
            ClockwiseType cw_type = MAX_CW_TYPE);

        friend bool operator==(const Contour &c1, const Contour &c2);

    private:
        
        template <typename Collection, typename T>
        void generateContourGrids(const Collection &grids,
            std::function<Grid(const T&)> get_grid);
        void judgeClockwiseType();
        bool getNormalAngle(size_t idx, float &angle_r);
        bool getTangentialAngle(size_t idx, float &angle_r);
        bool getCurvature(size_t idx, float &curvature);
        bool getCurvatureHelper(size_t idx, size_t step, float &curvature);
        void generateGridInfo();
        void calculateRect();

        std::vector<ContourGrid> m_grids;
        ClockwiseType m_cw_type;
        ContourType m_contour_type;
        GridRect m_rect;
    };

    struct AreaContour
    {
        AreaContour(const std::vector<Grid> &&outer_grids);
        AreaContour(const Contour &&outer_contour);
        AreaContour(const AreaContour &area_contour);
        void addInnerContour(const std::vector<Grid> &&inner_grids);
        void addInnerContour(const Contour &&inner_contour);

        cv::Mat getMat() const;

        Contour outer_contour;
        std::vector<Contour> inner_contours;
    };
}

#endif // UTILS_CONTOUR_H