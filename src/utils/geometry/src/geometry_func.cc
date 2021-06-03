#include "geometry_func.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace planning_utils
{
    float deg2rad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    float rad2deg(float rad)
    {
        return rad * 180.0 / M_PI;
    }

    float getDistance(const Point &p1, const Point &p2)
    {
        return hypot(p1.x - p2.x, p1.y - p2.y);
    }

    float getDistance(const Grid &grid1, const Grid &grid2)
    {
        return hypot(grid1.x - grid2.x, grid1.y - grid2.y);
    }

    float getDistance(const Grid &grid1, const Grid &grid2, float resolution)
    {
        return hypot(grid1.x - grid2.x, grid1.y - grid2.y) * resolution;
    }

    float getAngleR(const Point &p1, const Point &p2)
    {
        return toStdAngleRangeR(atan2f(p2.y - p1.y, p2.x - p1.x));
    }

    float getAngleR(const Grid &grid1, const Grid &grid2)
    {
        return toStdAngleRangeR(atan2f(grid2.y - grid1.y, grid2.x - grid1.x));
    }

    Point getPoint(const Point &pt, float distance, float angle_r)
    {
        return Point(pt.x + distance * cosf(angle_r),
            pt.y + distance * sinf(angle_r));
    }

    float toStdAngleRangeR(float angle_r)
    {
        if (angle_r >= 0.0 && angle_r < M_2PI)
            return angle_r;

        int cnt = (int)(angle_r / M_2PI);

        angle_r -= cnt * M_2PI;
        if (angle_r < 0)
            angle_r += M_2PI;
        return angle_r; 
    }

    float toNPPiAngleRangeR(float angle_r)
    {
        angle_r = toStdAngleRangeR(angle_r);
        if (angle_r >= M_PI)
            angle_r -= M_2PI;
        return angle_r;
    }

    bool withinAngleRangeR(float angle_r, float range_min_r, float range_max_r)
    {
        angle_r = toStdAngleRangeR(angle_r);
        range_min_r = toStdAngleRangeR(range_min_r);
        range_max_r = toStdAngleRangeR(range_max_r);

        if (range_max_r > range_min_r)
        {
            if (angle_r <= range_max_r && angle_r >= range_min_r)
                return true;
            else
                return false;
        }
        else if (range_max_r == range_min_r)
        {
            return true;
        }
        else
        {
            if (angle_r >= range_min_r || angle_r <= range_max_r)
                return true;
            else
                return false;
        }
    }

    /* angle between angle2 and angle1 (angle2 - angle1) */
    float angleBetweenR(float angle1_r, float angle2_r)
    {
        float angle_diff_r = toStdAngleRangeR(angle2_r - angle1_r);
        return angle_diff_r >= M_PI ? angle_diff_r - M_2PI : angle_diff_r;
    }

    //normalize angle into the range(-pi ~ +pi)
    float normalizeAngle(float angle_r)
    {
        if (angle_r >= -M_PI && angle_r < M_PI)
        return angle_r;
    
        float multiplier = std::floor(angle_r / (2*M_PI));
        angle_r = angle_r - multiplier*2*M_PI;
        if (angle_r >= M_PI)
            angle_r -= 2*M_PI;
        if (angle_r < -M_PI)
            angle_r += 2*M_PI;

        return angle_r;
    }


    bool getGridRectIntersection(const GridRect &rect1, const GridRect &rect2,
        GridRect &intersect_rect)
    {
        Grid min_grid{
            std::max(rect1.min_grid.x, rect2.min_grid.x),
            std::max(rect1.min_grid.y, rect2.min_grid.y)
        };
        Grid max_grid{
            std::min(rect1.max_grid.x, rect2.max_grid.x),
            std::min(rect1.max_grid.y, rect2.max_grid.y)
        };

        if (min_grid.x > max_grid.x || min_grid.y > max_grid.y)
            return false;
        
        intersect_rect = GridRect{min_grid, max_grid};
        return true;
    }

    /* transform represent the transform from the original frame to target form */

    Pose transformFrame(const Pose &transform, const Pose &origin_pose)
    {
        Pose result;
        result.pt.x = transform.pt.x + origin_pose.pt.x * cosf(transform.theta)
            - origin_pose.pt.y * sinf(transform.theta);
        result.pt.y = transform.pt.y + origin_pose.pt.x * sinf(transform.theta)
            + origin_pose.pt.y * cosf(transform.theta);
        result.theta = transform.theta + origin_pose.theta;
        return result;
    }

    Grid transformFrame(const Pose &transform, const Grid &origin_grid,
        const std::function<Pose(const Grid &)> &grid_to_pose,
        const std::function<Grid(const Pose &)> &pose_to_grid)
    {
        Point result;
        Point origin_pt;
        if (grid_to_pose)
            origin_pt = grid_to_pose(origin_grid).pt;
        else
            origin_pt = Point{origin_grid};
        
        result.x = transform.pt.x + origin_pt.x * cosf(transform.theta)
            - origin_pt.y * sinf(transform.theta);
        result.y = transform.pt.y + origin_pt.x * sinf(transform.theta)
            + origin_pt.y * cosf(transform.theta);

        if (pose_to_grid)
            return pose_to_grid(Pose{result});
        else
            return Grid{result};
    }
    
    GridSet preciseTransformGrid(const Pose &transform,
        const Grid &origin_grid, float resolution,
        const std::function<Pose(const Grid &)> &grid_to_pose,
        const std::function<Grid(const Pose &)> &pose_to_grid)
    {
        Pose origin_pose;
        if (grid_to_pose)
            origin_pose = grid_to_pose(origin_grid);
        else
            origin_pose = Pose{origin_grid};

        float half_res = 0.5f * resolution;
        Pose p1{origin_pose.pt.x - half_res, origin_pose.pt.y - half_res};
        Pose p2{origin_pose.pt.x - half_res, origin_pose.pt.y + half_res};
        Pose p3{origin_pose.pt.x + half_res, origin_pose.pt.y + half_res};
        Pose p4{origin_pose.pt.x + half_res, origin_pose.pt.y - half_res};

        p1 = transformFrame(transform, p1);
        p2 = transformFrame(transform, p2);
        p3 = transformFrame(transform, p3);
        p4 = transformFrame(transform, p4);

        GridSet grids;
        if (pose_to_grid)
        {
            grids.emplace(pose_to_grid(p1));
            grids.emplace(pose_to_grid(p2));
            grids.emplace(pose_to_grid(p3));
            grids.emplace(pose_to_grid(p4));
        }
        else
        {
            grids.emplace(Grid{p1.pt});
            grids.emplace(Grid{p2.pt});
            grids.emplace(Grid{p3.pt});
            grids.emplace(Grid{p4.pt});
        }

        return grids;
    }

    Pose reverseTransformFrame(const Pose &transform, const Pose &target_pose)
    {
        Pose result;
        float tmp_x = target_pose.pt.x - transform.pt.x;
        float tmp_y = target_pose.pt.y - transform.pt.y;
        result.pt.x = tmp_x * cosf(transform.theta) + tmp_y * sinf(transform.theta);
        result.pt.y = -tmp_x * sinf(transform.theta) + tmp_y * cosf(transform.theta);
        result.theta = toStdAngleRangeR(target_pose.theta - transform.theta);
        return result;
    }

    Grid reverseTransformFrame(const Pose &transform, const Grid &target_grid,
        const std::function<Pose(const Grid &)> &grid_to_pose,
        const std::function<Grid(const Pose &)> &pose_to_grid)
    {
        Point result;
        Point target_pt;
        if (grid_to_pose)
            target_pt = grid_to_pose(target_grid).pt;
        else
            target_pt = Point{target_grid};

        float tmp_x = target_pt.x - transform.pt.x;
        float tmp_y = target_pt.y - transform.pt.y;
        result.x = tmp_x * cosf(transform.theta) + tmp_y * sinf(transform.theta);
        result.y = -tmp_x * sinf(transform.theta) + tmp_y * cosf(transform.theta);
        
        if (pose_to_grid)
            return pose_to_grid(Pose{result});
        else
            return Grid{result};
    }

    GridSet preciseReverseTransformGrid(const Pose &transform,
        const Grid &target_grid, float resolution,
        const std::function<Pose(const Grid &)> &grid_to_pose,
        const std::function<Grid(const Pose &)> &pose_to_grid)
    {
        Pose target_pose;
        if (grid_to_pose)
            target_pose = grid_to_pose(target_grid);
        else
            target_pose = Pose{target_grid};

        float half_res = 0.5f * resolution;
        Pose p1{target_pose.pt.x - half_res, target_pose.pt.y - half_res};
        Pose p2{target_pose.pt.x - half_res, target_pose.pt.y + half_res};
        Pose p3{target_pose.pt.x + half_res, target_pose.pt.y + half_res};
        Pose p4{target_pose.pt.x + half_res, target_pose.pt.y - half_res};

        p1 = transformFrame(transform, p1);
        p2 = transformFrame(transform, p2);
        p3 = transformFrame(transform, p3);
        p4 = transformFrame(transform, p4);

        GridSet grids;
        if (pose_to_grid)
        {
            grids.emplace(pose_to_grid(p1));
            grids.emplace(pose_to_grid(p2));
            grids.emplace(pose_to_grid(p3));
            grids.emplace(pose_to_grid(p4));
        }
        else
        {
            grids.emplace(Grid{p1.pt});
            grids.emplace(Grid{p2.pt});
            grids.emplace(Grid{p3.pt});
            grids.emplace(Grid{p4.pt});
        }

        return grids;
    }

    Pose getTransform(const Pose &pose_in_origin_frame,
        const Pose &pose_in_target_frame)
    {
        Pose origin = pose_in_origin_frame;
        Pose target = pose_in_target_frame;
        Pose transform;
        transform.theta = target.theta - origin.theta;
        transform.pt.x = target.pt.x - (origin.pt.x * cosf(transform.theta)
            - origin.pt.y * sinf(transform.theta));
        transform.pt.y = target.pt.y - (origin.pt.x * sinf(transform.theta)
            + origin.pt.y * cosf(transform.theta));
        return transform;
    }

    Pose reverseTransform(const Pose &transform)
    {
        Pose p1(0.0f, 0.0f, 0.0f);
        Pose p2 = transformFrame(transform, p1);
        Pose reverse_transform = getTransform(p2, p1);
        return reverse_transform;
    }

    std::vector<Point> getLine(const Point &start_pos, const Point &end_pos,
        const float resolution)
    {
        std::vector<Point> line;
        
        int dx = (int)((end_pos.x - start_pos.x) / resolution);
        int dy = (int)((end_pos.y - start_pos.y) / resolution);

        unsigned int abs_dx = std::abs(dx);
        unsigned int abs_dy = std::abs(dy);

        float offset_dx = dx > 0.0 ? resolution : -resolution;
        float offset_dy = dy > 0.0 ? resolution : -resolution;

        float pos_x = start_pos.x;
        float pos_y = start_pos.y;

        if (abs_dx >= abs_dy)
        {
            int error = abs_dx;
            int error_max = 2 * abs_dx;
            int error_step = 2 * abs_dy;

            for (unsigned int i = 0; i < abs_dx; ++i)
            {
                line.push_back(Point(pos_x, pos_y));
                pos_x += offset_dx;
                error += error_step;
                if (error >= error_max)
                {
                    pos_y += offset_dy;
                    error -= error_max;
                }
            }
            line.push_back(end_pos);      
        }
        else
        {
            int error = abs_dy;
            int error_max = 2 * abs_dy;
            int error_step = 2 * abs_dx;

            for (unsigned int i = 0; i < abs_dy; ++i)
            {
                line.push_back(Point(pos_x, pos_y));
                pos_y += offset_dy;
                error += error_step;
                if (error >= error_max)
                {
                    pos_x += offset_dx;
                    error -= error_max;
                }
            }
            line.push_back(end_pos);
        }

        return line;
    }

    std::vector<Point> getLine(Line &line, const float resolution)
    {
        Point start_pt, end_pt;
        line.getStart(start_pt);
        line.getEnd(end_pt);
        return getLine(start_pt, end_pt, resolution);
    }

    std::vector<Grid> getLine(const Grid &start_grid, const Grid &end_grid)
    {
        Point start_pt(start_grid.x, start_grid.y);
        Point end_pt(end_grid.x, end_grid.y);

        std::vector<Point> &&pt_line = getLine(start_pt, end_pt, 1.0f);
        std::vector<Grid> grid_line;
        if (pt_line.empty())
            return grid_line;
        Grid last_grid{pt_line.front()};
        grid_line.emplace_back(last_grid);
        for (auto &pt : pt_line)
        {
            Grid grid{
                static_cast<int32_t>(roundf(pt.x)),
                static_cast<int32_t>(roundf(pt.y))};
            if (grid == last_grid)
                continue;
            grid_line.emplace_back(grid);
            last_grid = grid;
        }

        return grid_line;
    }

    std::vector<Point> getRangeIntersection(const Point &src,
        const Line &line_, float range)
    {
        Line line = line_;
        std::vector<Point> intersections;

        float distance = line.getDistanceToLine(src);
        Point nearest_pt = line.getNearestPointOnLine(src);
        if (range < distance)
            return intersections;
        else if (range == distance)
        {
            if (line.isTrueOnLine(nearest_pt, 0.02f))
            {
                intersections.push_back(nearest_pt);
                return intersections;
            }
            return intersections;
        }

        float offest = sqrtf(range * range - distance *distance);
        float angle_r1 = line.getAngleR();
        float angle_r2 = toStdAngleRangeR(line.getAngleR() + M_PI);

        Point pt1 = getPoint(nearest_pt, offest, angle_r1);
        Point pt2 = getPoint(nearest_pt, offest, angle_r2);

        if (line.isTrueOnLine(pt1, 0.02f))
            intersections.push_back(pt1);
        if (line.isTrueOnLine(pt2, 0.02f))
            intersections.push_back(pt2);
        
        return intersections;
    }

    bool isBetweenLine(const Point &pt, const Line &line1, const Line &line2)
    {
        Line l1 = line1;
        Line l2 = line2;

        float angle1_r = l1.getAngleR();
        float angle2_r = l2.getAngleR();
        float angle_diff_r = fabs(angleBetweenR(angle1_r, angle2_r));
        if (angle_diff_r > 0.01f && angle_diff_r < M_PI - 0.01f)
            return false;

        Point p1 = l1.getNearestPointOnLine(pt);
        Point p2 = l2.getNearestPointOnLine(pt);

        return fabs(angleBetweenR(getAngleR(pt, p1), getAngleR(p2, pt))) < 0.01f;
    }

    bool generateContourFromMat(cv::Mat &mat,
        std::function<bool(uint8_t)> is_occupied, const GridRect &grid_rect,
        std::vector<AreaContour> &area_contours)
    {
        int mat_type = mat.type();
        if (mat_type != CV_8UC1 && mat_type != CV_8UC3)
            return false;

        // 如果时三通道，将输入的mat先转成单通道灰度
        if (mat_type == CV_8UC3)
            cv::cvtColor(mat, mat, CV_BGR2GRAY);
        
        // 根据给定的条件二值化
        int rows = mat.rows;
        int cols = mat.cols;
        uint8_t *row;
        for (int i = 0; i < rows; i++)
        {
            row = mat.ptr<uint8_t>(i);
            for (int j = 0; j < cols; j++)
            {
                uint8_t *ptr = row + j;
                if (is_occupied(*ptr))
                    *ptr = 255;
                else
                    *ptr = 0;
            }
        }

        // 获取轮廓层次信息
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(mat, contours, hierarchy, CV_RETR_CCOMP,
            CV_CHAIN_APPROX_NONE);

        std::vector<Grid> grids;
        size_t contour_size = contours.size();
        for (size_t i = 0; i < contour_size; ++i)
        {
            cv::Vec4i info = hierarchy[i];

            // 找到外层轮廓
            if (info[3] == -1)
            {
                // 获取外层contour
                grids.clear();
                std::vector<cv::Point> &father_contour = contours[i];
                for (auto &pt : father_contour)
                    grids.emplace_back(matPtToGrid(pt, grid_rect));
                if (grids.size() < 3)
                    continue;
                AreaContour area_contour(std::move(grids));

                // 获取内层contour
                int son_idx = info[2];
                while (son_idx != -1)
                {

                    grids.clear();
                    std::vector<cv::Point> &son_contour = contours[son_idx];
                    info = hierarchy[son_idx];
                    son_idx = info[2];
                    for (auto &pt : son_contour)
                        grids.emplace_back(matPtToGrid(pt, grid_rect));
                    if (grids.size() < 3)
                        continue;
                    area_contour.addInnerContour(std::move(grids));
                }

                // 添加该area_contour
                area_contours.emplace_back(std::move(area_contour));
            }
        }

        return !area_contours.empty();
    }

    bool fillContour(const Contour &contour, GridSet &inner_grids)
    {
        inner_grids.clear();

        GridRect rect = contour.getRect();
        Grid size = rect.getSize();
        std::vector<std::vector<cv::Point>> contours;

        std::vector<cv::Point> ctr;
        size_t contour_size = contour.size();
        for (size_t i = 0; i < contour_size; i++)
        {
            ctr.emplace_back(gridToMatPt(contour[i].grid, rect));
        }
        contours.emplace_back(ctr);

        cv::Mat mat(size.x, size.y,CV_8UC1, cv::Scalar(0));
        cv::drawContours(mat, contours, 0, cv::Scalar(255), CV_FILLED);

        int rows = mat.rows;
        int cols = mat.cols;
        uint8_t *row;
        for (int i = 0; i < rows; i++)
        {
            row = mat.ptr<uint8_t>(i);
            for (int j = 0; j < cols; j++)
            {
                uint8_t *ptr = row + j;
                if (*ptr == 255)
                {
                    inner_grids.emplace(rect.max_grid.x - i, rect.max_grid.y - j);
                }
            }
        }

        return true;
    }

    // bool fillContour(const Contour &contour, GridSet &inner_grids)
    // {
    //     inner_grids.clear();

    //     cv::Mat &&mat = contour.getMat();
    //     GridRect rect = contour.getRect();
    //     // Grid size = rect.getSize();
    //     std::vector<std::vector<cv::Point>> contours;
    //     std::vector<cv::Vec4i> hierarchy;

    //     // cv::Mat mat(size.x, size.y,CV_8UC1, cv::Scalar(0));
    //     cv::findContours(mat, contours, hierarchy, CV_RETR_EXTERNAL,
    //         CV_CHAIN_APPROX_NONE);
    //     size_t contour_size = contours.size();
    //     if (contour_size == 0)
    //         return false;

    //     for (size_t i = 0; i < contour_size; ++i)
    //     {
    //         cv::Vec4i &info = hierarchy[i];
    //         if (info[3] == -1)
    //             cv::drawContours(mat, contours, i, cv::Scalar(255), CV_FILLED);
    //     }

    //     int rows = mat.rows;
    //     int cols = mat.cols;
    //     uint8_t *row;
    //     for (int i = 0; i < rows; i++)
    //     {
    //         row = mat.ptr<uint8_t>(i);
    //         for (int j = 0; j < cols; j++)
    //         {
    //             uint8_t *ptr = row + j;
    //             if (*ptr == 255)
    //             {
    //                 inner_grids.emplace(rect.max_grid.x - i, rect.max_grid.y - j);
    //             }
    //         }
    //     }

    //     return true;
    // }

    bool fillAreaContour(const AreaContour &area_contour, GridSet &inner_grids)
    {
        if (!fillContour(area_contour.outer_contour, inner_grids))
            return false;
            
        for (auto &inner_contour : area_contour.inner_contours)
        {
            GridSet hole_grids;
            if (fillContour(inner_contour, hole_grids))
            {
                for (auto &grid : hole_grids)
                    inner_grids.erase(grid);
            }
                // inner_grids.erase(hole_grids.begin(), hole_grids.end());
        }
        return !inner_grids.empty();
    }

    Contour transformContour(const Pose &transform, const Contour &origin_contour,
        const std::function<Pose(const Grid &)> &grid_to_pose,
        const std::function<Grid(const Pose &)> &pose_to_grid)
    {
        std::vector<Grid> target_grids;
        Contour::ContourType type = origin_contour.getContourType();
        size_t size = origin_contour.size();
        for (size_t i = 0; i < size; ++i)
        {
            const auto &grid = origin_contour.at(i).grid;
            target_grids.emplace_back(transformFrame(transform, grid,
                grid_to_pose, pose_to_grid));
        }

        return Contour(std::move(target_grids), type);
    }

    AreaContour transformAreaContour(const Pose &transform,
        const AreaContour &origin_area_contour,
        const std::function<Pose(const Grid &)> &grid_to_pose,
        const std::function<Grid(const Pose &)> &pose_to_grid)
    {
        AreaContour new_area_contour{
            transformContour(transform, origin_area_contour.outer_contour,
                grid_to_pose, pose_to_grid)
        };
        
        const auto &inner_contours = origin_area_contour.inner_contours;
        for (const auto &inner_contour : inner_contours)
        {
            new_area_contour.addInnerContour(
                transformContour(transform, inner_contour,
                    grid_to_pose, pose_to_grid)
            );
        }
        return new_area_contour;
    }

    Area transformArea(const Pose &transform, const Area &origin_area,
        const std::function<Pose(const Grid &)> &grid_to_pose,
        const std::function<Grid(const Pose &)> &pose_to_grid)
    {
        return Area(transformAreaContour(transform, origin_area.contour,
            grid_to_pose, pose_to_grid));
    }

    AreaPtr transformArea(const Pose &transform, AreaPtr origin_area,
        const std::function<Pose(const Grid &)> &grid_to_pose,
        const std::function<Grid(const Pose &)> &pose_to_grid)
    {
        return std::make_shared<Area>(
            transformAreaContour(transform, origin_area->contour,
                grid_to_pose, pose_to_grid));
    }

    Rect GridRectToRect(const GridRect &rect,
        const std::function<Pose(const Grid &)> &grid_to_pose)
    {
        Pose max_pose, min_pose;
        if (grid_to_pose)
        {
            min_pose = grid_to_pose(rect.min_grid);
            max_pose = grid_to_pose(rect.max_grid);
        }
        else
        {
            min_pose = Pose{rect.min_grid};
            max_pose = Pose{rect.max_grid};
        }

        return Rect{min_pose.pt, max_pose.pt};
    }
}