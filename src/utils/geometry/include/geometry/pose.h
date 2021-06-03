#ifndef UTILS_POSE_H
#define UTILS_POSE_H

#include "point.h"

namespace planning_utils
{
    struct Grid;
    struct Point;
    struct Pose
    {
        Point pt;
        float theta;

        Pose();
        Pose(const Point &pt);
        Pose(const Point &pt, float theta);
        Pose(const Grid &grid);
        Pose(const Grid &grid, float theta);
        Pose(float x, float y);
        Pose(float x, float y, float theta);
        Pose(const Pose &p);

        std::string toString() const;
        const char* toCString() const;

        friend std::ostream &operator<<(std::ostream &out, const Pose &pose);
        friend bool operator==(const Pose &p1, const Pose &p2);
    };

    inline std::string Pose::toString() const
    {
        static std::string str = "";
        str = "{ "
            + std::to_string(pt.x)
            + ", "
            + std::to_string(pt.y)
            + ", "
            + std::to_string(theta * 180.0f / M_PI)
            + " }";
        return str;
    }

    inline const char* Pose::toCString() const
    {
        return toString().c_str();
    }
}

#endif // UTILS_POSE_H
