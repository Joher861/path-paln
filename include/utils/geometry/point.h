#ifndef UTILS_POINT_H
#define UTILS_POINT_H

#include <ostream>
#include <cmath>
#include <string>

#include "grid.h"
// #include "pose.h"

#include "misc/planning_defs.h"

namespace planning_utils
{
    struct Grid;
    // struct Pose;

    struct Point
    {
        float x;
        float y;

        Point();
        Point(float x, float y);
        Point(const Point &p);
        // Point(const Pose &pose);
        Point(const Grid &grid);

        std::string toString() const;
        const char* toCString() const;
        float norm() const;

        friend std::ostream &operator<<(std::ostream &out, const Point &pt);
        friend bool operator==(const Point &p1, const Point &p2);
        friend bool operator!=(const Point &p1, const Point &p2);
        friend Point operator-(const Point &p1, const Point &p2);
        friend Point operator+(const Point &p1, const Point &p2);
        friend Point operator*(const Point &p1, float factor);
        friend Point operator/(const Point &p1, float factor);
        friend bool operator<(const Point &p1, const Point &p2);
    };

    inline std::string Point::toString() const
    {
        static std::string str = "";
        str = "{ "
            + std::to_string(x)
            + ", "
            + std::to_string(y)
            + " }";
        return str;
    }

    inline const char* Point::toCString() const
    {
        return toString().c_str();
    }

    inline float Point::norm() const
    {
        return std::sqrt(x*x + y*y);
    }
}


#endif // UTILS_POINT_H

