#include "point.h"

namespace planning_utils
{
    Point::Point() 
        : x(0.0), y(0.0)
    {}

    Point::Point(float _x, float _y)
        : x(_x), y(_y)
    {}

    Point::Point(const Point &p)
    {
        x = p.x;
        y = p.y;
    }

    Point::Point(const Grid &grid)
        : x(static_cast<float>(grid.x)), y(static_cast<float>(grid.y))
    {}

    // Point::Point(const Pose &pose)
    //     : x(pose.pt.x), y(pose.pt.y)
    // {}

    std::ostream &operator<<(std::ostream &out, const Point &pt)
    {
        out << "{ " << pt.x << ", " << pt.y << " }";
        return out;
    }

    bool operator==(const Point &p1, const Point &p2)
    {
        return (fabs(p1.x - p2.x) < FLOAT_EPS)
            && (fabs(p1.y - p2.y) < FLOAT_EPS);
    }

    bool operator!=(const Point &p1, const Point &p2)
    {
        return (fabs(p1.x - p2.x) >= FLOAT_EPS)
            || (fabs(p1.y - p2.y) >= FLOAT_EPS);
    }

    Point operator-(const Point &p1, const Point &p2)
    {
        return Point(p1.x - p2.x, p1.y - p2.y);
    }

    Point operator+(const Point &p1, const Point &p2)
    {
        return Point(p1.x + p2.x, p1.y + p2.y);
    }

    Point operator*(const Point &p1, float factor)
    {
        return Point(p1.x * factor, p1.y * factor);
    }

    Point operator/(const Point &p1, float factor)
    {
        return Point(p1.x / factor, p1.y / factor);
    }

    bool operator<(const Point &p1, const Point &p2)
    {
        if (p1.x < p2.x)
            return true;
        
        return (p1.y < p2.y);
    }
}