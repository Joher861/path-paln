#include "pose.h"

namespace planning_utils
{
    Pose::Pose()
        : pt(), theta(0.0f)
    {}

    Pose::Pose(const Point &_pt)
        : pt(_pt), theta(0.0f)
    {}
    
    Pose::Pose(const Point &_pt, float _theta)
        : pt(_pt), theta(_theta) 
    {}

    Pose::Pose(const Grid &grid)
        : pt(grid), theta(0.0f)
    {}

    Pose::Pose(const Grid &grid, float _theta)
        : pt(grid), theta(_theta)
    {}

    Pose::Pose(float x_, float y_)
        : pt(x_, y_), theta(0.0f)
    {}

    Pose::Pose(float x_, float y_, float _theta)
        : pt(x_, y_), theta(_theta)
    {}

    Pose::Pose(const Pose &p)
        : pt(p.pt), theta(p.theta)
    {}

    std::ostream &operator<<(std::ostream &out, const Pose &pose)
    {
        out << "{ " << pose.pt.x << ", " << pose.pt.y
            << ", " << pose.theta << " }";
        return out;
    }

    bool operator==(const Pose &p1, const Pose &p2)
    {
        return p1.pt == p2.pt && (fabs(p1.theta - p2.theta) < FLOAT_EPS);
    }
}