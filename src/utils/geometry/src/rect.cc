
#include "rect.h"

namespace planning_utils
{
    Rect::Rect()
    {}

    Rect::Rect(const Point &min_pt, const Point &max_pt)
    {
        pt[0] = min_pt;
        pt[1] = Point{max_pt.x, min_pt.y};
        pt[2] = max_pt;
        pt[3] = Point{min_pt.x, max_pt.y};
    }

    Rect::Rect(const Point &pt1, const Point &pt2,
        const Point &pt3, const Point &pt4)
    {
        pt[0] = pt1;
        pt[1] = pt2;
        pt[2] = pt3;
        pt[3] = pt4;
    }

    float Rect::getCross(const Point &p1, const Point &p2, const Point &p) const
    {
        return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
    }

    bool operator==(const Rect &p1, const Rect &p2)
    {
        Point match = p1.pt[0];
        size_t match_index = 4;
        for (size_t i = 0; i < 4; i++)
        {
            if (p2.pt[i] == match)
            {
                match_index = i;
                break;
            }
        }

        if (match_index == 4)
            return false;
        
        for (size_t i = match_index + 1; i < 4; i++)
        {
            if ((p1.pt[i - match_index] != p2.pt[i]))
                return false;
        }

        for (size_t i = 0; i < match_index; i++)
        {
            if ((p1.pt[4 - match_index + i] != p2.pt[i]))
                return false;
        }

        return true;
    }
}