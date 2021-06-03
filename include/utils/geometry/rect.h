#ifndef UTILS_RECT_H
#define UTILS_RECT_H

#include "point.h"

namespace planning_utils
{
    struct Rect
    {
        Point pt[4];

        Rect();
        Rect(const Point &min_pt, const Point &max_pt);

        /**
          * @brief  构造函数
          * @param  pt1, pt2, pt3, pt4逆时针四个点
          * @retval None
          */
        Rect(const Point &pt1, const Point &pt2, const Point &pt3, const Point &pt4);

        bool inRange(const Point &point) const;
        std::string toString(bool orthogonal = false) const;
        const char* toCString(bool orthogonal = false) const;

        friend bool operator==(const Rect &p1, const Rect &p2);

      private:
        
        float getCross(const Point &p1, const Point &p2, const Point &p) const;
    };

    inline bool Rect::inRange(const Point &point) const
    {
        return (getCross(pt[0], pt[1], point)
                * getCross(pt[2], pt[3], point) >= 0)
            && (getCross(pt[1], pt[2], point)
                * getCross(pt[3], pt[0], point) >= 0);
    }

    inline std::string Rect::toString(bool orthogonal) const
    {
        static std::string str = "";
        if (orthogonal)
        {
            str = "{ "
                + pt[0].toString()
                + ", "
                + pt[2].toString()
                + " }";
        }
        else
        {
            str = "{ "
                + pt[0].toString()
                + ", "
                + pt[1].toString()
                + ", "
                + pt[2].toString()
                + ", "
                + pt[3].toString()
                + " }";
        }

        return str;
    }

    inline const char* Rect::toCString(bool orthogonal) const
    {
        return toString().c_str();
    }
}

#endif // UTILS_RECT_H