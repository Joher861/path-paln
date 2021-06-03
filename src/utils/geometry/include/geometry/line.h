#ifndef UTILS_LINE_H
#define UTILS_LINE_H

#include "pose.h"

namespace planning_utils
{
    struct Line
    {
      public:
        Line();
        Line(const Point &start, const Point &end);
        Line(const Pose &pose);
        Line(const Line &line);
        Line(float a, float b, float c, float angle_r);

        float getDistanceToLine(const Point &pt);
        Point getNearestPointOnLine(const Point &pt);
        bool isOnLine(const Point &pt, float error);
        float getTrueDistanceToLine(const Point &pt);
        Point getTrueNearestPointOnLine(const Point &pt);
        bool isTrueOnLine(const Point &pt, float error);
        bool setStart(const Point &start);
        bool setEnd(const Point &end);
        void setNormalAngleR(float angle_r);
        bool hasStart();
        bool hasEnd();
        bool getStart(Point &start);
        bool getEnd(Point &end);

        float getAngleR();
        float getNormalAngleR();

        friend bool operator==(const Line &line1, const Line &line2);

      private:
        float m_a, m_b, m_c;
        float m_angle_r;
        float m_normal_angle_r;
        Point m_start, m_end;
        bool m_has_start, m_has_end;
    };
}

#endif // UTILS_LINE_H
