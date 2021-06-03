#include "line.h"
#include "geometry_func.h"

using namespace planning_utils;

Line::Line()
{}

Line::Line(const Point &start, const Point &end)
{
    if (start == end)
    {
        m_a = 0.0f;
        m_b = 1.0f;
        m_c = -start.y;
        m_start = start;
        m_end = end;
        m_has_start = m_has_end = true;
        m_angle_r = m_normal_angle_r = 0.0f;
        return;
    }

    if (start.x != end.x)
    {
        m_a = (start.y - end.y) / (end.x - start.x);
        m_b = 1.0f;
        m_c = (start.x * end.y - end.x * start.y) / (end.x - start.x);
        m_angle_r = planning_utils::getAngleR(start, end);
        m_normal_angle_r = toStdAngleRangeR(m_angle_r + M_PI / 2.0f);
    }
    else
    {
        m_a = 1.0f;
        m_b = 0.0f;
        m_c = -start.x;
        m_angle_r = planning_utils::getAngleR(start, end);
        m_normal_angle_r = toStdAngleRangeR(m_angle_r + M_PI / 2.0f);
    }

    m_start = start;
    m_end = end;
    m_has_start = m_has_end = true;
}

Line::Line(const Pose &pose)
{
    m_angle_r = pose.theta;
    m_normal_angle_r = toStdAngleRangeR(m_angle_r + M_PI / 2.0f);

    if (m_angle_r == M_PI_2 || m_angle_r == 1.5f * M_PI)
    {
        m_a = 1.0f;
        m_b = 0.0f;
        m_c = -pose.pt.x;
    }
    else
    {
        m_a = -tanf(m_angle_r);
        m_b = 1.0f;
        m_c = -pose.pt.y - m_a * pose.pt.x;
    }

    m_start = pose.pt;
    m_has_start = true;
    m_has_end = false;
}

Line::Line(const Line &line)
{
    m_a = line.m_a;
    m_b = line.m_b;
    m_c = line.m_c;
    m_angle_r = line.m_angle_r;
    m_normal_angle_r = line.m_normal_angle_r;
    m_start = line.m_start;
    m_end = line.m_end;
    m_has_start = line.m_has_start;
    m_has_end = line.m_has_end;
}

Line::Line(float a, float b, float c, float angle_r)
    : m_a(a), m_b(b), m_c(c), m_angle_r(angle_r), m_start(), m_end(),
        m_has_start(false), m_has_end(false)
{}

float Line::getDistanceToLine(const Point &pt)
{
    return fabs((m_a * pt.x + m_b * pt.y + m_c) / sqrt(m_a * m_a + m_b * m_b));
}

Point Line::getNearestPointOnLine(const Point &pt)
{
    float x = (m_b * m_b * pt.x - m_a * m_b * pt.y - m_a * m_c)
            / (m_a * m_a + m_b * m_b);
    float y = (m_a * m_a * pt.y - m_a * m_b * pt.x - m_b * m_c)
            / (m_a * m_a + m_b * m_b);
    return Point(x, y);
}

bool Line::isOnLine(const Point &pt, float error)
{
    return getDistanceToLine(pt) <= error;
}

float Line::getTrueDistanceToLine(const Point &pt)
{
    Point p = pt;
    Point nearest_pt = getTrueNearestPointOnLine(p);
    return getDistance(p, nearest_pt);
}

Point Line::getTrueNearestPointOnLine(const Point &pt)
{
    Point nearest_pt = getNearestPointOnLine(pt);
    if (!m_has_start && !m_has_end)
    {
        return nearest_pt;
    }
    else if (m_has_start && !m_has_end)
    {
        if (nearest_pt == m_start)
            return m_start;
        if (fabs(angleBetweenR(planning_utils::getAngleR(m_start,
            nearest_pt), m_angle_r)) < 0.01f)
            return nearest_pt;
        else
            return m_start;
    }
    else if (m_has_start && m_has_end)
    {
        if (nearest_pt == m_start)
            return m_start;
        if (nearest_pt == m_end)
            return m_end;

        bool start_on
            = fabs(angleBetweenR(planning_utils::getAngleR(m_start,
                nearest_pt), m_angle_r)) < 0.01;
        bool end_on
            = fabs(angleBetweenR(planning_utils::getAngleR(nearest_pt,
                m_end), m_angle_r)) < 0.01;
        if (start_on && end_on)
            return nearest_pt;
        else if (!start_on)
            return m_start;
        else if (!end_on)
            return m_end;
    }

    return Point(0.0f, 0.0f);
}

bool Line::isTrueOnLine(const Point &pt, float error)
{
    float dist = getTrueDistanceToLine(pt);
    return dist <= error;
}


bool Line::setStart(const Point &start)
{
    if (m_a == 0.0f && m_b == 0.0f)
        return false;

    m_start = start;
    m_has_start = true;
    return true;
}

bool Line::setEnd(const Point &end)
{
    if (!m_has_start)
        return false;

    m_end = end;
    m_has_end = true;
    return true;
}

void Line::setNormalAngleR(float angle_r)
{
    m_normal_angle_r = angle_r;
}

bool Line::hasStart()
{
    return m_has_start;
}

bool Line::hasEnd()
{
    return m_has_end;
}

float Line::getAngleR()
{
    return m_angle_r;
}

bool Line::getStart(Point &start)
{
    if (!m_has_start)
        return false;
    
    start = m_start;
    return true;
}

bool Line::getEnd(Point &end)
{
    if (!m_has_end)
        return false;
    
    end = m_end;
    return true;
}

float Line::getNormalAngleR()
{
    return m_normal_angle_r;
}

namespace planning_utils
{
    bool operator==(const Line &line1, const Line &line2)
    {
        bool step1
            = (fabs(line1.m_a - line2.m_a) < FLOAT_EPS
            && fabs(line1.m_b - line2.m_b) < FLOAT_EPS
            && fabs(line1.m_c - line2.m_c) < FLOAT_EPS
            && fabs(line1.m_angle_r - line2.m_angle_r) < FLOAT_EPS
            && fabs(line1.m_normal_angle_r - line2.m_normal_angle_r) < FLOAT_EPS
            && line1.m_has_start == line2.m_has_start
            && line1.m_has_end == line2.m_has_end);
        
        if (!step1)
            return false;
        
        if (line1.m_has_start && !(line1.m_start == line2.m_start))
            return false;
        if (line1.m_has_end && !(line1.m_end == line2.m_end))
            return false;
            
        return true;
    }
}