#include "trajectory.h"
#include "geometry_func.h"

namespace planning_utils
{
    template <typename T>
    Trajectory<T>::Trajectory()
    {}

    template <typename T>
    Trajectory<T>::Trajectory(const Trajectory<T> &traj)
        : m_pts(traj.m_pts)
    {}

    template <typename T>
    bool Trajectory<T>::empty() const
    {
        return m_pts.empty();
    }

    template <typename T>
    size_t Trajectory<T>::size() const
    {
        return m_pts.size();
    }

    template <typename T>
    void Trajectory<T>::add(const T &pt, uint64_t ts)
    {
        m_pts.emplace_back(pt, ts);
    }

    template <typename T>
    void Trajectory<T>::erase(size_t idx)
    {
        if (idx >= m_pts.size())
            return;

        m_pts.erase(m_pts.begin() + idx);
    }

    template <typename T>
    void Trajectory<T>::erase(size_t start_idx, size_t end_idx)
    {
        if (end_idx > start_idx)
            return;

        size_t size = m_pts.size();
        if (start_idx >= size)
            return;
        if (end_idx >= size)
            end_idx = size - 1;

        m_pts.erase(m_pts.begin() + start_idx, m_pts.begin() + (end_idx + 1));
    }

    template <typename T>
    void Trajectory<T>::clear()
    {
        m_pts.clear();
    }

    template <typename T>
    const std::pair<T, uint64_t>& Trajectory<T>::front() const
    {
        return m_pts.front();
    }

    template <typename T>
    const std::pair<T, uint64_t>& Trajectory<T>::back() const
    {
        return m_pts.back();
    }

    template <typename T>
    const std::pair<T, uint64_t>& Trajectory<T>::at(size_t n) const
    {
        return m_pts.at(n);
    }

    template <typename T>
    std::pair<T, uint64_t>& Trajectory<T>::operator[](size_t n)
    {
        return m_pts[n];
    }

    template <typename T>
    const std::pair<T, uint64_t>& Trajectory<T>::operator[](size_t n) const
    {
        return m_pts[n];
    }

    template <typename T>
    std::tuple<size_t, std::pair<T, uint64_t>, float>
        Trajectory<T>::getNearestPt(const T &src_pt) const
    {
        if (m_pts.empty())
            return {0, {T{}, 0}, -1.0f};

        size_t min_idx = 0;
        std::pair<T, uint64_t> min_pt = m_pts.front();
        float min_dist
            = hypot(src_pt.x - min_pt.first.x, src_pt.y - min_pt.first.y);

        size_t size = m_pts.size();
        for (size_t i = 1; i < size; ++i)
        {
            std::pair<T, uint64_t> pt = m_pts[i];
            float dist = hypot(src_pt.x - pt.first.x, src_pt.y - pt.first.y);
            if (dist <= min_dist)
            {
                min_idx = i;
                min_pt = pt;
                min_dist = dist;
            }
        }

        return {min_idx, min_pt, min_dist};
    }

    template <typename T>
    int32_t Trajectory<T>::GetPtIndex(const T &pt) const
    {
        size_t size = m_pts.size();
        for (size_t i = 0; i < size; ++i)
        {
            if (m_pts[i].first == pt)
                return i;
        }

        return -1;
    }

    template <typename T>
    int32_t Trajectory<T>::GetPtIndex(uint64_t ts) const
    {
        size_t size = m_pts.size();
        for (size_t i = 0; i < size; ++i)
        {
            if (m_pts[i].second == ts)
                return i;
        }

        return -1;
    }

    template <typename T>
    std::pair<int32_t, std::pair<T, uint64_t>>
        Trajectory<T>::prev(size_t cur_idx, size_t n) const
    {
        size_t size = m_pts.size();
        if (size == 0 || cur_idx >= size || n >= size)
        {
            return {-1, {T{}, 0}};
        }
        
        size_t idx;
        if (cur_idx < n)
            return {-1, {T{}, 0}};
        else
            idx = cur_idx - n;

        return {idx, m_pts[idx]};
    }

    template <typename T>
    std::pair<int32_t, std::pair<T, uint64_t>>
        Trajectory<T>::next(size_t cur_idx, size_t n) const
    {
        size_t size = m_pts.size();
        if (size == 0 || cur_idx >= size || n >= size)
        {
            return {-1, {T{}, 0}};
        }
        
        size_t idx;
        if (cur_idx + n >= size)
            return {-1, {T{}, 0}};
        else
            idx = cur_idx + n;

        return {idx, m_pts[idx]};
    }
    
    template <typename T>
    bool Trajectory<T>::getNormalAngle(size_t target_idx, float& angle_r) const
    {
        std::list<T> neighbour_prev_pts;
        std::list<T> neighbour_next_pts;
        auto && [idx0, pt0] = prev(target_idx, 3);
        auto && [idx6, pt6] = next(target_idx, 3);
        if (idx0 >= 0 && idx6 >= 0)
        {
            neighbour_prev_pts.push_back(pt0.first);
            neighbour_next_pts.push_front(pt6.first);
        }
        auto && [idx1, pt1] = prev(target_idx, 2);
        auto && [idx5, pt5] = next(target_idx, 2);
        if (idx1 >= 0 && idx5 >= 0)
        {
            neighbour_prev_pts.push_back(pt1.first);
            neighbour_next_pts.push_front(pt5.first);
        }
        auto && [idx2, pt2] = prev(target_idx, 1);
        auto && [idx4, pt4] = next(target_idx, 1);
        if (idx2 >= 0 && idx4 >= 0)
        {
            neighbour_prev_pts.push_back(pt2.first);
            neighbour_next_pts.push_front(pt4.first);
        }
        auto && [idx3, pt3] = next(target_idx, 0);
        if (idx3 < 0)
            return false;
        neighbour_next_pts.push_front(pt3.first);

        std::vector<T> n_pts;
        n_pts.insert(n_pts.end(), neighbour_prev_pts.begin(), neighbour_prev_pts.end());
        n_pts.insert(n_pts.end(), neighbour_next_pts.begin(), neighbour_next_pts.end());

        if (n_pts.size() < 3)
            return false;

        std::pair<float, float> vec = std::make_pair(0, 0);    
        for (size_t i = 0; i < n_pts.size() - 3; ++i)
        {
            Point offset{
                static_cast<float>(n_pts[i + 2].x - n_pts[i].x),
                static_cast<float>(n_pts[i + 2].y - n_pts[i].y)
            };
            float length = hypot(offset.x, offset.y);
            if (length < 0.001f)
                return false;
            offset.x /= length;
            offset.y /= length;

            vec.first += offset.x;
            vec.second += offset.y;
        }

        angle_r = getAngleR(Point{0.0f, 0.0f}, Point(vec.first, vec.second));
        angle_r = toStdAngleRangeR(angle_r - 0.5f * M_PI);
        return true;
    }
    
    template <typename T>
    bool Trajectory<T>::getTangentialAngle(size_t target_idx, float& angle_r) const
    {
        float normal_angle;
        if (!getNormalAngle(target_idx, normal_angle))
            return false;
        
        angle_r = toStdAngleRangeR(normal_angle + 0.5 * M_PI);

        return true;
    }

    template <typename T>
    bool Trajectory<T>::getCurvatureHelper(size_t target_idx, size_t step, float &curvature) const
    {
        auto && [prev_idx, prev_pt] = prev(target_idx, step);
        if (prev_idx < 0)
            return false;
        auto && [next_idx, next_pt] = next(target_idx, step);
        if (next_idx < 0)
            return false;

        auto & [cur_pt, cur_ts] = m_pts[target_idx];

        int dx = next_pt.first.x - cur_pt.x;

        if (dx == 0)
        {
            curvature = 0;
            return true;
        }
        
        float first_derivative = (next_pt.first.y - cur_pt.y) / dx;
        float second_derivative = (next_pt.first.y + next_pt.first.y - 2 * cur_pt.y) / (dx * dx);

        curvature = fabs(second_derivative) / powf((1 + first_derivative * first_derivative), 1.5f);

        return true;
    }

    template <typename T>
    bool Trajectory<T>::getCurvature(size_t target_idx, float& curvature) const
    {
        int32_t cnt = 0;
        float c1 = 0.0f;
        float c2 = 0.0f;
        float c3 = 0.0f;
        if (getCurvatureHelper(target_idx, 3, c3))
            cnt++;
        if (getCurvatureHelper(target_idx, 2, c2))
            cnt++;
        if (getCurvatureHelper(target_idx, 1, c1))
            cnt++;

        if (cnt == 0)
            return false;

        curvature = (c1 + c2 + c3) / cnt;
        return true;
    }

    template class Trajectory<Grid>;
    template class Trajectory<Point>;

    template <>
    void Trajectory<Pose>::add(const Pose &pose, uint64_t ts)
    {
        m_pts.emplace_back(pose, ts);
    }

    template <>
    std::tuple<size_t, std::pair<Pose, uint64_t>, float>
        Trajectory<Pose>::getNearestPt(const Pose &src_pt) const
    {
        if (m_pts.empty())
            return {0, {Pose{}, 0}, -1.0f};

        size_t min_idx = 0;
        std::pair<Pose, uint64_t> min_pt = m_pts.front();
        float min_dist
            = hypot(src_pt.pt.x - min_pt.first.pt.x, src_pt.pt.y - min_pt.first.pt.y);

        size_t size = m_pts.size();
        for (size_t i = 1; i < size; ++i)
        {
            std::pair<Pose, uint64_t> pt = m_pts[i];
            float dist
                = hypot(src_pt.pt.x - pt.first.pt.x, src_pt.pt.y - pt.first.pt.y);
            if (min_dist <= min_dist)
            {
                min_idx = i;
                min_pt = pt;
                min_dist = dist;
            }
        }

        return {min_idx, min_pt, min_dist};
    }

    template <>
    bool Trajectory<Pose>::getNormalAngle(size_t target_idx, float& angle_r) const
    {
        std::list<Pose> neighbour_prev_pts;
        std::list<Pose> neighbour_next_pts;
        auto && [idx0, pt0] = prev(target_idx, 3);
        auto && [idx6, pt6] = next(target_idx, 3);
        if (idx0 >= 0 && idx6 >= 0)
        {
            neighbour_prev_pts.push_back(pt0.first);
            neighbour_next_pts.push_front(pt6.first);
        }
        auto && [idx1, pt1] = prev(target_idx, 2);
        auto && [idx5, pt5] = next(target_idx, 2);
        if (idx1 >= 0 && idx5 >= 0)
        {
            neighbour_prev_pts.push_back(pt1.first);
            neighbour_next_pts.push_front(pt5.first);
        }
        auto && [idx2, pt2] = prev(target_idx, 1);
        auto && [idx4, pt4] = next(target_idx, 1);
        if (idx2 >= 0 && idx4 >= 0)
        {
            neighbour_prev_pts.push_back(pt2.first);
            neighbour_next_pts.push_front(pt4.first);
        }
        auto && [idx3, pt3] = next(target_idx, 0);
        if (idx3 < 0)
            return false;
        neighbour_next_pts.push_front(pt3.first);

        std::vector<Pose> n_pts;
        n_pts.insert(n_pts.end(), neighbour_prev_pts.begin(), neighbour_prev_pts.end());
        n_pts.insert(n_pts.end(), neighbour_next_pts.begin(), neighbour_next_pts.end());

        if (n_pts.size() < 3)
            return false;

        std::pair<float, float> vec = std::make_pair(0, 0);    
        for (size_t i = 0; i < n_pts.size() - 2; ++i)
        {
            Point offset{
                static_cast<float>(n_pts[i + 2].pt.x - n_pts[i].pt.x),
                static_cast<float>(n_pts[i + 2].pt.y - n_pts[i].pt.y)
            };
            float length = hypot(offset.x, offset.y);
            if (length < 0.001f)
                return false;
            offset.x /= length;
            offset.y /= length;

            vec.first += offset.x;
            vec.second += offset.y;
        }

        angle_r = getAngleR(Point{0.0f, 0.0f}, Point(vec.first, vec.second));
        angle_r = toStdAngleRangeR(angle_r - 0.5f * M_PI);
        return true;
    }
    
    template <>
    bool Trajectory<Pose>::getCurvatureHelper(size_t target_idx, size_t step, float &curvature) const
    {
        auto && [prev_idx, prev_pt] = prev(target_idx, step);
        if (prev_idx < 0)
            return false;
        auto && [next_idx, next_pt] = next(target_idx, step);
        if (next_idx < 0)
            return false;

        auto & [cur_pt, cur_ts] = m_pts[target_idx];

        int dx = next_pt.first.pt.x - cur_pt.pt.x;

        if (dx == 0)
        {
            curvature = 0;
            return true;
        }
        
        float first_derivative = (next_pt.first.pt.y - cur_pt.pt.y) / dx;
        float second_derivative = (next_pt.first.pt.y + next_pt.first.pt.y - 2 * cur_pt.pt.y) / (dx * dx);

        curvature = fabs(second_derivative) / powf((1 + first_derivative * first_derivative), 1.5f);

        return true;
    }

    template class Trajectory<Pose>;
}