#include "path.h"

#include "geometry_func.h"

namespace planning_utils
{
    template <typename T>
    Path<T>::Path(bool ring_path)
        : m_ring_path(ring_path)
    {}

    template <typename T>
    Path<T>::Path(const std::vector<T> &pts, bool ring_path)
        : m_pts(pts), m_ring_path(ring_path)
    {}

    template <typename T>
    Path<T>::Path(const std::vector<T> &&pts, bool ring_path)
        : m_pts(pts), m_ring_path(ring_path)
    {}

    template <typename T>
    Path<T>::Path(const Trajectory<T> &traj)
        : m_ring_path(false)
    {
        size_t traj_size = traj.size();
        for (size_t idx = 0; idx < traj_size; ++idx)
        {
            m_pts.push_back(traj.at(idx).first);
        }
    }

    template <typename T>
    Path<T>::Path(const Path<T> &path)
        : m_pts(path.m_pts), m_ring_path(path.m_ring_path)
    {}

    template <typename T>
    bool Path<T>::empty() const
    {
        return m_pts.empty();
    }

    template <typename T>
    bool Path<T>::is_ring() const
    {
        return m_ring_path;
    }

    template <typename T>
    size_t Path<T>::size() const
    {
        return m_pts.size();
    }

    template <typename T>
    void Path<T>::add(const T &pt)
    {
        m_pts.push_back(pt);
    }

    template <typename T>
    void Path<T>::erase(size_t idx)
    {
        if (idx >= m_pts.size())
            return;

        m_pts.erase(m_pts.begin() + idx);
    }

    template <typename T>
    void Path<T>::erase(size_t start_idx, size_t end_idx)
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
    void Path<T>::clear()
    {
        m_pts.clear();
    }

    template <typename T>
    const T& Path<T>::front() const
    {
        return m_pts.front();
    }

    template <typename T>
    const T& Path<T>::back() const
    {
        return m_pts.back();
    }

    template <typename T>
    const T& Path<T>::at(size_t n) const
    {
        return m_pts.at(n);
    }

    template <typename T>
    T& Path<T>::operator[](size_t n)
    {
        return m_pts[n];
    }

    template <typename T>
    const T& Path<T>::operator[](size_t n) const
    {
        return m_pts[n];
    }

    template <typename T>
    std::tuple<size_t, T, float> Path<T>::getNearestPt(const T &src_pt)
    {
        if (m_pts.empty())
            return {0, T{}, -1.0f};

        size_t min_idx = 0;
        T &min_pt = m_pts.front();
        float min_dist = hypot(src_pt.x - min_pt.x, src_pt.y - min_pt.y);

        size_t size = m_pts.size();
        for (size_t i = 1; i < size; ++i)
        {
            T &pt = m_pts[i];
            float dist = hypot(src_pt.x - pt.x, src_pt.y - pt.y);
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
    int32_t Path<T>::GetPtIndex(const T &pt)
    {
        size_t size = m_pts.size();
        for (size_t i = 0; i < size; ++i)
        {
            if (m_pts[i] == pt)
                return i;
        }

        return -1;
    }

    template <typename T>
    std::pair<int32_t, T> Path<T>::prev(size_t cur_idx, size_t n)
    {
        size_t size = m_pts.size();
        if (size == 0 || cur_idx >= size || n >= size)
        {
            return {-1, T{}};
        }
        
        size_t idx;
        if (!m_ring_path)
        {
            if (cur_idx < n)
                return {-1, T{}};
            else
                idx = cur_idx - n;
        }
        else
        {
            if (cur_idx >= n)
                idx = cur_idx - n;
            else
                idx = size + cur_idx - n;
        }

        return {idx, m_pts[idx]};
    }

    template <typename T>
    std::pair<int32_t, T> Path<T>::next(size_t cur_idx, size_t n)
    {
        size_t size = m_pts.size();
        if (size == 0 || cur_idx >= size || n >= size)
        {
            return {-1, T{}};
        }
        
        size_t idx;
        if (!m_ring_path)
        {
            if (cur_idx + n >= size)
                return {-1, T{}};
            else
                idx = cur_idx + n;
        }
        else
        {
            if (cur_idx + n < size)
                idx = cur_idx + n;
            else
                idx = cur_idx + n - size;
        }

        return {idx, m_pts[idx]};
    }
    
    template <typename T>
    bool Path<T>::getNormalAngle(size_t target_idx, float& angle_r)
    {
        std::list<T> neighbour_prev_pts;
        std::list<T> neighbour_next_pts;
        auto && [idx0, pt0] = prev(target_idx, 3);
        auto && [idx6, pt6] = next(target_idx, 3);
        if (idx0 >= 0 && idx6 >= 0)
        {
            neighbour_prev_pts.push_back(pt0);
            neighbour_next_pts.push_front(pt6);
        }
        auto && [idx1, pt1] = prev(target_idx, 2);
        auto && [idx5, pt5] = next(target_idx, 2);
        if (idx1 >= 0 && idx5 >= 0)
        {
            neighbour_prev_pts.push_back(pt1);
            neighbour_next_pts.push_front(pt5);
        }
        auto && [idx2, pt2] = prev(target_idx, 1);
        auto && [idx4, pt4] = next(target_idx, 1);
        if (idx2 >= 0 && idx4 >= 0)
        {
            neighbour_prev_pts.push_back(pt2);
            neighbour_next_pts.push_front(pt4);
        }
        auto && [idx3, pt3] = next(target_idx, 0);
        if (idx3 < 0)
            return false;
        neighbour_next_pts.push_front(pt3);

        std::vector<T> n_pts;
        n_pts.insert(n_pts.end(), neighbour_prev_pts.begin(), neighbour_prev_pts.end());
        n_pts.insert(n_pts.end(), neighbour_next_pts.begin(), neighbour_next_pts.end());

        if (n_pts.size() < 3)
            return false;

        std::pair<float, float> vec = std::make_pair(0, 0);    
        for (size_t i = 0; i < n_pts.size() - 2; ++i)
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
    bool Path<T>::getTangentialAngle(size_t target_idx, float& angle_r)
    {
        float normal_angle;
        if (!getNormalAngle(target_idx, normal_angle))
            return false;
        
        angle_r = toStdAngleRangeR(normal_angle + 0.5 * M_PI);

        return true;
    }

    template <typename T>
    bool Path<T>::getCurvatureHelper(size_t target_idx, size_t step, float &curvature)
    {
        auto && [prev_idx, prev_pt] = prev(target_idx, step);
        if (prev_idx < 0)
            return false;
        auto && [next_idx, next_pt] = next(target_idx, step);
        if (next_idx < 0)
            return false;

        T &cur_pt = m_pts[target_idx];

        int dx = next_pt.x - cur_pt.x;

        if (dx == 0)
        {
            curvature = 0;
            return true;
        }
        
        float first_derivative = (next_pt.y - cur_pt.y) / dx;
        float second_derivative = (next_pt.y + next_pt.y - 2 * cur_pt.y) / (dx * dx);

        curvature = fabs(second_derivative) / powf((1 + first_derivative * first_derivative), 1.5f);

        return true;
    }

    template <typename T>
    bool Path<T>::getCurvature(size_t target_idx, float& curvature)
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

    template <typename T>
    float Path<T>::calcPathDistance()
    {
        float accum = 0;
        size_t size = m_pts.size();
        for (int i = size-2; i >= 0; i--)
        {
            accum += hypot(m_pts[i+1].x - m_pts[i].x,m_pts[i+1].y - m_pts[i].y); 
        }
        return accum;
    }

    template class Path<Grid>;
    template class Path<Point>;

    template <>
    void Path<Pose>::add(const Pose &pose)
    {
        m_pts.push_back(pose);
    }

    template <>
    std::tuple<size_t, Pose, float> Path<Pose>::getNearestPt(const Pose &src_pt)
    {
        if (m_pts.empty())
            return {0, Pose{}, -1.0f};

        size_t min_idx = 0;
        Pose min_pt = m_pts.front(); //不能加引用
        float min_dist = hypot(src_pt.pt.x - min_pt.pt.x, src_pt.pt.y - min_pt.pt.y);

        size_t size = m_pts.size();
        for (size_t i = 1; i < size; ++i)
        {
            Pose pt = m_pts[i]; //不需要加引用
            float dist = hypot(src_pt.pt.x - pt.pt.x, src_pt.pt.y - pt.pt.y);
            if (dist <= min_dist)
            {
                min_idx = i;
                min_pt = pt;
                min_dist = dist;
            }
        }

        return {min_idx, min_pt, min_dist};
    }

    template <>
    bool Path<Pose>::getNormalAngle(size_t target_idx, float& angle_r)
    {
        std::list<Pose> neighbour_prev_pts;
        std::list<Pose> neighbour_next_pts;
        auto && [idx0, pt0] = prev(target_idx, 3);
        auto && [idx6, pt6] = next(target_idx, 3);
        if (idx0 >= 0 && idx6 >= 0)
        {
            neighbour_prev_pts.push_back(pt0);
            neighbour_next_pts.push_front(pt6);
        }
        auto && [idx1, pt1] = prev(target_idx, 2);
        auto && [idx5, pt5] = next(target_idx, 2);
        if (idx1 >= 0 && idx5 >= 0)
        {
            neighbour_prev_pts.push_back(pt1);
            neighbour_next_pts.push_front(pt5);
        }
        auto && [idx2, pt2] = prev(target_idx, 1);
        auto && [idx4, pt4] = next(target_idx, 1);
        if (idx2 >= 0 && idx4 >= 0)
        {
            neighbour_prev_pts.push_back(pt2);
            neighbour_next_pts.push_front(pt4);
        }
        auto && [idx3, pt3] = next(target_idx, 0);
        if (idx3 < 0)
            return false;
        neighbour_next_pts.push_front(pt3);

        std::vector<Pose> n_pts;
        n_pts.insert(n_pts.end(), neighbour_prev_pts.begin(), neighbour_prev_pts.end());
        n_pts.insert(n_pts.end(), neighbour_next_pts.begin(), neighbour_next_pts.end());

        if (n_pts.size() < 3)
            return false;

        std::pair<float, float> vec = std::make_pair(0, 0);    
        for (size_t i = 0; i < n_pts.size() - 3; ++i)
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
    bool Path<Pose>::getCurvatureHelper(size_t target_idx, size_t step, float &curvature)
    {
        auto && [prev_idx, prev_pt] = prev(target_idx, step);
        if (prev_idx < 0)
            return false;
        auto && [next_idx, next_pt] = next(target_idx, step);
        if (next_idx < 0)
            return false;

        Pose &cur_pt = m_pts[target_idx];

        int dx = next_pt.pt.x - cur_pt.pt.x;

        if (dx == 0)
        {
            curvature = 0;
            return true;
        }
        
        float first_derivative = (next_pt.pt.y - cur_pt.pt.y) / dx;
        float second_derivative = (next_pt.pt.y + next_pt.pt.y - 2 * cur_pt.pt.y) / (dx * dx);

        curvature = fabs(second_derivative) / powf((1 + first_derivative * first_derivative), 1.5f);

        return true;
    }

    template <>
    float Path<Pose>::calcPathDistance()
    {
        float accum = 0;
        size_t size = m_pts.size();
        for (int i = size-2; i >= 0; i--)
        {
            accum += hypot(m_pts[i+1].pt.x - m_pts[i].pt.x,m_pts[i+1].pt.y - m_pts[i].pt.y); 
        }
        return accum;
    }
    
    template class Path<Pose>;
}