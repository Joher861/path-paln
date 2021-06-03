#ifndef UTILS_PATH_H
#define UTILS_PATH_H

#include <vector>
#include <list>
#include <tuple>

#include "trajectory.h"

namespace planning_utils
{
    template <typename T>
    class Path
    {
    public:

        Path(bool ring_path = false);
        Path(const std::vector<T> &pts, bool ring_path = false);
        Path(const std::vector<T> &&pts, bool ring_path = false);
        Path(const Trajectory<T> &traj);
        Path(const Path<T> &path);

        bool empty() const;
        bool is_ring() const;
        size_t size() const;
        void add(const T &pt);
        void erase(size_t idx);
        void erase(size_t start_idx, size_t end_idx);
        void clear();
        const T& front() const;
        const T& back() const;
        const T& at(size_t n) const;
        T& operator[](size_t n);
        const T& operator[](size_t n) const;
        // auto begin() const -> decltype(std::declval<std::vector<T>>().begin());

        std::tuple<size_t, T, float> getNearestPt(const T &src_pt);
        int32_t GetPtIndex(const T &pt);
        std::pair<int32_t, T> prev(size_t cur_idx, size_t n);
        std::pair<int32_t, T> next(size_t cur_idx, size_t n);
        bool getNormalAngle(size_t idx, float &angle_r);
        bool getTangentialAngle(size_t idx, float &angle_r);
        bool getCurvature(size_t idx, float &curvature);
        float calcPathDistance();

    private:
        
        bool getCurvatureHelper(size_t idx, size_t step, float &curvature);

        std::vector<T> m_pts;
        bool m_ring_path;
    };

    template <>
    void Path<Pose>::add(const Pose &pose);
    template <>
    std::tuple<size_t, Pose, float> Path<Pose>::getNearestPt(const Pose &src_pt);
    template <>
    bool Path<Pose>::getNormalAngle(size_t target_idx, float& angle_r);
    template <>
    bool Path<Pose>::getCurvatureHelper(size_t target_idx, size_t step, float &curvature);
    template <>
    float Path<Pose>::calcPathDistance();
}

#endif // UTILS_PATH_H
