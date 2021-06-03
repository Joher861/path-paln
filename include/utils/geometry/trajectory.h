#ifndef UTILS_TRAJECTORY_H
#define UTILS_TRAJECTORY_H

#include <vector>
#include <tuple>
#include <list>

#include "grid.h"
#include "point.h"
#include "pose.h"

namespace planning_utils
{
    template <typename T>
    class Trajectory
    {
    public:

        Trajectory();
        Trajectory(const Trajectory<T> &traj);

        bool empty() const;
        size_t size() const;
        void add(const T &pt, uint64_t ts);
        void erase(size_t idx);
        void erase(size_t start_idx, size_t end_idx);
        void clear();
        const std::pair<T, uint64_t>& front() const;
        const std::pair<T, uint64_t>& back() const;
        const std::pair<T, uint64_t>& at(size_t n) const;
        std::pair<T, uint64_t>& operator[](size_t n);
        const std::pair<T, uint64_t>& operator[](size_t n) const;

        std::tuple<size_t, std::pair<T, uint64_t>, float>
            getNearestPt(const T &src_pt) const;
        int32_t GetPtIndex(const T &pt) const;
        int32_t GetPtIndex(uint64_t ts) const;
        std::pair<int32_t, std::pair<T, uint64_t>>
            prev(size_t cur_idx, size_t n) const;
        std::pair<int32_t, std::pair<T, uint64_t>>
            next(size_t cur_idx, size_t n) const;
        bool getNormalAngle(size_t idx, float &angle_r) const;
        bool getTangentialAngle(size_t idx, float &angle_r) const;
        bool getCurvature(size_t idx, float &curvature) const;

    private:
        
        bool getCurvatureHelper(size_t idx, size_t step, float &curvature) const;

        std::vector<std::pair<T, uint64_t>> m_pts;
    };

    template <>
    void Trajectory<Pose>::add(const Pose &pose, uint64_t ts);
    template <>
    std::tuple<size_t, std::pair<Pose, uint64_t>, float>
        Trajectory<Pose>::getNearestPt(const Pose &src_pt) const;
    template <>
    bool Trajectory<Pose>::getNormalAngle(size_t target_idx, float& angle_r) const;
    template <>
    bool Trajectory<Pose>::getCurvatureHelper(size_t target_idx, size_t step, float &curvature) const;
}

#endif // UTILS_TRAJECTORY_H