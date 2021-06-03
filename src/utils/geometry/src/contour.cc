#include "contour.h"
#include "geometry_func.h"

namespace planning_utils
{
    // Contour::Contour(const std::vector<Grid> &grids,
    //         ContourType contour_type = OUTER_CONTOUR)
    // {
    //     Contour(std::move(grids), contour_type);
    // }

    Contour::Contour(const std::vector<Grid> &&grids, ContourType contour_type)
        : m_contour_type(contour_type)
    {
        if (grids.size() < 3)
            return;
        
        generateContourGrids<std::vector<Grid>, Grid>(grids,
            [](const Grid &cell){
                return cell;
            });
        judgeClockwiseType();
        generateGridInfo();
        calculateRect();
    }

    Contour::Contour(const Path<Grid> &&path, ContourType contour_type)
        : m_contour_type(contour_type)
    {
        if (path.size() < 3)
            return;
        
        generateContourGrids<Path<Grid>, Grid>(path,
            [](const Grid &cell){
                return cell;
            });
        judgeClockwiseType();
        generateGridInfo();
        calculateRect();
    }

    Contour::Contour(const Path<Point> &&path,
            std::function<Grid(const Pose&)> poseToGrid, ContourType contour_type)
        : m_contour_type(contour_type)
    {
        size_t path_size = path.size();
        if (path_size < 3)
            return;
        
        std::vector<Grid> grids;
        for (size_t i = 0; i < path_size; ++i)
        {
            grids.emplace_back(poseToGrid(Pose{path[i]}));
        }

        generateContourGrids<std::vector<Grid>, Grid>(grids,
            [](const Grid &cell){
                return cell;
            });
        judgeClockwiseType();
        generateGridInfo();
        calculateRect();
    }

    Contour::Contour(const Path<Pose> &&path,
            std::function<Grid(const Pose&)> poseToGrid, ContourType contour_type)
        : m_contour_type(contour_type)
    {
        size_t path_size = path.size();
        if (path_size < 3)
            return;
        
        std::vector<Grid> grids;
        for (size_t i = 0; i < path_size; ++i)
        {
            grids.emplace_back(poseToGrid(path[i]));
        }

        generateContourGrids<std::vector<Grid>, Grid>(grids,
            [](const Grid &cell){
                return cell;
            });
        judgeClockwiseType();
        generateGridInfo();
        calculateRect();
    }

    Contour::Contour(const Trajectory<Grid> &&traj, ContourType contour_type)
        : m_contour_type(contour_type)
    {
        if (traj.size() < 3)
            return;
        
        generateContourGrids<Trajectory<Grid>, std::pair<Grid, uint64_t>>(traj,
            [](const std::pair<Grid, uint64_t> &cell){
                return cell.first;
            });
        judgeClockwiseType();
        generateGridInfo();
        calculateRect();
    }

    Contour::Contour(const Trajectory<Point> &&traj,
            std::function<Grid(const Pose&)> poseToGrid, ContourType contour_type)
        : m_contour_type(contour_type)
    {
        size_t traj_size = traj.size();
        if (traj_size < 3)
            return;
        
        std::vector<Grid> grids;
        for (size_t i = 0; i < traj_size; ++i)
        {
            grids.emplace_back(poseToGrid(Pose{traj[i].first}));
        }

        generateContourGrids<std::vector<Grid>, Grid>(grids,
            [](const Grid &cell){
                return cell;
            });
        judgeClockwiseType();
        generateGridInfo();
        calculateRect();
    }

    Contour::Contour(const Trajectory<Pose> &&traj,
            std::function<Grid(const Pose&)> poseToGrid, ContourType contour_type)
        : m_contour_type(contour_type)
    {
        size_t traj_size = traj.size();
        if (traj_size < 3)
            return;
        
        std::vector<Grid> grids;
        for (size_t i = 0; i < traj_size; ++i)
        {
            grids.emplace_back(poseToGrid(traj[i].first));
        }

        generateContourGrids<std::vector<Grid>, Grid>(grids,
            [](const Grid &cell){
                return cell;
            });
        judgeClockwiseType();
        generateGridInfo();
        calculateRect();
    }

    Contour::Contour(const Contour &contour)
        : m_grids(contour.m_grids), m_cw_type(contour.m_cw_type),
        m_contour_type(contour.m_contour_type), m_rect(contour.m_rect)
    {}

    Contour::Contour(const Contour &&contour)
        : m_grids(contour.m_grids), m_cw_type(contour.m_cw_type),
        m_contour_type(contour.m_contour_type), m_rect(contour.m_rect)
    {}


    Contour::ClockwiseType Contour::getClockwiseType() const
    {
        return m_cw_type;
    }

    Contour::ContourType Contour::getContourType() const
    {
        return m_contour_type;
    }

    GridRect Contour::getRect() const
    {
        return m_rect;
    }

    bool Contour::empty() const
    {
        return m_grids.empty();
    }

    size_t Contour::size() const
    {
        return m_grids.size();
    }

    const ContourGrid& Contour::at(size_t n) const
    {
        return m_grids.at(n);
    }

    ContourGrid& Contour::operator[](size_t n)
    {
        return m_grids[n];
    }

    const ContourGrid& Contour::operator[](size_t n) const
    {
        return m_grids[n];
    }

    std::tuple<size_t, ContourGrid, float>
        Contour::getNearestGrid(const Grid &src_grid)
    {
        if (m_grids.empty())
            return {0, ContourGrid{}, -1.0f};

        size_t min_idx = 0;
        ContourGrid &min_grid = m_grids.front();
        float min_dist
            = hypot(src_grid.x - min_grid.grid.x, src_grid.y - min_grid.grid.y);

        size_t size = m_grids.size();
        for (size_t i = 1; i < size; ++i)
        {
            ContourGrid &grid = m_grids[i];
            float dist = hypot(src_grid.x - grid.grid.x, src_grid.y - grid.grid.y);
            if (dist <= min_dist)
            {
                min_idx = i;
                min_grid = grid;
                min_dist = dist;
            }
        }

        return {min_idx, min_grid, min_dist};
    }

    int32_t Contour::getGridIndex(const Grid &grid)
    {
        size_t size = m_grids.size();
        for (size_t i = 0; i < size; ++i)
        {
            if (m_grids[i].grid == grid)
                return i;
        }

        return -1;
    }

    std::pair<int32_t, ContourGrid> Contour::prev(size_t cur_idx, size_t n)
    {
        size_t size = m_grids.size();
        if (size == 0 || cur_idx >= size || n >= size)
        {
            return {-1, ContourGrid{}};
        }
        
        size_t idx;
        if (cur_idx >= n)
            idx = cur_idx - n;
        else
            idx = size + cur_idx - n;

        return {idx, m_grids[idx]};
    }

    std::pair<int32_t, ContourGrid> Contour::next(size_t cur_idx, size_t n)
    {
        size_t size = m_grids.size();
        if (size == 0 || cur_idx >= size || n >= size)
        {
            return {-1, ContourGrid{}};
        }
        
        size_t idx;
        if (cur_idx + n < size)
            idx = cur_idx + n;
        else
            idx = cur_idx + n - size;

        return {idx, m_grids[idx]};
    }

    cv::Mat Contour::getMat() const
    {
        Grid size = m_rect.getSize();
        cv::Mat mat(size.x, size.y, CV_8UC1, cv::Scalar(0));

        for (auto & contour_grid : m_grids)
        {
            const Grid &grid = contour_grid.grid;
            mat.at<uint8_t>(m_rect.max_grid.x - grid.x,
                m_rect.max_grid.y - grid.y) = 255;
        }

        return mat;
    }

    void Contour::print() const
    {
        printf("clockwise type = %d\n", m_cw_type);
        printf("contour type = %d\n", m_contour_type);
        printf("rect = %s\n", m_rect.toString().c_str());
        int cnt = 0;
        for (auto & contour_grid : m_grids)
        {
            printf("grid %d = %s, %f, %f, %f\n", 
                cnt++, contour_grid.grid.toString().c_str(),
                rad2deg(contour_grid.tangential), rad2deg(contour_grid.normal),
                contour_grid.curvature);
        }
    }

    std::vector<ContourGrid> Contour::getPart(size_t start_idx, size_t end_idx,
        Contour::ClockwiseType cw_type)
    {
        std::vector<ContourGrid> cgs;
        size_t size = m_grids.size();
        if (start_idx >= size || end_idx > size)
            return cgs;

        bool different_cw = (cw_type != MAX_CW_TYPE && cw_type != m_cw_type);

        std::function<std::pair<int32_t, ContourGrid>(size_t, size_t)> step;
        step = std::bind(&Contour::next, this,
            std::placeholders::_1, std::placeholders::_2);
        if (different_cw)
        {
            step = std::bind(&Contour::prev, this,
                std::placeholders::_1, std::placeholders::_2);
        }
        
        size_t n = 0;
        while (true)
        {
            auto && [idx, cg] = step(start_idx, n++);
            if (idx == -1)
            {
                cgs.clear();
                return cgs;
            }

            size_t next_idx = static_cast<size_t>(idx);
            cgs.push_back(cg);
            if (next_idx == end_idx)
            {
                if (different_cw)
                {
                    for (auto & cg : cgs)
                        cg.tangential = toNPPiAngleRangeR(cg.tangential + M_PI);
                }
                return cgs;
            }
        }

        return cgs;
    }

    std::vector<ContourGrid> Contour::getPart(const Grid &start_grid,
        const Grid &end_grid, ClockwiseType cw_type)
    {
        std::vector<ContourGrid> cgs;
        int32_t start = getGridIndex(start_grid);
        if (start < 0)
            return cgs;
        
        bool different_cw = (cw_type != MAX_CW_TYPE && cw_type != m_cw_type);

        size_t start_idx = static_cast<size_t>(start);
        std::function<std::pair<int32_t, ContourGrid>(size_t, size_t)> step;
        step = std::bind(&Contour::next, this,
            std::placeholders::_1, std::placeholders::_2);
        if (different_cw)
        {
            step = std::bind(&Contour::prev, this,
                std::placeholders::_1, std::placeholders::_2);
        }

        size_t n = 0;
        while (true)
        {
            auto && [idx, cg] = step(start_idx, n++);
            if (idx == -1)
            {
                cgs.clear();
                return cgs;
            }

            cgs.push_back(cg);
            if (cg.grid == end_grid)
            {
                if (different_cw)
                {
                    for (auto & cg : cgs)
                        cg.tangential = toNPPiAngleRangeR(cg.tangential + M_PI);
                }
                return cgs;
            }
        }
    }

    template <typename Collection, typename T>
        void Contour::generateContourGrids(const Collection &grids,
            std::function<Grid(const T&)> get_grid)
    {
        size_t grids_size = grids.size();
        if (grids_size == 0)
            return;
        Grid prev_grid = get_grid(grids.at(0));
        m_grids.emplace_back(prev_grid);
        for (size_t i = 1; i <= grids_size; ++i)  
        {
            size_t idx = i;
            if (i == grids_size)
                idx = 0;
            Grid grid = get_grid(grids.at(idx));

            if (abs(grid.x - prev_grid.x) > 1 || abs(grid.y - prev_grid.y) > 1)
            {
                std::vector<Grid> &&line = getLine(prev_grid, grid);
                size_t line_size = line.size();
                for (size_t line_idx = 1; line_idx < line_size; ++line_idx)
                {
                    Grid lg = line[line_idx];
                    if (lg != m_grids.back().grid)
                        m_grids.emplace_back(lg);
                }
            }
            else
            {
                if (grid != m_grids.back().grid)
                    m_grids.emplace_back(grid);
            }
            prev_grid = grid;
        }
    }

    void Contour::judgeClockwiseType()
    {
        size_t grids_size = m_grids.size();
        if (grids_size < 3)
            m_cw_type = CW;

        int32_t sum = 0;
        Grid prev_grid = m_grids.back().grid;
        for (auto & contour_grid : m_grids)
        {
            Grid &grid = contour_grid.grid;
            sum += (grid.x - prev_grid.x) * (grid.y + prev_grid.y);
            prev_grid = grid;
        }
        
        if (sum >= 0)
            m_cw_type = CW;
        else
            m_cw_type = CCW;
    }

    bool Contour::getTangentialAngle(size_t target_idx, float& angle_r)
    {
        std::list<Grid> neighbour_prev_grids;
        std::list<Grid> neighbour_next_grids;
        auto && [idx0, grid0] = prev(target_idx, 3);
        auto && [idx6, grid6] = next(target_idx, 3);
        if (idx0 >= 0 && idx6 >= 0)
        {
            neighbour_prev_grids.push_back(grid0.grid);
            neighbour_next_grids.push_front(grid6.grid);
        }
        auto && [idx1, grid1] = prev(target_idx, 2);
        auto && [idx5, grid5] = next(target_idx, 2);
        if (idx1 >= 0 && idx5 >= 0)
        {
            neighbour_prev_grids.push_back(grid1.grid);
            neighbour_next_grids.push_front(grid5.grid);
        }
        auto && [idx2, grid2] = prev(target_idx, 1);
        auto && [idx4, grid4] = next(target_idx, 1);
        if (idx2 >= 0 && idx4 >= 0)
        {
            neighbour_prev_grids.push_back(grid2.grid);
            neighbour_next_grids.push_front(grid4.grid);
        }
        auto && [idx3, grid3] = next(target_idx, 0);
        if (idx3 < 0)
            return false;
        neighbour_next_grids.push_front(grid3.grid);

        std::vector<Grid> n_grids;
        n_grids.insert(n_grids.end(), neighbour_prev_grids.begin(),
            neighbour_prev_grids.end());
        n_grids.insert(n_grids.end(), neighbour_next_grids.begin(),
            neighbour_next_grids.end());

        if (n_grids.size() < 3)
            return false;

        std::pair<float, float> vec = std::make_pair(0, 0);    
        for (size_t i = 0; i < n_grids.size() - 2; ++i)
        {
            Point offset{
                static_cast<float>(n_grids[i + 2].x - n_grids[i].x),
                static_cast<float>(n_grids[i + 2].y - n_grids[i].y)
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

        return true;
    }
    
    bool Contour::getNormalAngle(size_t target_idx, float& angle_r)
    {
        float tangential_angle;
        if (!getTangentialAngle(target_idx, tangential_angle))
            return false;
        
        if ((m_cw_type == CCW && m_contour_type == OUTER_CONTOUR)
            || (m_cw_type == CW && m_contour_type == INNER_CONTOUR))
        {
            angle_r = toStdAngleRangeR(angle_r + 0.5f * M_PI);
        }
        else if ((m_cw_type == CCW && m_contour_type == INNER_CONTOUR)
            || (m_cw_type == CW && m_contour_type == OUTER_CONTOUR))
        {
            angle_r = toStdAngleRangeR(angle_r - 0.5f * M_PI);
        }
        else
        {
            return false;
        }

        return true;
    }

    bool Contour::getCurvatureHelper(size_t target_idx, size_t step, float &curvature)
    {
        auto && [prev_idx, prev_grid] = prev(target_idx, step);
        if (prev_idx < 0)
            return false;
        auto && [next_idx, next_grid] = next(target_idx, step);
        if (next_idx < 0)
            return false;

        Grid &cur_grid = m_grids[target_idx].grid;

        int dx = next_grid.grid.x - cur_grid.x;

        if (dx == 0)
        {
            curvature = 0;
            return true;
        }
        
        float first_derivative = (next_grid.grid.y - cur_grid.y) / dx;
        float second_derivative
            = (next_grid.grid.y + next_grid.grid.y - 2 * cur_grid.y) / (dx * dx);

        curvature
            = fabs(second_derivative) / powf((1 + first_derivative * first_derivative), 1.5f);

        return true;
    }

    bool Contour::getCurvature(size_t target_idx, float& curvature)
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

    void Contour::generateGridInfo()
    {
        for (size_t idx = 0; idx < m_grids.size(); ++idx)
        {
            ContourGrid &grid = m_grids[idx];
            if (getTangentialAngle(idx, grid.tangential))
            {
                if ((m_cw_type == CCW && m_contour_type == OUTER_CONTOUR)
                    || (m_cw_type == CW && m_contour_type == INNER_CONTOUR))
                {
                    grid.normal = toStdAngleRangeR(grid.tangential + 0.5f * M_PI);
                }
                else if ((m_cw_type == CCW && m_contour_type == INNER_CONTOUR)
                    || (m_cw_type == CW && m_contour_type == OUTER_CONTOUR))
                {
                    grid.normal = toStdAngleRangeR(grid.tangential - 0.5f * M_PI);
                }
            }
            getCurvature(idx, grid.curvature);
        }
    }

    void Contour::calculateRect()
    {
        if (m_grids.empty())
            return;

        Grid first_grid = m_grids.front().grid;
        m_rect.min_grid = first_grid;
        m_rect.max_grid = first_grid;

        size_t contour_size = m_grids.size();
        for (size_t i = 1; i < contour_size; ++i)
        {
            Grid &grid = m_grids[i].grid;
            m_rect.min_grid.x = std::min(m_rect.min_grid.x, grid.x);
            m_rect.min_grid.y = std::min(m_rect.min_grid.y, grid.y);
            m_rect.max_grid.x = std::max(m_rect.max_grid.x, grid.x);
            m_rect.max_grid.y = std::max(m_rect.max_grid.y, grid.y);
        }
    }

    bool operator==(const Contour &c1, const Contour &c2)
    {
        if (c1.m_cw_type != c2.m_cw_type)
            return false;
        if (c1.m_contour_type != c2.m_contour_type)
            return false;
        
        size_t c1_grids_size = c1.m_grids.size();
        size_t c2_grids_size = c2.m_grids.size();
        if (c1_grids_size != c2_grids_size)
            return false;

        if (c1_grids_size == 0)
            return true;
        if (c1_grids_size == 1 && c1.m_grids.front() == c2.m_grids.front())
            return true;

        const ContourGrid &c1_front = c1.m_grids.front();
        bool find_same = false;
        size_t c1_start = 1;
        size_t c2_start;
        for (size_t i = 0; i < c2_grids_size; i++)
        {
            if (c1_front  == c2.m_grids[i])
            {
                find_same = true;
                if (i != c2_grids_size - 1)
                    c2_start = i + 1;
                else
                    c2_start = 0;
                break;
            }
        }
        if (!find_same)
            return false;

        for (size_t i = c1_start; i < c1_grids_size; ++i)
        {
            size_t c2_idx = c2_start + i;
            if (c2_idx >= c2_grids_size)
                c2_idx -= c2_grids_size;
            if (c1.m_grids[i] != c2.m_grids[c2_idx])
                return false;
        }

        return true;
    }

    AreaContour::AreaContour(const std::vector<Grid> &&outer_grids)
        : outer_contour(std::forward<const std::vector<Grid>>(outer_grids))
    {}

    AreaContour::AreaContour(const Contour &&outer_contour)
        : outer_contour(std::forward<const Contour>(outer_contour))
    {}

    AreaContour::AreaContour(const AreaContour &area_contour)
        : outer_contour(area_contour.outer_contour),
          inner_contours(area_contour.inner_contours)
    {}

    void AreaContour::addInnerContour(const std::vector<Grid> &&inner_grids)
    {
        inner_contours.emplace_back(inner_grids, Contour::INNER_CONTOUR);
    }

    void AreaContour::addInnerContour(const Contour &&inner_contour)
    {
        inner_contours.emplace_back(inner_contour);
    }

    cv::Mat AreaContour::getMat() const
    {
        GridRect rect = outer_contour.getRect();
        Grid size = rect.getSize();
        cv::Mat mat(size.x, size.y, CV_8UC1, cv::Scalar(0));

        size_t outer_contour_size = outer_contour.size();
        for (size_t i = 0; i < outer_contour_size; ++i)
        {
            const Grid &grid = outer_contour[i].grid;
            mat.at<uint8_t>(rect.max_grid.x - grid.x,
                rect.max_grid.y - grid.y) = 255; 
        }

        for (auto &inner_contour : inner_contours)
        {
            size_t inner_contour_size = inner_contour.size();
            for (size_t i = 0; i < inner_contour_size; ++i)
            {
                const Grid &grid = inner_contour[i].grid;
                mat.at<uint8_t>(rect.max_grid.x - grid.x,
                    rect.max_grid.y - grid.y) = 255; 
            }
        }

        return mat;
    }
}