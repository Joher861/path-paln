#ifndef MAP_LAYER_MAP_H
#define MAP_LAYER_MAP_H

#include <string>
#include <stdexcept>
#include <functional>
#include <strings.h>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "layer.h"
#include "map_log.h"
#include "misc/planning_typedefs.h"
#include "misc/planning_defs.h"

using MapSize = RobotGrid;

namespace planning_map
{
    template <typename Cell>
    class LayerMap : public Layer
    {
    public:
        LayerMap(std::string name, MapInfoPtr map_info, Cell &default_value,
                 std::string log_name);
        virtual ~LayerMap();

        void reset();
        void reset(const RobotGridRect &range);
        Cell *getMapAddr();
        Cell &operator[](const RobotGrid &grid) const;
        void set(const RobotGrid &grid, const Cell &cell);
        void set(int32_t x, int32_t y, const Cell &cell);
        void get(const RobotGrid &grid, Cell &cell);
        void get(int32_t x, int32_t y, Cell &cell);
        void getMap(const RobotGridRect &region, Cell *dest_map);
        void setMap(const RobotGridRect &region, Cell *src_map);
        void setMapFromLayer(const RobotGridRect &dest_region,
                             LayerPtr layer, const RobotGridRect &src_region);

        bool inRange(const Point &pt);
        bool inRange(const RobotGrid &grid);
        RobotGrid poseToGrid(const RobotPose &pose);
        RobotGrid poseToGridWithoutBoundary(const RobotPose &pose);
        RobotPose gridToPose(const RobotGrid &grid);
        cv::Mat saveMapToMat(RobotGridRect region, MapSaveOption opt,
                             uint8_t channel = CV_8UC1, uint32_t padding = 5);
        bool loadMapFromMat(const RobotGridRect &region, const cv::Mat &map,
                            uint8_t channel = CV_8UC1);
        void regionOperate(const RobotGridRect &region,
                           std::function<void(Cell &)> func);

        void raytraceLine(std::vector<RobotGrid> &cells,
                          RobotGrid grid0, RobotGrid grid1, unsigned int max_length = UINT_MAX);

        void polygonOutlineCells(const std::vector<RobotGrid> &polygon,
                                 std::vector<RobotGrid> &polygon_cells);

        void convexFillCells(const std::vector<RobotGrid> &polygon,
                             std::vector<RobotGrid> &polygon_cells);

        bool setConvexPolygonCost(const std::vector<RobotPose> &polygon, Cell cell);

    protected:
        virtual bool isDefaultValueZero() const = 0;
        int32_t getIndex(const RobotGrid &grid) const;
        int32_t getIndex(int32_t x, int32_t y) const;
        RobotGrid parseIndex(int32_t index) const;
        bool outOfRange(const RobotGrid &grid) const;
        bool outOfRange(int32_t x, int32_t y) const;
        virtual bool isOccupied(const RobotGrid &grid) const = 0;
        RobotGridRect getOccupiedRegion() const;
        RobotGridRect getPaddedRegion(const RobotGridRect &region,
                                   uint32_t padding) const;
        virtual void setMapGridToMatCell(const RobotGrid &grid, uint8_t *mat_cell,
                                         uint8_t channel) const = 0;
        virtual void setMapGridFromMatCell(const RobotGrid &grid,
                                           const uint8_t *mat_cell, uint8_t channel) const = 0;

        inline int sign(int x)
        {
            return x > 0 ? 1.0 : -1.0;
        }

        void bresenham2D(std::vector<RobotGrid> &cells,
                         unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                         int offset_b, unsigned int offset, unsigned int max_length);

        bool m_initialized;
        std::string m_name;
        std::string m_log_name;
        MapInfoPtr m_map_info;
        Cell *m_map;
        Cell m_default_value;
        size_t m_cell_size;
        size_t m_total_size;
    };

    template <typename Cell>
    LayerMap<Cell>::LayerMap(std::string name,
                             MapInfoPtr map_info,
                             Cell &default_value,
                             std::string log_name)
        : m_initialized(false),
          m_name(name),
          m_log_name(log_name),
          m_map_info(map_info),
          m_map(nullptr),
          m_default_value(default_value),
          m_cell_size(sizeof(Cell)),
          m_total_size(map_info->size.x * map_info->size.y)
    {
        m_map = new Cell[m_total_size];
        for (size_t i = 0; i < m_total_size; i++)
            m_map[i] = default_value;
        m_initialized = true;
    }

    template <typename Cell>
    LayerMap<Cell>::~LayerMap()
    {
        if (m_initialized)
        {
            if (m_map)
            {
                delete[] m_map;
            }
        }
    }

    template <typename Cell>
    void LayerMap<Cell>::reset()
    {
        // 对于default_value为0的对象，可以用bzero清零
        if (isDefaultValueZero())
        {
            bzero(m_map, m_total_size * m_cell_size);
            return;
        }

        for (size_t i = 0; i < m_total_size; i++)
        {
            m_map[i] = m_default_value;
        }
    }

    template <typename Cell>
    void LayerMap<Cell>::reset(const RobotGridRect &range)
    {
        RobotGrid min_grid = range.min_grid;
        RobotGrid max_grid = range.max_grid;
        MapSize range_size = max_grid - min_grid + RobotGrid{1, 1};
        if (isDefaultValueZero())
        {
            for (int32_t i = min_grid.x; i <= max_grid.x; ++i)
            {
                bzero(&m_map[getIndex(i, min_grid.y)],
                      range_size.y * m_cell_size);
            }
            return;
        }

        for (int32_t i = min_grid.x; i <= max_grid.x; ++i)
        {
            for (int32_t j = min_grid.y; j <= max_grid.y; ++j)
            {
                m_map[getIndex(i, j)] = m_default_value;
            }
        }
    }

    template <typename Cell>
    inline Cell *LayerMap<Cell>::getMapAddr()
    {
        return m_map;
    }

    template <typename Cell>
    inline Cell &LayerMap<Cell>::operator[](const RobotGrid &grid) const
    {
        if (outOfRange(grid))
        {
            std::string err_str = grid.toString() + " is out of map range.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::out_of_range(err_str.c_str());
        }

        return m_map[getIndex(grid)];
    }

    template <typename Cell>
    inline void LayerMap<Cell>::set(const RobotGrid &grid, const Cell &cell)
    {
        if (outOfRange(grid))
        {
            std::string err_str = grid.toString() + " is out of map range.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::out_of_range(err_str.c_str());
        }
        m_map[getIndex(grid)] = cell;
    }

    template <typename Cell>
    inline void LayerMap<Cell>::set(int32_t x, int32_t y, const Cell &cell)
    {
        RobotGrid grid{x, y};
        set(grid, cell);
    }

    template <typename Cell>
    inline void LayerMap<Cell>::get(const RobotGrid &grid, Cell &cell)
    {
        if (outOfRange(grid))
        {
            std::string err_str = grid.toString() + " is out of map range.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::out_of_range(err_str.c_str());
        }
        cell = m_map[getIndex(grid)];
    }

    template <typename Cell>
    inline void LayerMap<Cell>::get(int32_t x, int32_t y, Cell &cell)
    {
        RobotGrid grid{x, y};
        set(grid, cell);
    }

    template <typename Cell>
    void LayerMap<Cell>::getMap(const RobotGridRect &region, Cell *dest_map)
    {
        if (outOfRange(region.min_grid) || outOfRange(region.max_grid))
        {
            std::string err_str = region.toString() + " is out of map range.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::out_of_range(err_str.c_str());
        }

        size_t row_size = (region.max_grid.y - region.min_grid.y + 1) * m_cell_size;
        for (int32_t i = region.min_grid.x; i <= region.max_grid.x; i++)
        {
            int32_t index = getIndex(i, region.min_grid.y);
            memcpy(dest_map + i * m_cell_size,
                   m_map + index * m_cell_size, row_size);
        }
    }

    template <typename Cell>
    void LayerMap<Cell>::setMap(const RobotGridRect &region, Cell *src_map)
    {
        if (outOfRange(region.min_grid) || outOfRange(region.max_grid))
        {
            std::string err_str = region.toString() + " is out of map range.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::out_of_range(err_str.c_str());
        }

        size_t row_size = (region.max_grid.y - region.min_grid.y + 1) * m_cell_size;
        for (int32_t i = region.min_grid.x; i <= region.max_grid.x; i++)
        {
            int32_t index = getIndex(i, region.min_grid.y);
            memcpy(m_map + index * m_cell_size,
                   src_map + i * m_cell_size, row_size);
        }
    }

    template <typename Cell>
    void LayerMap<Cell>::setMapFromLayer(const RobotGridRect &dest_region,
                                         LayerPtr layer, const RobotGridRect &src_region)
    {
        RobotGrid src_size = src_region.getSize();
        RobotGrid dest_size = dest_region.getSize();
        if (src_size != dest_size)
        {
            std::string err_str = "src_size " + src_size.toString() + "dest_size " + dest_size.toString() + " is not same.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::invalid_argument(err_str.c_str());
        }

        if (outOfRange(dest_region.min_grid) || outOfRange(dest_region.max_grid))
        {
            std::string err_str = "dest_region " + dest_region.toString() + " is out of map range.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::out_of_range(err_str.c_str());
        }

        std::shared_ptr<LayerMap<Cell>> lyr = std::dynamic_pointer_cast<LayerMap<Cell>>(layer);

        if (lyr->outOfRange(src_region.min_grid) || lyr->outOfRange(src_region.max_grid))
        {
            std::string err_str = "src_region" + dest_region.toString() + " is out of map range.";
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            throw std::out_of_range(err_str.c_str());
        }

        Cell *src_map = lyr->getMapAddr();
        size_t row_size = src_size.y * m_cell_size;
        for (int32_t i = src_region.min_grid.x; i <= src_region.max_grid.x;
             i++)
        {
            int32_t src_index = lyr->getIndex(i, src_region.min_grid.y);
            int32_t dest_index = getIndex(i - src_region.min_grid.x + dest_region.min_grid.x,
                                          dest_region.min_grid.y);
            memcpy(m_map + dest_index, src_map + src_index, row_size);
        }
    }

    template <typename Cell>
    inline bool LayerMap<Cell>::inRange(const Point &pt)
    {
        return m_map_info->range.inRange(pt);
    }

    template <typename Cell>
    inline bool LayerMap<Cell>::inRange(const RobotGrid &grid)
    {
        return m_map_info->grid_range.inRange(grid);
    }

    template <typename Cell>
    inline RobotGrid LayerMap<Cell>::poseToGrid(const RobotPose &pose)
    {
        Point diff = pose.pt - m_map_info->origin_pt;
        diff = diff / m_map_info->resolution;
        RobotGrid grid_diff{(int32_t)roundf(diff.x), (int32_t)roundf(diff.y)};

        RobotGrid grid = m_map_info->origin_grid + grid_diff;
        if (!inRange(grid))
        {
            std::string err_str = pose.toString() + " => " + grid.toString() + " is out of map range, origin_pt = " + m_map_info->origin_pt.toString() + "map range = %s" + m_map_info->grid_range.toString();
            MAP_ERROR_LOG(m_log_name, "%s", err_str.c_str());
            TRIG_SEGFAULT();
            throw std::out_of_range(err_str.c_str());
        }

        return grid;
    }

    template <typename Cell>
    inline RobotGrid LayerMap<Cell>::poseToGridWithoutBoundary(const RobotPose &pose)
    {
        Point diff = pose.pt - m_map_info->origin_pt;
        diff = diff / m_map_info->resolution;
        RobotGrid grid_diff{(int32_t)roundf(diff.x), (int32_t)roundf(diff.y)};

        RobotGrid grid = m_map_info->origin_grid + grid_diff;
        if (!inRange(grid))
            MAP_WARN_LOG(m_log_name, "%s => %s  is out of map range.",
                         pose.toString().c_str(), grid.toString().c_str());

        return grid;
    }

    template <typename Cell>
    inline RobotPose LayerMap<Cell>::gridToPose(const RobotGrid &grid)
    {
        RobotGrid grid_diff = grid - m_map_info->origin_grid;
        Point diff{grid_diff.x * m_map_info->resolution,
                     grid_diff.y * m_map_info->resolution};

        return {m_map_info->origin_pt + diff, 0.0f};
    }

    template <typename Cell>
    cv::Mat LayerMap<Cell>::saveMapToMat(RobotGridRect region, MapSaveOption opt,
                                         uint8_t channel, uint32_t padding)
    {
        if (channel != CV_8UC1 && channel != CV_8UC3)
        {
            MAP_ERROR_LOG(m_log_name, "channel %d not supported", channel);
            return cv::Mat{};
        }

        if (opt == SAVE_WHOLE)
        {
            region = m_map_info->grid_range;
        }
        else if (opt == SAVE_OCCUPIED)
        {
            RobotGridRect occupied_region = getOccupiedRegion();
            region = getPaddedRegion(occupied_region, padding);
            if (outOfRange(region.min_grid))
                return cv::Mat{};
        }
        else if (opt == SAVE_FIXED)
        {
            if (outOfRange(region.min_grid) || outOfRange(region.max_grid))
                return cv::Mat{};
        }
        else
        {
            return cv::Mat{};
        }

        int32_t rows = region.max_grid.x - region.min_grid.x + 1;
        int32_t cols = region.max_grid.y - region.min_grid.y + 1;
        cv::Mat map(rows, cols, channel);

        int32_t channel_cnt = 1;
        if (channel == CV_8UC3)
            channel_cnt = 3;
        uint8_t *row;
        for (int32_t i = 0; i < rows; i++)
        {
            row = map.ptr<uint8_t>(i);
            for (int32_t j = 0; j < cols; j++)
            {
                uint8_t *ptr = row + j * channel_cnt;
                RobotGrid grid = region.max_grid - RobotGrid{i, j};
                setMapGridToMatCell(grid, ptr, channel);
            }
        }

        return map;
    }

    template <typename Cell>
    bool LayerMap<Cell>::loadMapFromMat(const RobotGridRect &region, const cv::Mat &map,
                                        uint8_t channel)
    {
        if (map.type() != channel)
        {
            MAP_ERROR_LOG(m_log_name, "channel %d is not match the input mat", channel);
            return false;
        }
        if (channel != CV_8UC1 && channel != CV_8UC3)
        {
            MAP_ERROR_LOG(m_log_name, "channel %d not supported", channel);
            return false;
        }

        int32_t channel_cnt = 1;
        if (channel == CV_8UC3)
            channel_cnt = 3;
        const uint8_t *row;
        for (int32_t i = 0; i < map.rows; i++)
        {
            row = map.ptr<uint8_t>(i);
            for (int32_t j = 0; j < map.cols; j++)
            {
                const uint8_t *ptr = row + j * channel_cnt;
                RobotGrid grid = region.max_grid - RobotGrid{i, j};
                setMapGridFromMatCell(grid, ptr, channel);
            }
        }
        return true;
    }

    template <typename Cell>
    void LayerMap<Cell>::regionOperate(const RobotGridRect &region,
                                       std::function<void(Cell &)> func)
    {
        if (!m_map_info->grid_range.inRange(region))
        {
            MAP_ERROR_LOG(m_log_name, "region operate failed, region %s is "
                                      "not in map range %s",
                          region.toString(),
                          m_map_info->grid_range.toString());
        }

        RobotGrid min_grid = region.min_grid;
        RobotGrid max_grid = region.max_grid;
        for (int32_t x = min_grid.x; x <= max_grid.x; ++x)
        {
            for (int32_t y = min_grid.y; y <= max_grid.y; ++y)
            {
                Cell &cell = m_map[getIndex(x, y)];
                func(cell);
            }
        }
    }

    template <typename Cell>
    inline int32_t LayerMap<Cell>::getIndex(const RobotGrid &grid) const
    {
        return grid.x * m_map_info->size.y + grid.y;
    }

    template <typename Cell>
    inline int32_t LayerMap<Cell>::getIndex(int32_t x, int32_t y) const
    {
        return x * m_map_info->size.y + y;
    }

    template <typename Cell>
    inline RobotGrid LayerMap<Cell>::parseIndex(int32_t index) const
    {
        RobotGrid grid;
        grid.x = index / m_map_info->size.y;
        grid.y = index % m_map_info->size.y;
        return grid;
    }

    template <typename Cell>
    inline bool LayerMap<Cell>::outOfRange(const RobotGrid &grid) const
    {
        return (grid.x < 0 || grid.x >= m_map_info->size.x || grid.y < 0 || grid.y >= m_map_info->size.y);
    }

    template <typename Cell>
    inline bool LayerMap<Cell>::outOfRange(int32_t x, int32_t y) const
    {
        return (x < 0 || x >= m_map_info->size.x || y < 0 || y >= m_map_info->size.y);
    }

    template <typename Cell>
    RobotGridRect LayerMap<Cell>::getOccupiedRegion() const
    {
        RobotGrid min_grid = m_map_info->grid_range.min_grid;
        RobotGrid max_grid = m_map_info->grid_range.max_grid;
        RobotGridRect rect{max_grid + RobotGrid{1, 1}, min_grid - RobotGrid{1, 1}};
        for (int32_t i = min_grid.x; i <= max_grid.x; i++)
        {
            for (int32_t j = min_grid.y; j <= max_grid.y; j++)
            {
                RobotGrid grid{i, j};
                if (isOccupied(grid))
                {
                    rect.min_grid.x = std::min(rect.min_grid.x, grid.x);
                    rect.min_grid.y = std::min(rect.min_grid.y, grid.y);
                    rect.max_grid.x = std::max(rect.max_grid.x, grid.x);
                    rect.max_grid.y = std::max(rect.max_grid.y, grid.y);
                }
            }
        }
        return rect;
    }

    template <typename Cell>
    inline RobotGridRect LayerMap<Cell>::getPaddedRegion(const RobotGridRect &region,
                                                      uint32_t padding) const
    {
        int32_t p = static_cast<int32_t>(padding);
        RobotGrid min_grid = region.min_grid - RobotGrid{p, p};
        RobotGrid max_grid = region.max_grid + RobotGrid{p, p};

        min_grid.x = std::max(min_grid.x, m_map_info->grid_range.min_grid.x);
        min_grid.y = std::max(min_grid.y, m_map_info->grid_range.min_grid.y);
        max_grid.x = std::min(max_grid.x, m_map_info->grid_range.max_grid.x);
        max_grid.y = std::min(max_grid.y, m_map_info->grid_range.max_grid.y);

        return RobotGridRect{min_grid, max_grid};
    }

    /**
     * @brief  A 2D implementation of Bresenham's raytracing algorithm... 
     *         applies an action at each step
     */
    template <typename Cell>
    inline void LayerMap<Cell>::bresenham2D(std::vector<RobotGrid> &cells,
                                            unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                                            int offset_b, unsigned int offset, unsigned int max_length)
    {
        RobotGrid grid;
        unsigned int end = std::min(max_length, abs_da);
        for (unsigned int i = 0; i < end; ++i)
        {
            cells.push_back(parseIndex(offset));
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        cells.push_back(parseIndex(offset));
    }

    /**
     * @brief  Raytrace a line and apply some action at each step
     * @param  at The action to take... a functor
     * @param  grid0 The starting coordinate
     * @param  grid1 The ending coordinate
     * @param  max_length The maximum desired length of the segment... 
     *           allows you to not go all the way to the endpoint
     */
    template <typename Cell>
    inline void LayerMap<Cell>::raytraceLine(std::vector<RobotGrid> &cells,
                                             RobotGrid grid0, RobotGrid grid1, unsigned int max_length)
    {
        int dx = grid1.x - grid0.x;
        int dy = grid1.y - grid0.y;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = sign(dx) * m_map_info->size.y;
        // int offset_dy = sign(dy) * size_x_;
        int offset_dy = sign(dy);

        // unsigned int offset = y0 * size_x_ + x0;
        unsigned int offset = getIndex(grid0);

        // we need to chose how much to scale our dominant dimension, based on
        //the maximum length of the line
        double dist = hypot(dx, dy);
        double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

        // if x is dominant
        if (abs_dx >= abs_dy) //>=
        {
            int error_y = abs_dx / 2;
            bresenham2D(cells, abs_dx, abs_dy, error_y, offset_dx, offset_dy,
                        offset, (unsigned int)(scale * abs_dx));
            return;
        }

        // otherwise y is dominant
        int error_x = abs_dy / 2;
        bresenham2D(cells, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
                    (unsigned int)(scale * abs_dy));
    }

    template <typename Cell>
    inline void LayerMap<Cell>::polygonOutlineCells(
        const std::vector<RobotGrid> &polygon, std::vector<RobotGrid> &polygon_cells)
    {
        for (unsigned int i = 0; i < polygon.size() - 1; ++i)
        {
            raytraceLine(polygon_cells, polygon[i].x, polygon[i].y,
                         polygon[i + 1].x, polygon[i + 1].y);
        }
        if (!polygon.empty())
        {
            unsigned int last_index = polygon.size() - 1;
            // we also need to close the polygon by going from the last point
            //to the first
            raytraceLine(polygon_cells, polygon[last_index].x,
                         polygon[last_index].y, polygon[0].x, polygon[0].y);
        }
    }

    template <typename Cell>
    inline void LayerMap<Cell>::convexFillCells(const std::vector<RobotGrid> &polygon,
                                                std::vector<RobotGrid> &polygon_cells)
    {
        // we need a minimum polygon of a triangle
        if (polygon.size() < 3)
            return;

        // first get the cells that make up the outline of the polygon
        polygonOutlineCells(polygon, polygon_cells);

        // quick bubble sort to sort points by x
        RobotGrid swap;
        unsigned int i = 0;
        while (i < polygon_cells.size() - 1)
        {
            if (polygon_cells[i].x > polygon_cells[i + 1].x)
            {
                swap = polygon_cells[i];
                polygon_cells[i] = polygon_cells[i + 1];
                polygon_cells[i + 1] = swap;

                if (i > 0)
                    --i;
            }
            else
                ++i;
        }

        i = 0;
        RobotGrid min_pt;
        RobotGrid max_pt;
        unsigned int min_x = polygon_cells[0].x;
        unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

        // walk through each column and mark cells inside the polygon
        for (unsigned int x = min_x; x <= max_x; ++x)
        {
            if (i >= polygon_cells.size() - 1)
                break;

            if (polygon_cells[i].y < polygon_cells[i + 1].y)
            {
                min_pt = polygon_cells[i];
                max_pt = polygon_cells[i + 1];
            }
            else
            {
                min_pt = polygon_cells[i + 1];
                max_pt = polygon_cells[i];
            }

            i += 2;
            while (i < polygon_cells.size() && polygon_cells[i].x == x)
            {
                if (polygon_cells[i].y < min_pt.y)
                    min_pt = polygon_cells[i];
                else if (polygon_cells[i].y > max_pt.y)
                    max_pt = polygon_cells[i];
                ++i;
            }

            RobotGrid pt;
            // loop though cells in the column
            for (unsigned int y = min_pt.y; y < max_pt.y; ++y)
            {
                pt.x = x;
                pt.y = y;
                polygon_cells.push_back(pt);
            }
        }
    }

    template <typename Cell>
    inline bool LayerMap<Cell>::setConvexPolygonCost(
        const std::vector<RobotPose> &polygon, Cell cell)
    {
        // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
        std::vector<RobotGrid> map_polygon;
        for (unsigned int i = 0; i < polygon.size(); ++i)
        {
            RobotGrid loc = poseToGridWithoutBoundary(polygon[i]);

            map_polygon.push_back(loc);
        }

        std::vector<RobotGrid> polygon_cells;

        // get the cells that fill the polygon
        convexFillCells(map_polygon, polygon_cells);

        // set the cost of those cells
        for (unsigned int i = 0; i < polygon_cells.size(); ++i)
        {
            unsigned int index = getIndex(polygon_cells[i]);
            m_map[index] = cell;
        }
        return true;
    }

} // namespace planning_map

#endif // MAP_LAYER_MAP_H