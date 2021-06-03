#ifndef MAP_MAP_BASE_INFO_H
#define MAP_MAP_BASE_INFO_H

#include <memory>

#include "misc/planning_typedefs.h"

using MapSize = RobotGrid;

namespace planning_map
{
    inline Point toMapCellPt(const Point &pt, float resolution)
    {
        Point result{
            resolution * static_cast<int>(round(pt.x / resolution)),
            resolution * static_cast<int>(round(pt.y / resolution))};

        return result;
    }

    inline RobotPose toMapCellPose(const RobotPose &pose, float resolution)
    {
        RobotPose result{
            resolution * static_cast<int>(round(pose.pt.x / resolution)),
            resolution * static_cast<int>(round(pose.pt.y / resolution)),
            pose.theta};

        return result;
    }

    enum MapSaveOption
    {
        SAVE_OCCUPIED = 0,
        SAVE_WHOLE,
        SAVE_FIXED
    };

    // ******************************* map info ********************************

    struct MapInfo
    {
        float resolution, localResolution;
        MapSize size, localSize;
        Point origin_pt, local_origin_pt;
        RobotGrid origin_grid, local_origin_grid;
        Rect range;
        RobotGridRect grid_range;

	void setSize(const MapSize &_size)
	{
	    size = _size;
	    if (size.x % 2 == 0)
	        size.x += 1;
	    if (size.y % 2 == 0)
	        size.y += 1;
	    origin_grid = size / 2;
	    grid_range.min_grid = RobotGrid{0, 0};
	    grid_range.max_grid = size - RobotGrid{1, 1};
	}

	void setOriginPt(const Point &pt)
	{
	    Point map_cell_pt = toMapCellPt(pt, resolution);
	    origin_pt = map_cell_pt;
	    Point half_size_in_meter{
	        0.5f * size.x,
	        0.5f * size.y};
	    half_size_in_meter = half_size_in_meter * resolution;
	    range = Rect{
	        origin_pt - half_size_in_meter,
	        origin_pt + half_size_in_meter};
	}

	void setInfo(float _resolution, const MapSize &_size,
	             const Point &_origin_pt)
	{
	    resolution = _resolution;
	    setSize(_size);
	    setOriginPt(_origin_pt);
	}

        void setSizeOriginPtLocal(const MapSize &_size, const MapSize &_localSize, float _resolution, float _localResolution,
                             const Point &_origin_pt, const Point &_local_origin_pt)
        {
            size = _size;
            origin_grid.x = floor(-_origin_pt.x * _resolution);
            origin_grid.y = floor(-_origin_pt.y * _resolution);
            local_origin_grid.x = floor(-_local_origin_pt.x * _resolution);
            local_origin_grid.y = floor(-_local_origin_pt.y * _resolution);

            grid_range.min_grid = RobotGrid{0, 0};
            grid_range.max_grid = size - RobotGrid{1, 1};

            Point size_in_meter{
                (float)size.x,
                (float)size.y};
            size_in_meter = size_in_meter * resolution;
            range = Rect{
                origin_pt,
                origin_pt + size_in_meter};
        }

        void setInfoLocal(float _resolution, float _localResolution, const MapSize &_size, const MapSize &_Localsize,
                     const Point &_origin_pt, const Point &_local_origin_pt)
        {
            resolution = _resolution;
            localResolution = _localResolution;
            localSize = _Localsize;
            local_origin_pt = _local_origin_pt;
            origin_pt = _origin_pt;

            setSizeOriginPtLocal(_size, localSize, _resolution, _localResolution, _origin_pt, _local_origin_pt);
        }
    };
    using MapInfoPtr = std::shared_ptr<MapInfo>;
} // namespace planning_map

#endif // MAP_MAP_BASE_INFO_H