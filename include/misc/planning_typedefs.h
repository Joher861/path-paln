#ifndef MISC_PLANNING_TYPEDEF_H
#define MISC_PLANNING_TYPEDEF_H

#include <memory>

#include "geometry/pose.h"
#include "geometry/grid.h"
#include "geometry/grid_rect.h"
#include "geometry/rect.h"
#include "geometry/line.h"
#include "geometry/grid_contour.h"
#include "geometry/contour.h"
#include "geometry/area.h"
#include "geometry/trajectory.h"
#include "geometry/path.h"

using RobotTransform = planning_utils::Pose;
using RobotPose = planning_utils::Pose;
using Point = planning_utils::Point;
using RobotGrid = planning_utils::Grid;
using RobotGridHash = planning_utils::GridHash;
using RobotGridRect = planning_utils::GridRect;
using Rect = planning_utils::Rect;
using Line = planning_utils::Line;
using LinePtr = std::shared_ptr<Line>;
using RobotGridPath = planning_utils::Path<planning_utils::Grid>;
using RobotGridPathPtr = std::shared_ptr<RobotGridPath>;
using PointPath = planning_utils::Path<planning_utils::Point>;
using PointPathPtr = std::shared_ptr<PointPath>;
using PosePath = planning_utils::Path<planning_utils::Pose>;
using PosePathPtr = std::shared_ptr<PosePath>;
using RobotGridTrajectory = planning_utils::Trajectory<planning_utils::Grid>;
using RobotGridTrajectoryPtr = std::shared_ptr<RobotGridTrajectory>;
using PointTrajectory = planning_utils::Trajectory<planning_utils::Point>;
using PointTrajectoryPtr = std::shared_ptr<PointTrajectory>;
using RobotPoseTrajectory = planning_utils::Trajectory<planning_utils::Pose>;
using RobotPoseTrajectoryPtr = std::shared_ptr<RobotPoseTrajectory>;
using RobotGridContour = planning_utils::GridContour;
using Contour = planning_utils::Contour;
using ContourPtr = std::shared_ptr<planning_utils::Contour>;
using ContourGrid = planning_utils::ContourGrid;
using AreaContour = planning_utils::AreaContour;
using Area = planning_utils::Area;
using AreaPtr = std::shared_ptr<planning_utils::Area>;
using AreaGrids = planning_utils::AreaGrids;
using RobotTransform = planning_utils::Pose;

#endif // MISC_PLANNING_TYPEDEF_H 
