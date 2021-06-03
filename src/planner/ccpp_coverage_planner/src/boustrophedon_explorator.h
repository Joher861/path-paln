/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2016 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 11.2016
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <string>
#include <tuple>

#include <Eigen/Dense>

#include "navigation_planner/hybrid_breadth_first.h"

#include "concorde_TSP.h"
#include "genetic_TSP.h"
#include "meanshift2d.h"
#include "room_rotator.h"
#include "grid.h"
#include <boost/thread.hpp>
#include "misc/planning_typedefs.h"
#include "ccpp_data.h"

#define PI 3.14159265359

// Class that is used to store cells and obstacles in a certain manner. For this the vertexes are stored as points and
// the edges are stored as vectors in a counter-clockwise manner. The constructor becomes a set of respectively sorted
// points and computes the vectors out of them. Additionally the accessible/visible center of the polygon gets computed,
// to simplify the visiting order later, by using a meanshift algorithm.
class GeneralizedPolygon
{
protected:
	// vertexes
	std::vector<cv::Point> vertices_;

	// accessible center: a central point inside the polygon with maximum distance to walls
	cv::Point center_;

	// center of bounding rectangle of polygon, may be located outside the polygon, i.e. in an inaccessible area
	cv::Point bounding_box_center_;

	// min/max coordinates
	int max_x_, min_x_, max_y_, min_y_;

	// area of the polygon (cell number), in [pixel^2]
	double area_;

public:
	// constructor
	GeneralizedPolygon(const std::vector<cv::Point> &vertices, const double map_resolution);

	std::vector<cv::Point> getVertices() const
	{
		return vertices_;
	}

	cv::Point getCenter() const
	{
		return center_;
	}

	cv::Point getBoundingBoxCenter() const
	{
		return bounding_box_center_;
	}

	double getArea() const
	{
		return area_;
	}

	void drawPolygon(cv::Mat &image, const cv::Scalar &color) const;

	void getMinMaxCoordinates(int &min_x, int &max_x, int &min_y, int &max_y)
	{
		min_x = min_x_;
		max_x = max_x_;
		min_y = min_y_;
		max_y = max_y_;
	}
};

// Structure to save edges of a path on one row, that allows to easily get the order of the edges when planning the
// boustrophedon path.
struct BoustrophedonHorizontalLine
{
	cv::Point left_corner_, right_corner_;
};

// Structure for saving several properties of cells
struct BoustrophedonCell
{
	typedef std::set<boost::shared_ptr<BoustrophedonCell>> BoustrophedonCellSet;
	typedef std::set<boost::shared_ptr<BoustrophedonCell>>::iterator BoustrophedonCellSetIterator;

	int label_;						 // label id of the cell
	double area_;					 // area of the cell, in [pixel^2]
	cv::Rect bounding_box_;			 // bounding box of the cell
	BoustrophedonCellSet neighbors_; // pointer to neighboring cells

	BoustrophedonCell(const int label, const double area, const cv::Rect &bounding_box)
	{
		label_ = label;
		area_ = area;
		bounding_box_ = bounding_box;
	}
};

// Class that generates a room exploration path by using the morse cellular decomposition method, proposed by
//
// "H. Choset, E. Acar, A. A. Rizzi and J. Luntz,
// "Exact cellular decompositions in terms of critical points of Morse functions," Robotics and Automation, 2000. Proceedings.
// ICRA '00. IEEE International Conference on, San Francisco, CA, 2000, pp. 2270-2277 vol.3."
//
// This method decomposes the given environment into several cells which don't have any obstacle in it. For each of this
// cell the boustrophedon path is generated, which goes up and down in each cell, see the upper paper for reference.
// This class only produces a static path, regarding the given map in form of a point series. To react on dynamic
// obstacles, one has to do this in upper algorithms.
//
class BoustrophedonExplorer
{
protected:
	// pathplanner to check for the next nearest locations
	AStarPlanner path_planner_;

	static const uchar BORDER_PIXEL_VALUE = 25;

	// rotates the original map for a good axis alignment and divides it into Morse cells
	// the functions tries two axis alignments with 90deg rotation difference and chooses the one with the lower number of cells
	void findBestCellDecomposition(const cv::Mat &room_map, const float map_resolution, const double min_cell_area,
								   const int min_cell_width, cv::Mat &R, cv::Rect &bbox, cv::Mat &rotated_room_map,
								   std::vector<GeneralizedPolygon> &cell_polygons, std::vector<cv::Point> &polygon_centers);

	// rotates the original map for a good axis alignment and divides it into Morse cells
	// @param rotation_offset can be used to put an offset to the computed rotation for good axis alignment, in [rad]
	void computeCellDecompositionWithRotation(const cv::Mat &room_map, const float map_resolution, const double min_cell_area,
											  const int min_cell_width, const double rotation_offset, cv::Mat &R, cv::Rect &bbox, cv::Mat &rotated_room_map,
											  std::vector<GeneralizedPolygon> &cell_polygons, std::vector<cv::Point> &polygon_centers);

	// divides the provided map into Morse cells
	void computeCellDecomposition(const cv::Mat &room_map, const float map_resolution, const double min_cell_area,
								  const int min_cell_width, std::vector<GeneralizedPolygon> &cell_polygons, std::vector<cv::Point> &polygon_centers);

	// merges cells after a cell decomposition according to various criteria specified in function @mergeCellsSelection
	// returns the number of cells after merging
	vector<int> mergeCells(cv::Mat &cell_map, cv::Mat &cell_map_labels, const double min_cell_area, const int min_cell_width);

	// implements the selection criterion for cell merging, in this case: too small (area) or too thin (width or height) cells
	// are merged with their largest neighboring cell.
	void mergeCellsSelection(cv::Mat &cell_map, cv::Mat &cell_map_labels, std::map<int, boost::shared_ptr<BoustrophedonCell>> &cell_index_mapping,
							 const double min_cell_area, const int min_cell_width);

	// executes the merger of minor cell into major cell
	void mergeTwoCells(cv::Mat &cell_map, cv::Mat &cell_map_labels, const BoustrophedonCell &minor_cell, BoustrophedonCell &major_cell,
					   std::map<int, boost::shared_ptr<BoustrophedonCell>> &cell_index_mapping);

	// this function corrects obstacles that are one pixel width at 45deg angle, i.e. a 2x2 pixel neighborhood with [0, 255, 255, 0] or [255, 0, 0, 255]
	void correctThinWalls(cv::Mat &room_map);

	BoustrophedonGrid getGridLines(cv::Mat &room_map_save,cv::Mat R_cell_inv,cv::Rect bbox,cv::Mat &R_cell_inv_part,const cv::Mat &room_map, const float map_resolution,
								   const GeneralizedPolygon &cell,
								   const int grid_spacing_as_int, const int half_grid_spacing_as_int,
								   const int max_deviation_from_track,
								   const int grid_obstacle_offset,
								   const float min_line_cleaning_distance, const int index_to_boundary);
	int getNextCell(std::vector<cv::Mat>R_cell_inv_vector,std::vector<GeneralizedPolygon> cell_polygons,cv::Point robot_pos, vector<int> &notCompute, vector<BoustrophedonGrid> &grid_lines_total, const float min_line_cleaning_distance,const float map_resolution);

	// computes the Boustrophedon path pattern for a single cell
	void computeBoustrophedonPath(cv::Mat R_cell_inv,BoustrophedonGrid grid_lines, std::vector<int> &firstPointIndex, ccppCellData &cellData, const cv::Mat &room_map, const float map_resolution, const GeneralizedPolygon &cell,
								  cv::Point &robot_pos,
								  const int grid_spacing_as_int, const int half_grid_spacing_as_int, const double path_eps,
								  const int max_deviation_from_track, const int grid_obstacle_offset = 0,
								  const float min_cleaning_distance = 3,
								  const int index_to_boundary = 10);

	// computes the Boustrophedon path pattern for a single cell
	void computeBoustrophedonPath_miss_out(std::vector<missOutPath> &missOutData, const cv::Mat &room_map, const float map_resolution, std::vector<GeneralizedPolygon> cell_polygons,
										   std::vector<int> optimal_order, cv::Point &robot_pos,
										   const int grid_spacing_as_int, const int half_grid_spacing_as_int, const double path_eps,
										   const int max_deviation_from_track, const int grid_obstacle_offset = 0, const float min_cleaning_distance = 5);

	cv::Mat calculate_R_cell_inv_for_each_cell(int cell_index, const cv::Mat &room_map, const float map_resolution, std::vector<GeneralizedPolygon> cell_polygons,
											   std::vector<int> optimal_order,
											   std::vector<cv::Point2f> &fov_middlepoint_path, cv::Point &robot_pos,
											   const int grid_spacing_as_int, const int half_grid_spacing_as_int, const double path_eps,
											   const int max_deviation_from_track, const int grid_obstacle_offset = 0, const float min_cleaning_distance = 5);

	// downsamples a given path original_path to waypoint distances of path_eps and appends the resulting path to downsampled_path
	void downsamplePath(const std::vector<cv::Point> &original_path, std::vector<cv::Point> &downsampled_path,
						cv::Point &cell_robot_pos, const double path_eps);

	// downsamples a given path original_path to waypoint distances of path_eps in reverse order as given in original_path
	// and appends the resulting path to downsampled_path
	void downsamplePathReverse(const std::vector<cv::Point> &original_path, std::vector<cv::Point> &downsampled_path,
							   cv::Point &robot_pos, const double path_eps);

	void printCells(std::map<int, boost::shared_ptr<BoustrophedonCell>> &cell_index_mapping);

public:
	// constructor
	BoustrophedonExplorer();

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const int map_size_x, const int map_size_y, const cv::Mat &room_map, DataCCPP &dataAll, const float map_resolution,
							cv::Point &starting_position, const cv::Point2d map_origin, const double grid_spacing_in_pixel,
							const double grid_obstacle_offset, const double path_eps, int cell_visiting_order, const bool plan_for_footprint,
							const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, const double min_cell_area,
							const int max_deviation_from_track, const float min_line_cleaning_distance,
							int complementary_path_index, int need_hybrid_index, int index_to_boundary, std::string name_path);

	std::vector<ipa_building_msgs::Point_x_y_theta> Get_Hybrid_path(cv::Mat map_expansion, std::vector<double> START, std::vector<double> GOAL);

	enum CellVisitingOrder
	{
		OPTIMAL_TSP = 1,
		LEFT_TO_RIGHT = 2
	};

	// 记录所有的grid_line
	std::vector<std::tuple<bool, int, BoustrophedonLine>> gird_lines_all;
	// 每一个grid_line,对应的 R_cell_inv
	std::vector<std::tuple<int, cv::Mat>> gird_lines_all_R_cell_inv;
	int first_cleaning_line_num = 0;
};

class BoustrophedonVariantExplorer : public BoustrophedonExplorer
{
protected:
	// implements the selection criterion for cell merging, in this case: only large cells with different major axis are not merged.
	void mergeCellsSelection(cv::Mat &cell_map, cv::Mat &cell_map_labels, std::map<int, boost::shared_ptr<BoustrophedonCell>> &cell_index_mapping,
							 const double min_cell_area, const int min_cell_width);

public:
	BoustrophedonVariantExplorer(){};
	~BoustrophedonVariantExplorer(){};
};
