#include "boustrophedon_explorator.h"
#include "geometry/geometry_func.h"
// #include "map/clean_map/clean_map.h"
#include "ccpp_coverage_planner_log.h"

GeneralizedPolygon::GeneralizedPolygon(const std::vector<cv::Point> &vertices, const double map_resolution)
{
	//save given vertexes
	vertices_ = vertices;

	// get max/min x/y coordinates
	max_x_ = 0;
	min_x_ = 100000;
	max_y_ = 0;
	min_y_ = 100000;
	for (size_t point = 0; point < vertices_.size(); ++point)
	{
		if (vertices_[point].x > max_x_)
			max_x_ = vertices_[point].x;
		if (vertices_[point].y > max_y_)
			max_y_ = vertices_[point].y;
		if (vertices_[point].x < min_x_)
			min_x_ = vertices_[point].x;
		if (vertices_[point].y < min_y_)
			min_y_ = vertices_[point].y;
	}

	bounding_box_center_.x = (min_x_ + max_x_) / 2;
	bounding_box_center_.y = (min_y_ + max_y_) / 2;
	// compute visible center
	MeanShift2D ms;
	cv::Mat room = cv::Mat::zeros(max_y_ + 10, max_x_ + 10, CV_8UC1);
	cv::drawContours(room, std::vector<std::vector<cv::Point>>(1, vertices), -1, cv::Scalar(255), CV_FILLED);
	area_ = cv::countNonZero(room);
	cv::Mat distance_map; //variable for the distance-transformed map, type: CV_32FC1
	cv::distanceTransform(room, distance_map, CV_DIST_L2, 5);
	// find point set with largest distance to obstacles
	double min_val = 0., max_val = 0.;
	cv::minMaxLoc(distance_map, &min_val, &max_val);

	std::vector<cv::Vec2d> room_cells;
	for (int v = 0; v < distance_map.rows; ++v)
		for (int u = 0; u < distance_map.cols; ++u)
			if (distance_map.at<float>(v, u) > max_val * 0.95f)
				room_cells.push_back(cv::Vec2d(u, v));

	// use meanshift to find the modes in that set
	// cv::Vec2d room_center = ms.findRoomCenter(room, room_cells, map_resolution);
	// save found center
	// center_.x = room_center[0];
	// center_.y = room_center[1];
	int min_x_temp = 100000;
	int max_x_temp = 0;
	for (size_t point = 0; point < vertices_.size(); ++point)
	{
		if (vertices_[point].y == bounding_box_center_.y)
		{
			if (vertices_[point].x > max_x_temp)
				max_x_temp = vertices_[point].x;
			if (vertices_[point].x < min_x_temp)
				min_x_temp = vertices_[point].x;
		}
	}
	center_.x = (min_x_temp + max_x_temp) / 2;
	center_.y = bounding_box_center_.y;
}

void GeneralizedPolygon::drawPolygon(cv::Mat &image, const cv::Scalar &color) const
{
	// draw polygon in an black image with necessary size
	cv::Mat black_image = cv::Mat(max_y_ + 10, max_x_ + 10, CV_8UC1, cv::Scalar(0));
	// cv::Mat black_image = cv::Mat(1241,845,  CV_8UC1, cv::Scalar(0));

	// std::vector<cv::Point> vertices_rotation = vertices_;
	cv::drawContours(black_image, std::vector<std::vector<cv::Point>>(1, vertices_), -1, color, CV_FILLED);
	// cv::imshow("aaa",black_image);
	// cv::waitKey(1000);
	// assign drawn map
	image = black_image.clone();
}

float wrapMax(float x, float max)
{
	/* integer math: `(max + x % max) % max` */
	return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
float wrapMinMax(float x, float min, float max)
{
	return min + wrapMax(x - min, max - min);
}

float wrapAngle(float yaw)
{
	return wrapMinMax(yaw, -M_PI, M_PI);
}

// Constructor
BoustrophedonExplorer::BoustrophedonExplorer()
{
}

// Function that creates a room exploration path for the given map, by using the morse cellular decomposition method proposed in
//
// "H. Choset, E. Acar, A. A. Rizzi and J. Luntz,
// "Exact cellular decompositions in terms of critical points of Morse functions," Robotics and Automation, 2000. Proceedings.
// ICRA '00. IEEE International Conference on, San Francisco, CA, 2000, pp. 2270-2277 vol.3."
//
// This method takes the given map and separates it into several cells. Each cell is obstacle free and so allows an
// easier path planning. For each cell then a boustrophedon path is planned, which goes up, down and parallel to the
// upper and lower boundaries of the cell, see the referenced paper for details. This function does the following steps:
// I.	Using the Sobel operator the direction of the gradient at each pixel is computed. Using this information, the direction is
//		found that suits best for calculating the cells, i.e. such that longer cells occur, and the map is rotated in this manner.
//		This allows to use the algorithm as it was and in the last step, the found path points simply will be transformed back to the
//		original orientation.
// II.	Sweep a slice (a morse function) trough the given map and check for connectivity of this line,
//		i.e. how many connected segments there are. If the connectivity increases, i.e. more segments appear,
//		an IN event occurs that opens new separate cells, if it decreases, i.e. segments merge, an OUT event occurs that
//		merges two cells together. If an event occurs, the algorithm checks along the current line for critical points,
//		that are points that trigger the events. From these the boundary of the cells are drawn, starting from the CP
//		and going left/right until a black pixel is hit.
// III.	After all cells have been determined by the sweeping slice, the algorithm finds these by using cv::findContours().
//		This gives a set of points for each cell, that are used to create a generalizedPolygon out of each cell.
// IV.	After all polygons have been created, plan the path trough all of them for the field of view s.t. the whole area
//		is covered. To do so, first a global path trough all cells is generated, using the traveling salesmen problem
//		formulation. This produces an optimal visiting order of the cells. Next for each cell a boustrophedon path is
//		determined, which goes back and forth trough the cell and between the horizontal paths along the boundaries of
//		the cell, what ensures that the whole area of the cell is covered. For each cell the longest edge is found and it is transformed
//		s.t. this edge lies horizontal to the x-direction. This produces longer but fewer edges, what improves the path for small but long
//		cells. The startpoint of the cell-path is determined by the endpoint of the previous cell, s.t. the distance between two
//		cell-paths is minimized. The cell-path is determined in the rotated manner, so in a last step, the cell-path is transformed to
//		the originally transformed cell and after that inserted in the global path.
// V.	The previous step produces a path for the field of view. If wanted this path gets mapped to the robot path s.t.
//		the field of view follows the wanted path. To do so simply a vector transformation is applied. If the computed robot
//		pose is not in the free space, another accessible point is generated by finding it on the radius around the field of view (fov)
//		middlepoint s.t. the distance to the last robot position is minimized. If this is not wanted one has to set the
//		corresponding Boolean to false (shows that the path planning should be done for the robot footprint).
// room_map = expects to receive the original, not inflated room map
void BoustrophedonExplorer::getExplorationPath(const int map_size_x, const int map_size_y, const cv::Mat &room_map, DataCCPP &dataAll,
											   const float map_resolution, cv::Point &starting_position, const cv::Point2d map_origin,
											   double grid_spacing_in_pixel, const double grid_obstacle_offset, const double path_eps,
											   int cell_visiting_order,
											   const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fov_vector,
											   const double min_cell_area,
											   const int max_deviation_from_track, const float min_line_cleaning_distance,
											   int complementary_path_index, int need_hybrid_index,
											   int index_to_boundary, string name_path)
{

	// std::cout << "Planning the boustrophedon path trough the room." << std::endl;

	int grid_spacing_as_int = (int)std::floor(grid_spacing_in_pixel);			 
	int half_grid_spacing_as_int = (int)std::floor(0.5 * grid_spacing_in_pixel); 
	int min_cell_width = (half_grid_spacing_as_int + 2. * grid_obstacle_offset / map_resolution) * 5;
	float min_line_cleaning_distance_ = min_line_cleaning_distance;

	//  1. Find the main directions of the map and rotate it in this manner
	//  2. Sweep a slice trough the map and mark the found cell boundaries
	//  3. Find the separated cells
	cv::Mat R; // R表示从persist_map的坐标系装换到主清扫方向坐标系
	cv::Rect bbox;
	cv::Mat rotated_room_map;
	std::vector<GeneralizedPolygon> cell_polygons_raw;
	std::vector<cv::Point> polygon_centers_raw;
	computeCellDecompositionWithRotation(room_map, map_resolution, min_cell_area, min_cell_width, 0., R,
										 bbox, rotated_room_map, cell_polygons_raw, polygon_centers_raw); 

	// std::cout << "Found the cells in the given map. " << std::endl;
	std::vector<GeneralizedPolygon> cell_polygons;
	std::vector<cv::Point> polygon_centers;

	cv::Mat R_cell_inv;
	cv::invertAffineTransform(R, R_cell_inv); // invert the rotation matrix to remap the determined points to the original cell
	for (int cell_index = 0; cell_index < (int)cell_polygons_raw.size(); cell_index++)
	{
		cv::Mat black_image = cv::Mat(room_map.rows, room_map.cols, CV_8UC1, cv::Scalar(0));

		std::vector<cv::Point> vertices_rotation = cell_polygons_raw[cell_index].getVertices();

		cv::transform(vertices_rotation, vertices_rotation, R_cell_inv);

		cv::drawContours(black_image, std::vector<std::vector<cv::Point>>(1, vertices_rotation), -1, cv::Scalar(255), CV_FILLED);
		// std::cout<<"cell_index  "<<cell_index<<"   include  "<<cv::pointPolygonTest(vertices_rotation, rotated_starting_point, false)<<std::endl;
		// assign drawn map
		cv::Mat cell_map_sub = black_image.clone();
		string name = name_path + to_string(cell_index) + ".png";
		imwrite(name, cell_map_sub);
	}
	
	for (int i = 0; i < (int)cell_polygons_raw.size(); i++)
	{
		double area_grid = cell_polygons_raw[i].getArea();
		double area_meter_square = area_grid * map_resolution * map_resolution;
		// std::cout << "area_grid, area_meter_square   " << area_grid << "  " << area_meter_square << endl;
		if (area_meter_square > min_cell_area)
		{
			cell_polygons.push_back(cell_polygons_raw[i]);
			polygon_centers.push_back(polygon_centers_raw[i]);
		}
	}
	// IV. Determine the cell paths.
	// determine the start cell that contains the start position
	std::vector<cv::Point> starting_point_vector(1, starting_position); // opencv

	cv::transform(starting_point_vector, starting_point_vector, R);
	cv::Point rotated_starting_point = starting_point_vector[0]; // point that keeps track of the last point after the boustrophedon path in each cell
	cv::Point robot_pos = rotated_starting_point;				 // point that keeps track of the last point after the boustrophedon path in each cell
	///////////////////////////////////////////////////////////////////////
	// go trough the cells [in optimal visiting order] and determine the boustrophedon paths
	// std::cout << "Starting to get the paths for each cell, number of cells: " << (int)cell_polygons.size() << std::endl;

	// std::cout << "Boustrophedon grid_spacing_as_int=" << grid_spacing_as_int << std::endl;

	map<int, vector<int>> firstPointPerLineIndex;
	vector<BoustrophedonGrid> grid_lines;

	dataAll.ccppData.clear();
	vector<int> notCompute;
	vector<cv::Mat> R_cell_inv_vector;
	cv::Mat room_map_save = room_map.clone();
	for (size_t cell = 0; cell < cell_polygons.size(); ++cell)
	{
		cv::Mat R_cell_inv_part;
		BoustrophedonGrid gridLine = getGridLines(room_map_save, R_cell_inv, bbox, R_cell_inv_part, rotated_room_map, map_resolution, cell_polygons[cell],
												  grid_spacing_as_int, half_grid_spacing_as_int,
												  max_deviation_from_track, grid_obstacle_offset / map_resolution,
												  min_line_cleaning_distance_, index_to_boundary);
		grid_lines.push_back(gridLine);
		notCompute.push_back(cell);
		R_cell_inv_vector.push_back(R_cell_inv_part);
	}

	int numTemp = 0;
	while ((int)notCompute.size() != 0)
	{
		int nextCell = getNextCell(R_cell_inv_vector, cell_polygons, robot_pos, notCompute, grid_lines, min_line_cleaning_distance_, map_resolution);
		if (nextCell != -1)
		{
			vector<int> firstPointIndex;
			std::vector<cv::Point> vertices_rotation = cell_polygons[nextCell].getVertices();
			cv::Mat R_cell_inv;
			cv::invertAffineTransform(R, R_cell_inv);
			cv::transform(vertices_rotation, vertices_rotation, R_cell_inv);
			ccppCellData cellDataTemp;
			cellDataTemp.index = numTemp;
			for (int i = 0; i < (int)vertices_rotation.size(); i++)
				cellDataTemp.vertices.push_back(vertices_rotation[i]);
			dataAll.ccppData.push_back(cellDataTemp);

			computeBoustrophedonPath(R_cell_inv_vector[nextCell], grid_lines[nextCell], firstPointIndex, dataAll.ccppData[numTemp], rotated_room_map, map_resolution, cell_polygons[nextCell],
									 robot_pos, grid_spacing_as_int, half_grid_spacing_as_int, path_eps,
									 max_deviation_from_track, grid_obstacle_offset / map_resolution,
									 min_line_cleaning_distance_, index_to_boundary);
			firstPointPerLineIndex.insert(std::pair<int, vector<int>>((int)numTemp, firstPointIndex));
			numTemp++;
		}
	}

	starting_position.x = robot_pos.x;
	starting_position.y = robot_pos.y;

	std::vector<cv::Point> starting_point_vector2(1, starting_position); // opencv
	cv::transform(starting_point_vector2, starting_point_vector2, R_cell_inv);
	starting_position.x = starting_point_vector2[0].x;
	starting_position.y = starting_point_vector2[0].y;

	std::ofstream savebustrophedon("ccpp_img/map_sub/a.txt");

	cv::Mat fov_path_map_save = room_map.clone();
	RoomRotator room_rotation;

	for (int i = 0; i < (int)dataAll.ccppData.size(); i++)
	{
		std::vector<cv::Point2f> pathBeforeTrans;
		std::vector<ipa_building_msgs::Point_x_y_theta> path_fov_poses;
		for (int j = 0; j < (int)dataAll.ccppData[i].current_path.size(); j++)
		{
			cv::Point2f temp;
			temp.x = dataAll.ccppData[i].current_path[j].pt.x;
			temp.y = dataAll.ccppData[i].current_path[j].pt.y;
			savebustrophedon << temp.y << " " << temp.x << std::endl;
			pathBeforeTrans.push_back(temp);
		}
		if ((int)pathBeforeTrans.size() > 0)
			room_rotation.transformPathBackToOriginalRotation(pathBeforeTrans, path_fov_poses, R);
		for (int j = 0; j < (int)path_fov_poses.size(); j++)
		{
			float x = path_fov_poses[j].x;
			float y = path_fov_poses[j].y;
			cv::Point p2(x, y);
			cv::circle(fov_path_map_save, p2, 0.5, cv::Scalar(100), CV_FILLED);

			dataAll.ccppData[i].current_path[j].pt.x = (map_size_x - 1 - path_fov_poses[j].y) * map_resolution + map_origin.x;
			dataAll.ccppData[i].current_path[j].pt.y = (map_size_y - 1 - path_fov_poses[j].x) * map_resolution + map_origin.y;

			savebustrophedon << path_fov_poses[j].y << " " << path_fov_poses[j].x << std::endl;
			savebustrophedon << dataAll.ccppData[i].current_path[j].pt.x << " " << dataAll.ccppData[i].current_path[j].pt.y << std::endl;
		}

		if ((int)path_fov_poses.size() == 1)
			dataAll.ccppData[i].current_path[0].theta = 0;
		else if ((int)path_fov_poses.size() > 1)
		{
			int lineNum = 1;
			if (lineNum >= (int)firstPointPerLineIndex[i].size())
				lineNum--;
			for (int j = 0; j < (int)path_fov_poses.size(); j++)
			{
				if (j == 0)
					dataAll.ccppData[i].current_path[j].theta = atan2(dataAll.ccppData[i].current_path[j + 1].pt.y - dataAll.ccppData[i].current_path[j].pt.y,
																	  dataAll.ccppData[i].current_path[j + 1].pt.x - dataAll.ccppData[i].current_path[j].pt.x);
				else
				{
					if (lineNum < (int)firstPointPerLineIndex[i].size())
					{
						if (j == firstPointPerLineIndex[i][lineNum])
						{
							// cout << i << " " << lineNum << " " << j << endl;
							dataAll.ccppData[i].current_path[j].theta = atan2(dataAll.ccppData[i].current_path[j + 1].pt.y - dataAll.ccppData[i].current_path[j].pt.y,
																			  dataAll.ccppData[i].current_path[j + 1].pt.x - dataAll.ccppData[i].current_path[j].pt.x);
							lineNum++;
						}
						else
							dataAll.ccppData[i].current_path[j].theta = atan2(dataAll.ccppData[i].current_path[j].pt.y - dataAll.ccppData[i].current_path[j - 1].pt.y,
																			  dataAll.ccppData[i].current_path[j].pt.x - dataAll.ccppData[i].current_path[j - 1].pt.x);
					}
					else
					{
						dataAll.ccppData[i].current_path[j].theta = atan2(dataAll.ccppData[i].current_path[j].pt.y - dataAll.ccppData[i].current_path[j - 1].pt.y,
																		  dataAll.ccppData[i].current_path[j].pt.x - dataAll.ccppData[i].current_path[j - 1].pt.x);
					}
					// cout << dataAll.ccppData[i].current_path[j].theta << endl;
				}
			}
		}
	}

	for (int i = 0; i < (int)dataAll.missOutData.size(); i++)
	{
		std::vector<cv::Point2f> pathBeforeTrans;
		std::vector<ipa_building_msgs::Point_x_y_theta> path_fov_poses;
		for (int j = 0; j < (int)dataAll.missOutData[i].current_path.size(); j++)
		{
			cv::Point2f temp;
			temp.x = dataAll.missOutData[i].current_path[j].pt.x;
			temp.y = dataAll.missOutData[i].current_path[j].pt.y;
			pathBeforeTrans.push_back(temp);
		}
		room_rotation.transformPathBackToOriginalRotation(pathBeforeTrans, path_fov_poses, R);
		for (int j = 0; j < (int)path_fov_poses.size(); j++)
		{
			float x = path_fov_poses[j].x;
			float y = path_fov_poses[j].y;
			cv::Point p2(x, y);
			cv::circle(fov_path_map_save, p2, 0.5, cv::Scalar(100), CV_FILLED);

			dataAll.missOutData[i].current_path[j].pt.x = (map_size_x - 1 - path_fov_poses[j].y) * map_resolution + map_origin.x;
			dataAll.missOutData[i].current_path[j].pt.y = (map_size_y - 1 - path_fov_poses[j].x) * map_resolution + map_origin.y;
			savebustrophedon << dataAll.missOutData[i].current_path[j].pt.x << " " << dataAll.missOutData[i].current_path[j].pt.y << std::endl;
		}
		if ((int)path_fov_poses.size() == 0)
			dataAll.missOutData[i].current_path[0].theta = 0;
		else if ((int)path_fov_poses.size() >= 1)
		{
			for (int j = 0; j < (int)path_fov_poses.size(); j++)
			{
				if (j == 0)
					dataAll.missOutData[i].current_path[j].theta = atan2(dataAll.missOutData[i].current_path[j + 1].pt.y - dataAll.missOutData[i].current_path[j].pt.y,
																		 dataAll.missOutData[i].current_path[j + 1].pt.x - dataAll.missOutData[i].current_path[j].pt.x);
				else
					dataAll.missOutData[i].current_path[j].theta = atan2(dataAll.missOutData[i].current_path[j].pt.y - dataAll.missOutData[i].current_path[j - 1].pt.y,
																		 dataAll.missOutData[i].current_path[j].pt.x - dataAll.missOutData[i].current_path[j - 1].pt.x);
			}
		}
	}
	string name_save = name_path + "_result_" + ".png";
	imwrite(name_save, fov_path_map_save);

	return;
}

void BoustrophedonExplorer::computeCellDecompositionWithRotation(const cv::Mat &room_map, const float map_resolution, const double min_cell_area,
																 const int min_cell_width, const double rotation_offset, cv::Mat &R, cv::Rect &bbox, cv::Mat &rotated_room_map,
																 std::vector<GeneralizedPolygon> &cell_polygons, std::vector<cv::Point> &polygon_centers)
{
	// I. Find the main directions of the map and rotate it in this manner
	RoomRotator room_rotation;
	room_rotation.computeRoomRotationMatrix(room_map, R, bbox, map_resolution, 0, rotation_offset);
	room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);									

	// II. Sweep a slice trough the map and mark the found cell boundaries
	// III. Find the separated cells
	computeCellDecomposition(rotated_room_map, map_resolution, min_cell_area, min_cell_width, cell_polygons, polygon_centers);
}

void BoustrophedonExplorer::computeCellDecomposition(const cv::Mat &room_map, const float map_resolution, const double min_cell_area,
													 const int min_cell_width, std::vector<GeneralizedPolygon> &cell_polygons, std::vector<cv::Point> &polygon_centers)
{
	// II. Sweep a slice trough the map and mark the found cell boundaries
	// create a map copy to mark the cell boundaries
	cv::Mat cell_map = room_map.clone(); 
	string SaveFileImg = "1.png";
	cv::imwrite(SaveFileImg, cell_map);

	// find smallest y-value for that a white pixel occurs, to set initial y value and find initial number of segments
	// room_map.at<uchar>(y, x) == 0,
	int y_start = 0;
	bool found = false;
	bool obstacle = false;
	int previous_number_of_segments = 0;
	std::vector<int> previous_obstacles_end_x; // keep track of the end points of obstacles
	for (int y = 0; y < room_map.rows; ++y)	
	{
		for (int x = 0; x < room_map.cols; ++x)
		{
			if (found == false && room_map.at<uchar>(y, x) == 255)
			{
				y_start = y;
				found = true;
			}
			else if (found == true && obstacle == false && room_map.at<uchar>(y, x) == 0)
			{
				++previous_number_of_segments;
				obstacle = true;
			}
			else if (found == true && obstacle == true && room_map.at<uchar>(y, x) == 255)
			{
				obstacle = false;
				previous_obstacles_end_x.push_back(x);
			}
		}
		if (found == true)
			break;
	}

	// sweep trough the map and detect critical points
	for (int y = y_start + 1; y < room_map.rows; ++y) // start at y_start+1 because we know number of segments at y_start
	{												
		int number_of_segments = 0;					  // int to count how many segments at the current slice are
		std::vector<int> current_obstacles_start_x;
		std::vector<int> current_obstacles_end_x;
		bool obstacle_hit = false;	  // bool to check if the line currently hit an obstacle, s.t. not all black pixels trigger an event
		bool hit_white_pixel = false; // bool to check if a white pixel has been hit at the current slice, to start the slice at the first white pixel

		// count number of segments within this row
		for (int x = 0; x < room_map.cols; ++x)
		{
			if (hit_white_pixel == false && room_map.at<uchar>(y, x) == 255)
				hit_white_pixel = true;
			else if (hit_white_pixel == true)
			{
				if (obstacle_hit == false && room_map.at<uchar>(y, x) == 0) // check for obstacle
				{
					++number_of_segments;
					obstacle_hit = true;
					current_obstacles_start_x.push_back(x);
				}
				else if (obstacle_hit == true && room_map.at<uchar>(y, x) == 255) // check for leaving obstacle
				{
					obstacle_hit = false;
					current_obstacles_end_x.push_back(x);
				}
			}
		}

		// if the number of segments did not change, check whether the position of segments has changed so that there is a gap between them
		bool segment_shift_detected = false; 
		if (previous_number_of_segments == number_of_segments && current_obstacles_start_x.size() == previous_obstacles_end_x.size() + 1)
		{
			for (size_t i = 0; i < previous_obstacles_end_x.size(); ++i)
				if (current_obstacles_start_x[i] > previous_obstacles_end_x[i])
				{
					segment_shift_detected = true;
					break;
				}
		}

		// reset hit_white_pixel to use this Boolean later
		hit_white_pixel = false;

		// check if number of segments has changed --> event occurred
		if (previous_number_of_segments < number_of_segments || segment_shift_detected == true) // IN event (or shift)
		{																						
			// check the current slice again for critical points
			for (int x = 0; x < room_map.cols; ++x)
			{
				if (hit_white_pixel == false && room_map.at<uchar>(y, x) == 255) 
					hit_white_pixel = true;
				else if (hit_white_pixel == true && room_map.at<uchar>(y, x) == 0) 
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for (int dx = -1; dx <= 1; ++dx) 
						if (room_map.at<uchar>(y - 1, std::max(0, std::min(x + dx, room_map.cols - 1))) == 0)
							critical_point = false;

					// if a critical point is found mark the separation, note that this algorithm goes left and right
					// starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
					// behind other obstacles on the same y-value as the critical point
					if (critical_point == true) 
					{
						// to the left until a black pixel is hit
						for (int dx = -1; x + dx >= 0; --dx)
						{
							uchar &val = cell_map.at<uchar>(y, x + dx);
							if (val == 255 && cell_map.at<uchar>(y - 1, x + dx) == 255)
								val = BORDER_PIXEL_VALUE;
							else if (val == 0)
								break;
						}

						// to the right until a black pixel is hit
						for (int dx = 1; x + dx < room_map.cols; ++dx)
						{
							uchar &val = cell_map.at<uchar>(y, x + dx);
							if (val == 255 && cell_map.at<uchar>(y - 1, x + dx) == 255)
								val = BORDER_PIXEL_VALUE;
							else if (val == 0)
								break;
						}
					}
				}
			}
		}
		else if (previous_number_of_segments > number_of_segments) // OUT event
		{														   
			// check the previous slice again for critical points --> y-1
			for (int x = 0; x < room_map.cols; ++x)
			{
				if (room_map.at<uchar>(y - 1, x) == 255 && hit_white_pixel == false) 
					hit_white_pixel = true;
				else if (hit_white_pixel == true && room_map.at<uchar>(y - 1, x) == 0)
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for (int dx = -1; dx <= 1; ++dx)
						if (room_map.at<uchar>(y, std::max(0, std::min(x + dx, room_map.cols - 1))) == 0) // check at side after obstacle
							critical_point = false;

					// if a critical point is found mark the separation, note that this algorithm goes left and right
					// starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
					// behind other obstacles on the same y-value as the critical point
					if (critical_point == true) 
					{
						const int ym2 = std::max(0, (int)y - 2);

						// to the left until a black pixel is hit
						for (int dx = -1; x + dx >= 0; --dx)
						{
							uchar &val = cell_map.at<uchar>(y - 1, x + dx);
							if (val == 255 && cell_map.at<uchar>(ym2, x + dx) == 255)
								val = BORDER_PIXEL_VALUE;
							else if (val == 0)
								break;
						}

						// to the right until a black pixel is hit
						for (int dx = 1; x + dx < room_map.cols; ++dx)
						{
							uchar &val = cell_map.at<uchar>(y - 1, x + dx);
							if (val == 255 && cell_map.at<uchar>(ym2, x + dx) == 255)
								val = BORDER_PIXEL_VALUE;
							else if (val == 0)
								break;
						}
					}
				}
			}
		}
		// save the found number of segments and the obstacle end points
		previous_number_of_segments = number_of_segments;
		previous_obstacles_end_x = current_obstacles_end_x;
	}
	// cv::imshow("cell_map", cell_map);
	string SaveFileImg2 = "2.png";
	cv::imwrite(SaveFileImg2, cell_map); 
	// II.b) merge too small cells into bigger cells

	cv::Mat cell_map_labels; //分区主函数
	vector<int> mergeData = mergeCells(cell_map, cell_map_labels, min_cell_area, min_cell_width);

	// std::cout << "end merge " << (int)mergeData.size() << std::endl;

	// III. Find the separated cells
	std::vector<std::vector<cv::Point>> cells;
	for (int i = 0; i < (int)mergeData.size(); i++)
	{
		cv::Mat cell_copy(cell_map_labels == mergeData[i]);
		std::vector<std::vector<cv::Point>> cellsi;
		// cv::findContours(cell_copy, cellsi, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		cv::findContours(cell_copy, cellsi, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		cells.insert(cells.end(), cellsi.begin(), cellsi.end());
	}
	// create generalized Polygons out of the contours to handle the cells
	for (size_t cell = 0; cell < cells.size(); ++cell)
	{
		// cout << "percent:" << (cell * 100) / (float)cells.size() << "%" << endl;
		CCPP_INFO_LOG("percent: %lf%", (cell * 100) / (float)cells.size());
		GeneralizedPolygon current_cell(cells[cell], map_resolution);
		if (current_cell.getArea() >= min_cell_area)
		{
			cell_polygons.push_back(current_cell);
			polygon_centers.push_back(current_cell.getCenter());
		}
	}
}

vector<int> BoustrophedonExplorer::mergeCells(cv::Mat &cell_map, cv::Mat &cell_map_labels, const double min_cell_area, const int min_cell_width)
{
	// label all cells
	//   --> create a label map with 0=walls/obstacles, -1=cell borders, 1,2,3,4...=cell labels
	cell_map.convertTo(cell_map_labels, CV_32SC1, 256, 0); 
	//   --> re-assign the cell borders with -1
	for (int v = 0; v < cell_map_labels.rows; ++v)
		for (int u = 0; u < cell_map_labels.cols; ++u)
			if (cell_map_labels.at<int>(v, u) == BORDER_PIXEL_VALUE * 256)
				cell_map_labels.at<int>(v, u) = -1;
	//   --> flood fill cell regions with unique id labels
	std::map<int, boost::shared_ptr<BoustrophedonCell>> cell_index_mapping; // maps each cell label --> to the cell object
	int label_index = 1;													
	for (int v = 0; v < cell_map_labels.rows; ++v)							
	{
		for (int u = 0; u < cell_map_labels.cols; ++u)
		{
			// if the map has already received a label for that pixel --> skip
			if (cell_map_labels.at<int>(v, u) != 65280)
				continue;

			// fill each cell with a unique id
			cv::Rect bounding_box; 
			const double area = cv::floodFill(cell_map_labels, cv::Point(u, v), label_index, &bounding_box, 0, 0, 4);
			cell_index_mapping[label_index] = boost::shared_ptr<BoustrophedonCell>(new BoustrophedonCell(label_index, area, bounding_box));
			label_index++;
			// if (label_index == INT_MAX)
			// 	std::cout << "WARN: BoustrophedonExplorer::mergeCells: label_index exceeds range of int." << std::endl;
		}
	}
	// std::cout << "INFO: BoustrophedonExplorer::mergeCells: found " << label_index - 1 << " cells before merging." << std::endl;
		CCPP_INFO_LOG("INFO: BoustrophedonExplorer::mergeCells: found : %ld cells before merging.",label_index - 1);

	// determine the neighborhood relationships between all cells
	for (int v = 1; v < cell_map_labels.rows - 1; ++v) 
	{
		for (int u = 1; u < cell_map_labels.cols - 1; ++u)
		{
			if (cell_map_labels.at<int>(v, u) == -1) // only check the border points for neighborhood relationships
			{
				const int label_left = cell_map_labels.at<int>(v, u - 1);
				const int label_right = cell_map_labels.at<int>(v, u + 1);
				if (label_left > 0 && label_right > 0)
				{
					cell_index_mapping[label_left]->neighbors_.insert(cell_index_mapping[label_right]);
					cell_index_mapping[label_right]->neighbors_.insert(cell_index_mapping[label_left]);
				}
				const int label_up = cell_map_labels.at<int>(v - 1, u);
				const int label_down = cell_map_labels.at<int>(v + 1, u);
				if (label_up > 0 && label_down > 0)
				{
					cell_index_mapping[label_up]->neighbors_.insert(cell_index_mapping[label_down]);
					cell_index_mapping[label_down]->neighbors_.insert(cell_index_mapping[label_up]);
				}
			}
		}
	}

	// iteratively merge cells
	mergeCellsSelection(cell_map, cell_map_labels, cell_index_mapping, min_cell_area, min_cell_width); 
	// re-assign area labels to 1,2,3,4,...
	int new_cell_label = 1; 

	for (std::map<int, boost::shared_ptr<BoustrophedonCell>>::iterator itc = cell_index_mapping.begin(); itc != cell_index_mapping.end(); ++itc, ++new_cell_label)
	{
		for (int v = 0; v < cell_map_labels.rows; ++v)
			for (int u = 0; u < cell_map_labels.cols; ++u)
				if (cell_map_labels.at<int>(v, u) == itc->second->label_)
					cell_map_labels.at<int>(v, u) = new_cell_label;
	}

	vector<int> cellData;
	for (int v = 0; v < cell_map_labels.rows; ++v)
	{
		for (int u = 0; u < cell_map_labels.cols; ++u)
		{
			if (cell_map_labels.at<int>(v, u) != 0)
			{
				if ((int)cellData.size() != 0)
				{
					for (int i = 0; i < (int)cellData.size(); i++)
					{
						if (cell_map_labels.at<int>(v, u) == cellData[i])
							break;
						if (i == (int)cellData.size() - 1)
							cellData.push_back(cell_map_labels.at<int>(v, u));
					}
				}
				else
					cellData.push_back(cell_map_labels.at<int>(v, u));
			}
		}
	}
	// std::cout << "INFO: BoustrophedonExplorer::mergeCells: " << (int)cellData.size() << " cells remaining after merging." << std::endl;
		CCPP_INFO_LOG("INFO: BoustrophedonExplorer::mergeCells: %ld cells remaining after merging.",(int)cellData.size());

	return cellData;
}

void BoustrophedonExplorer::mergeCellsSelection(cv::Mat &cell_map, cv::Mat &cell_map_labels, std::map<int, boost::shared_ptr<BoustrophedonCell>> &cell_index_mapping,
												const double min_cell_area, const int min_cell_width)
{
	float num = 0;
	float numSum = (int)cell_index_mapping.size();
	for (std::map<int, boost::shared_ptr<BoustrophedonCell>>::iterator it = cell_index_mapping.begin(); it != cell_index_mapping.end();)
	{
		if (it->second->area_ >= min_cell_area && it->second->bounding_box_.width >= min_cell_width && it->second->bounding_box_.height >= min_cell_width)
		{
			++it;
			continue;
		}

		// skip segments which have no neighbors
		if (it->second->neighbors_.size() == 0)
		{
			++it;
			continue;
		}
		// std::cout << "merge_percent:" << num / numSum * 100.0 << "%" << std::endl;
		CCPP_INFO_LOG("merge_percent: %ld %.",num / numSum * 100.0);


		num = num + 1;
		BoustrophedonCell &small_cell = *(it->second);

		// determine the largest neighboring cell
		std::multimap<double, boost::shared_ptr<BoustrophedonCell>, std::greater<double>> area_sorted_neighbors;
		for (BoustrophedonCell::BoustrophedonCellSetIterator itn = it->second->neighbors_.begin(); itn != it->second->neighbors_.end(); ++itn)
		{
			area_sorted_neighbors.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell>>((*itn)->area_, *itn));
		}
		BoustrophedonCell &large_cell = *(area_sorted_neighbors.begin()->second);
		mergeTwoCells(cell_map, cell_map_labels, small_cell, large_cell, cell_index_mapping); //合并进去

		it = cell_index_mapping.begin();
	}

	// label remaining border pixels with label of largest neighboring region label
	for (int v = 1; v < cell_map.rows - 1; ++v) 
	{
		for (int u = 1; u < cell_map.cols - 1; ++u)
		{
			if (cell_map.at<uchar>(v, u) == BORDER_PIXEL_VALUE)
			{
				std::set<int> neighbor_labels;
				for (int dv = -1; dv <= 1; ++dv)
				{
					for (int du = -1; du <= 1; ++du)
					{
						const int &val = cell_map_labels.at<int>(v + dv, u + du);
						if (val > 0)
							neighbor_labels.insert(val);
					}
				}
				if (neighbor_labels.size() > 0)
				{
					int new_label = -1;
					for (std::map<int, boost::shared_ptr<BoustrophedonCell>>::reverse_iterator it = cell_index_mapping.rbegin(); it != cell_index_mapping.rend(); ++it)
					{
						if (neighbor_labels.find(it->second->label_) != neighbor_labels.end())
						{
							cell_map_labels.at<int>(v, u) = it->second->label_;
							break;
						}
					}
				}
				// else
				// 	std::cout << "WARN: BoustrophedonExplorer::mergeCells: border pixel has no labeled neighbors." << std::endl;
			}
		}
	}
}

void BoustrophedonExplorer::mergeTwoCells(cv::Mat &cell_map, cv::Mat &cell_map_labels, const BoustrophedonCell &minor_cell, BoustrophedonCell &major_cell,
										  std::map<int, boost::shared_ptr<BoustrophedonCell>> &cell_index_mapping)
{
	// execute merging the minor cell into the major cell
	//   --> remove border from maps
	for (int v = 0; v < cell_map.rows; ++v)
		for (int u = 0; u < cell_map.cols; ++u)
			if (cell_map.at<uchar>(v, u) == BORDER_PIXEL_VALUE &&
				((cell_map_labels.at<int>(v, u - 1) == minor_cell.label_ && cell_map_labels.at<int>(v, u + 1) == major_cell.label_) ||
				 (cell_map_labels.at<int>(v, u - 1) == major_cell.label_ && cell_map_labels.at<int>(v, u + 1) == minor_cell.label_) ||
				 (cell_map_labels.at<int>(v - 1, u) == minor_cell.label_ && cell_map_labels.at<int>(v + 1, u) == major_cell.label_) ||
				 (cell_map_labels.at<int>(v - 1, u) == major_cell.label_ && cell_map_labels.at<int>(v + 1, u) == minor_cell.label_)))
			{
				cell_map.at<uchar>(v, u) = 255;
				cell_map_labels.at<int>(v, u) = major_cell.label_;
				major_cell.area_ += 1;
			}
	//   --> update old label in cell_map_labels
	for (int v = 0; v < cell_map_labels.rows; ++v)
		for (int u = 0; u < cell_map_labels.cols; ++u)
			if (cell_map_labels.at<int>(v, u) == minor_cell.label_)
				cell_map_labels.at<int>(v, u) = major_cell.label_;
	//   --> update major_cell
	major_cell.area_ += minor_cell.area_;
	for (BoustrophedonCell::BoustrophedonCellSetIterator itn = major_cell.neighbors_.begin(); itn != major_cell.neighbors_.end(); ++itn)
		if ((*itn)->label_ == minor_cell.label_)
		{
			major_cell.neighbors_.erase(itn);
			break;
		}
	for (BoustrophedonCell::BoustrophedonCellSetIterator itn = minor_cell.neighbors_.begin(); itn != minor_cell.neighbors_.end(); ++itn)
		if ((*itn)->label_ != major_cell.label_)
			major_cell.neighbors_.insert(*itn);

	// clean all references to minor_cell
	cell_index_mapping.erase(minor_cell.label_);
	for (std::map<int, boost::shared_ptr<BoustrophedonCell>>::iterator itc = cell_index_mapping.begin(); itc != cell_index_mapping.end(); ++itc)
		for (BoustrophedonCell::BoustrophedonCellSetIterator itn = itc->second->neighbors_.begin(); itn != itc->second->neighbors_.end(); ++itn)
			if ((*itn)->label_ == minor_cell.label_)
			{
				(*itn)->label_ = major_cell.label_;
				break;
			}
}

void BoustrophedonExplorer::correctThinWalls(cv::Mat &room_map)
{
	for (int v = 1; v < room_map.rows; ++v)
	{
		for (int u = 1; u < room_map.cols; ++u)
		{
			if (room_map.at<uchar>(v - 1, u - 1) == 255 && room_map.at<uchar>(v - 1, u) == 0 && room_map.at<uchar>(v, u - 1) == 0 && room_map.at<uchar>(v, u) == 255)
				room_map.at<uchar>(v, u) = 0;
			else if (room_map.at<uchar>(v - 1, u - 1) == 0 && room_map.at<uchar>(v - 1, u) == 255 && room_map.at<uchar>(v, u - 1) == 255 && room_map.at<uchar>(v, u) == 0)
				room_map.at<uchar>(v, u - 1) = 0;
		}
	}
}
BoustrophedonGrid BoustrophedonExplorer::getGridLines(cv::Mat &room_map_save, cv::Mat R_cell_inv, cv::Rect bbox, cv::Mat &R_cell_inv_part, const cv::Mat &room_map, const float map_resolution,
													  const GeneralizedPolygon &cell,
													  const int grid_spacing_as_int, const int half_grid_spacing_as_int,
													  const int max_deviation_from_track,
													  const int grid_obstacle_offset,
													  const float min_line_cleaning_distance, const int index_to_boundary)
{
	// get a map that has only the current cell drawn in
	//	Remark:	single cells are obstacle free so it is sufficient to use the cell to check
	// if a position can be reached during the
	//			execution of the coverage path
	cv::Mat cell_map;
	cell.drawPolygon(cell_map, cv::Scalar(255));

	// align the longer dimension of the cell horizontally with the x-axis
	cv::Point cell_center = cell.getBoundingBoxCenter(); //中心点
	cv::Mat R_cell;
	cv::Rect cell_bbox;
	cv::Mat rotated_cell_map;
	RoomRotator cell_rotation;

	// compute the affine rotation matrix for rotating a room into parallel alignment with x-axis (longer side of the room is aligned with x-axis)
	// R is the transform
	// bounding_rect is the ROI of the warped image
	cell_rotation.computeRoomRotationMatrix(cell_map, R_cell, cell_bbox, map_resolution, &cell_center);
	cell_rotation.rotateRoom(cell_map, rotated_cell_map, R_cell, cell_bbox);
	cv::invertAffineTransform(R_cell, R_cell_inv_part);
	// create inflated obstacles room map and rotate according to cell
	//  --> used later for checking accessibility of Boustrophedon path inside the cell
	cv::Mat inflated_room_map, rotated_inflated_room_map, cellMapSaveFile;

	// cv::imshow("a",cell_map);
	// cv::waitKey(100000);
	cell_rotation.rotateRoom(cell_map, rotated_inflated_room_map, R_cell, cell_bbox);
	// float sideSafeDistance = 1.5 + 0.2 - index_to_boundary * 0.05;
	// int erodeMain = 0.5/map_resolution * 2 + 1;
	// int erodeSecond = erodeMain + index_to_boundary + half_grid_spacing_as_int * 2 + 1;
	int erodeMain = 0.3 / map_resolution * 2 + 1;
	int erodeSecond = 1;
	cv::morphologyEx(rotated_inflated_room_map, rotated_inflated_room_map, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erodeMain, erodeSecond)));

	erodeMain = 1.2 / map_resolution * 2 + 1;
	cv::morphologyEx(rotated_inflated_room_map, cellMapSaveFile, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erodeMain, erodeSecond)));
	cell_rotation.rotateRoom(cellMapSaveFile, cellMapSaveFile, R_cell_inv_part, cell_bbox);
	cell_rotation.rotateRoom(cellMapSaveFile, cellMapSaveFile, R_cell_inv, bbox);

	for (int i = 0; i < cellMapSaveFile.rows; i++)
	{
		for (int j = 0; j < cellMapSaveFile.cols; j++)
		{
			if (cellMapSaveFile.at<unsigned char>(j, i) != 0)
				room_map_save.at<unsigned char>(j, i) = 128;
		}
	}

	cv::Mat rotated_inflated_cell_map = rotated_cell_map.clone();
	for (int v = 0; v < rotated_inflated_cell_map.rows; ++v)
		for (int u = 0; u < rotated_inflated_cell_map.cols; ++u)
			if (rotated_inflated_cell_map.at<uchar>(v, u) != 0 && rotated_inflated_room_map.at<uchar>(v, u) == 0)
			{
				rotated_inflated_cell_map.at<uchar>(v, u) = 128;
			}
	// cv::imshow("a",rotated_inflated_cell_map);
	// cv::waitKey(100000);
	// step 1: 
	// compute the basic Boustrophedon grid lines
	BoustrophedonGrid grid_lines;
	// cout << "grid_spacing_as_int:" << grid_spacing_as_int << " half_grid_spacing_as_int:" << half_grid_spacing_as_int << endl;
	GridGenerator::generateBoustrophedonGrid(min_line_cleaning_distance, map_resolution, rotated_cell_map, rotated_inflated_cell_map, -1, grid_lines, cv::Vec4i(-1, -1, -1, -1), //cv::Vec4i(min_x, max_x, min_y, max_y),
											 grid_spacing_as_int, half_grid_spacing_as_int, 1, max_deviation_from_track, index_to_boundary);
	for (int i = 0; i < grid_lines.size(); i++)
	{
		for (int j = 0; j < grid_lines[i].upper_line.size(); j++)
		{
			int col = grid_lines[i].upper_line[j].x;
			int row = grid_lines[i].upper_line[j].y;

			rotated_cell_map.at<unsigned char>(row, col) = 128;
		}
	}

	return grid_lines;
}
int BoustrophedonExplorer::getNextCell(std::vector<cv::Mat> R_cell_inv_vector, std::vector<GeneralizedPolygon> cell_polygons, cv::Point robot_pos, vector<int> &notCompute, vector<BoustrophedonGrid> &grid_lines_total, const float min_line_cleaning_distance, const float map_resolution)
{
	int nextCellIndex = -1;
	int nextNotComputeIndex = -1;
	float minDistance = 10000000;
	float distanceLastTemp = 0;
	float distance0Temp = 0;
	for (int i = 0; i < (int)notCompute.size(); i++)
	{
		cv::Mat R_cell_inv = R_cell_inv_vector[notCompute[i]];
		for (int j = 0; j < (int)grid_lines_total[notCompute[i]].size(); ++j)
		{
			int start_x = grid_lines_total[notCompute[i]][j].upper_line.front().x;
			int start_y = grid_lines_total[notCompute[i]][j].upper_line.front().y;
			int goal_x = grid_lines_total[notCompute[i]][j].upper_line.back().x;
			int goal_y = grid_lines_total[notCompute[i]][j].upper_line.back().y;

			float distance = map_resolution * hypotf((goal_y - start_y), (goal_x - start_x));
			if (distance < min_line_cleaning_distance)
			{
				grid_lines_total[notCompute[i]].erase(grid_lines_total[notCompute[i]].begin() + j);
				j = max(0, j - 1);
			}
		}

		if ((int)grid_lines_total[notCompute[i]].size() == 0)
		{
			notCompute.erase(notCompute.begin() + i);
			i = max(0, i - 1);
			continue;
		}

		cv::Point point_start_0 = grid_lines_total[notCompute[i]].front().upper_line.front();
		cv::Point point_end_0 = grid_lines_total[notCompute[i]].front().upper_line.back();
		cv::Point point_start_last = grid_lines_total[notCompute[i]].back().upper_line.front();
		cv::Point point_end_last = grid_lines_total[notCompute[i]].back().upper_line.back();

		vector<cv::Point> point;
		point.push_back(point_start_0);
		point.push_back(point_end_0);
		point.push_back(point_start_last);
		point.push_back(point_end_last);

		cv::transform(point, point, R_cell_inv);
		point_start_0 = point[0];
		point_end_0 = point[1];
		point_start_last = point[2];
		point_end_last = point[3];

		float distance_start_0 = map_resolution * hypotf((robot_pos.y - point_start_0.y), (robot_pos.x - point_start_0.x));
		float distance_end_0 = map_resolution * hypotf((robot_pos.y - point_end_0.y), (robot_pos.x - point_end_0.x));
		float distance_0 = min(distance_start_0, distance_end_0);

		float distance_start_last = map_resolution * hypotf((robot_pos.y - point_start_last.y), (robot_pos.x - point_start_last.x));
		float distance_end_last = map_resolution * hypotf((robot_pos.y - point_end_last.y), (robot_pos.x - point_end_last.x));
		float distance_last = min(distance_start_last, distance_end_last);
		if (distance_last < minDistance || distance_0 < minDistance)
		{
			minDistance = min(distance_last, distance_0);
			nextCellIndex = notCompute[i];
			nextNotComputeIndex = i;
			distanceLastTemp = distance_last;
			distance0Temp = distance_0;
		}
	}

	if (nextCellIndex != -1)
	{
		if (distanceLastTemp < distance0Temp)
		{
			std::reverse(grid_lines_total[nextCellIndex].begin(), grid_lines_total[nextCellIndex].end());
		}
		notCompute.erase(notCompute.begin() + nextNotComputeIndex);
	}

	return nextCellIndex;
}

void BoustrophedonExplorer::computeBoustrophedonPath(cv::Mat R_cell_inv, BoustrophedonGrid grid_lines, std::vector<int> &firstPointIndex, ccppCellData &cellData, const cv::Mat &room_map, const float map_resolution,
													 const GeneralizedPolygon &cell,
													 cv::Point &robot_pos,
													 const int grid_spacing_as_int, const int half_grid_spacing_as_int,
													 const double path_eps, const int max_deviation_from_track,
													 const int grid_obstacle_offset,
													 const float min_line_cleaning_distance, const int index_to_boundary)
{
	std::vector<cv::Point> current_fov_path;
	current_fov_path.clear();

	const int lineNumTotal = (int)grid_lines.size();
	int sortLine[lineNumTotal];

	std::vector<std::pair<int, int>> sort;

	cv::Point point_start;
	cv::Point point_end;
	cv::Point point_start_next;
	cv::Point point_end_next;
	std::pair<int, int> temp;
	temp.first = 0;
	temp.second = 0;
	if (lineNumTotal > 0)
	{
		point_start = grid_lines[0].upper_line.front();
		point_end = grid_lines[0].upper_line.back();
	}
	for (int i = 0; i < lineNumTotal; i++)
	{
		point_start_next = grid_lines[i].upper_line.front();
		point_end_next = grid_lines[i].upper_line.back();
		if (i == lineNumTotal - 1)
		{
			float disStart = map_resolution * hypotf((point_start_next.y - point_start.y), (point_start_next.x - point_start.x));
			float disEnd = map_resolution * hypotf((point_end_next.y - point_end.y), (point_end_next.x - point_end.x));
			if (disStart > 1 || disEnd > 1)
			{
				sort.push_back(temp);
				temp.first = i;
				temp.second = i;
				sort.push_back(temp);
			}
			else
			{
				temp.second = i;
				sort.push_back(temp);
			}
		}
		else
		{
			float disStart = map_resolution * hypotf((point_start_next.y - point_start.y), (point_start_next.x - point_start.x));
			float disEnd = map_resolution * hypotf((point_end_next.y - point_end.y), (point_end_next.x - point_end.x));
			if (disStart > 1 || disEnd > 1)
			{
				sort.push_back(temp);
				temp.first = i;
				temp.second = i;
			}
			else
			{
				temp.second = i;
			}
			point_start = point_start_next;
			point_end = point_end_next;
		}
	}

	for (int i = 0; i < (int)sort.size(); i++)
	{
		int startSort = sort[i].first;
		int endSort = sort[i].second;
		int lineNum = endSort - startSort + 1;

		if (lineNum == 1)
			sortLine[startSort + 0] = startSort + 0;
		if (lineNum == 2)
		{
			sortLine[startSort + 0] = startSort + 0;
			sortLine[startSort + 1] = startSort + 1;
		}
		if (lineNum == 3)
		{
			sortLine[startSort + 0] = startSort + 0;
			sortLine[startSort + 1] = startSort + 2;
			sortLine[startSort + 2] = startSort + 1;
		}
		if (lineNum == 4)
		{
			sortLine[startSort + 0] = startSort + 0;
			sortLine[startSort + 1] = startSort + 3;
			sortLine[startSort + 2] = startSort + 1;
			sortLine[startSort + 3] = startSort + 2;
		}
		if (lineNum >= 5)
		{
			int fiveNum = 0;
			int sevenNum = 0;
			int nineNum = 0;
			int oneNum = 0;
			fiveNum = lineNum / 5 - 1;
			int remainNum = lineNum - fiveNum * 5;
			if (remainNum == 5)
				fiveNum = fiveNum + 1;
			if (remainNum == 6)
			{
				fiveNum = fiveNum + 1;
				oneNum = 1;
			}
			if (remainNum == 7)
				sevenNum = 1;
			if (remainNum == 8)
			{
				sevenNum = 1;
				oneNum = 1;
			}
			if (remainNum == 9)
				nineNum = 1;

			for (int i = 0; i < fiveNum; i++)
			{
				sortLine[startSort + i * 5 + 0] = startSort + i * 5 + 0;
				sortLine[startSort + i * 5 + 1] = startSort + i * 5 + 3;
				sortLine[startSort + i * 5 + 2] = startSort + i * 5 + 1;
				sortLine[startSort + i * 5 + 3] = startSort + i * 5 + 4;
				sortLine[startSort + i * 5 + 4] = startSort + i * 5 + 2;
			}
			if (sevenNum == 1)
			{
				sortLine[startSort + fiveNum * 5 + 0] = startSort + fiveNum * 5 + 0;
				sortLine[startSort + fiveNum * 5 + 1] = startSort + fiveNum * 5 + 4;
				sortLine[startSort + fiveNum * 5 + 2] = startSort + fiveNum * 5 + 1;
				sortLine[startSort + fiveNum * 5 + 3] = startSort + fiveNum * 5 + 5;
				sortLine[startSort + fiveNum * 5 + 4] = startSort + fiveNum * 5 + 2;
				sortLine[startSort + fiveNum * 5 + 5] = startSort + fiveNum * 5 + 6;
				sortLine[startSort + fiveNum * 5 + 6] = startSort + fiveNum * 5 + 3;
			}
			if (nineNum == 1)
			{
				sortLine[startSort + fiveNum * 5 + 0] = startSort + fiveNum * 5 + 0;
				sortLine[startSort + fiveNum * 5 + 1] = startSort + fiveNum * 5 + 5;
				sortLine[startSort + fiveNum * 5 + 2] = startSort + fiveNum * 5 + 1;
				sortLine[startSort + fiveNum * 5 + 3] = startSort + fiveNum * 5 + 6;
				sortLine[startSort + fiveNum * 5 + 4] = startSort + fiveNum * 5 + 2;
				sortLine[startSort + fiveNum * 5 + 5] = startSort + fiveNum * 5 + 7;
				sortLine[startSort + fiveNum * 5 + 6] = startSort + fiveNum * 5 + 3;
				sortLine[startSort + fiveNum * 5 + 7] = startSort + fiveNum * 5 + 8;
				sortLine[startSort + fiveNum * 5 + 8] = startSort + fiveNum * 5 + 4;
			}
			if (oneNum == 1)
				sortLine[startSort + lineNum - 1] = startSort + lineNum - 1;
		}
	}
	int line_index = 0;
	for (int i = 0; i < (int)grid_lines.size(); i++)
	{
		line_index = sortLine[i];
		
		cv::Point point_start = grid_lines[line_index].upper_line.front();
		cv::Point point_end = grid_lines[line_index].upper_line.back();
		vector<cv::Point> point;
		point.push_back(point_start);
		point.push_back(point_end);

		cv::transform(point, point, R_cell_inv);
		point_start = point[0];
		point_end = point[1];

		float distance_start = map_resolution * hypotf((robot_pos.y - point_start.y), (robot_pos.x - point_start.x));
		float distance_end = map_resolution * hypotf((robot_pos.y - point_end.y), (robot_pos.x - point_end.x));

		if (min(distance_end, distance_start) > 5 && (i % 5 == 1 || i % 5 == 3))
		{
			int startReSortIndex = 0;
			int endReSortIndex = 0;
			if (i == 1)
			{
				startReSortIndex = i;
				endReSortIndex = min((int)grid_lines.size() - 1, i + 3);
			}
			if (i == 3)
			{
				startReSortIndex = i;
				endReSortIndex = min((int)grid_lines.size() - 1, i + 1);
			}

			if (endReSortIndex != startReSortIndex)
			{
				int lineIndexTemp = sortLine[endReSortIndex];

				cv::Point pointStartTemp = grid_lines[lineIndexTemp].upper_line.front();
				cv::Point pointEndTemp = grid_lines[lineIndexTemp].upper_line.back();

				float distanceStartTemp = map_resolution * hypotf((robot_pos.y - pointStartTemp.y), (robot_pos.x - pointStartTemp.x));
				float distanceEndTemp = map_resolution * hypotf((robot_pos.y - pointEndTemp.y), (robot_pos.x - pointEndTemp.x));

				if (distanceStartTemp < 5 || distanceEndTemp < 5)
				{
					int temp = sortLine[startReSortIndex];
					sortLine[startReSortIndex] = sortLine[endReSortIndex];
					sortLine[endReSortIndex] = temp;
					i--;
					continue;
				}
			}
		}

		if (distance_end <= distance_start)
		{ 
			for (int i = grid_lines[line_index].upper_line.size() - 1; i >= 0; i--)
			{
				cv::Point point_tmp;
				point_tmp.x = grid_lines[line_index].upper_line[i].x;
				point_tmp.y = grid_lines[line_index].upper_line[i].y;

				robot_pos.x = point_tmp.x;
				robot_pos.y = point_tmp.y;
				current_fov_path.push_back(point_tmp);
				if (i == (int)grid_lines[line_index].upper_line.size() - 1)
					firstPointIndex.push_back((int)current_fov_path.size() - 1);
			}
		}
		else
		{ // 从头加到尾

			for (int i = 0; i < (int)grid_lines[line_index].upper_line.size(); i++)
			{
				cv::Point point_tmp;
				point_tmp.x = grid_lines[line_index].upper_line[i].x;
				point_tmp.y = grid_lines[line_index].upper_line[i].y;
			
				robot_pos.x = point_tmp.x;
				robot_pos.y = point_tmp.y;
				current_fov_path.push_back(point_tmp);
				if (i == 0)
					firstPointIndex.push_back((int)current_fov_path.size() - 1);
			}
		}

		vector<cv::Point> robotPos;
		robotPos.push_back(robot_pos);
		cv::transform(robotPos, robotPos, R_cell_inv);
		robot_pos.x = robotPos[0].x;
		robot_pos.y = robotPos[0].y;
	}

	// remap the fov path to the originally rotated cell and add the found points to the global path
	std::vector<cv::Point2f> fov_middlepoint_path_part;
	for (std::vector<cv::Point>::iterator point = current_fov_path.begin(); point != current_fov_path.end(); ++point)
		fov_middlepoint_path_part.push_back(cv::Point2f(point->x, point->y));
	cv::transform(fov_middlepoint_path_part, fov_middlepoint_path_part, R_cell_inv);

	for (int i = 0; i < (int)fov_middlepoint_path_part.size(); i++)
	{
		planning_utils::Pose poseTemp;
		poseTemp.pt.x = fov_middlepoint_path_part[i].x;
		poseTemp.pt.y = fov_middlepoint_path_part[i].y;
		cellData.current_path.add(poseTemp);
	}
}


void BoustrophedonExplorer::computeBoustrophedonPath_miss_out(std::vector<missOutPath> &missOutData, const cv::Mat &room_map, const float map_resolution,
															  std::vector<GeneralizedPolygon> cell_polygons,
															  std::vector<int> optimal_order,
															  cv::Point &robot_pos,
															  const int grid_spacing_as_int, const int half_grid_spacing_as_int,
															  const double path_eps, const int max_deviation_from_track,
															  const int grid_obstacle_offset,
															  const float min_line_cleaning_distance)
{
	int i = 0;
	int cleaning_num = 0;
	int miss_out_index = 0;
	int numMissOut = 0;
	missOutData.clear();
	if ((int)gird_lines_all.size() > 0)
	{
		for (auto iter = gird_lines_all.end() - 1; iter >= gird_lines_all.begin(); iter--)
		{
			if (std::get<0>(*iter) == 1)
			{
				cleaning_num++;
			}

			if (std::get<0>(*iter) == 0)
			{
				cv::Point point_start = std::get<2>(*iter).upper_line.front();
				cv::Point point_end = std::get<2>(*iter).upper_line.back();

				float distance_start = map_resolution * hypotf((robot_pos.y - point_start.y), (robot_pos.x - point_start.x));
				float distance_end = map_resolution * hypotf((robot_pos.y - point_end.y), (robot_pos.x - point_end.x));

				miss_out_index = std::get<1>(*iter);
				std::vector<cv::Point> current_fov_path;

				// 取distance_start和distance_end中大的那一个
				float min_distance = distance_start < distance_end ? distance_start : distance_end;

				int end_start_index = std::get<2>(*iter).upper_line.size() - 1;
				int front_start_index = 0;

				if (min_distance < grid_spacing_as_int * 2)
				{
					for (int p = std::get<2>(*iter).upper_line.size() - 1; p >= 0; p--)
					{
						cv::Point point_tmp;
						point_tmp.x = std::get<2>(*iter).upper_line[p].x;
						point_tmp.y = std::get<2>(*iter).upper_line[p].y;
						float index_distance_tmp = hypotf((robot_pos.y - point_tmp.y), (robot_pos.x - point_tmp.x));

						if (index_distance_tmp > grid_spacing_as_int * 2)
						{
							end_start_index = p;
							break;
						}
					}

					for (int q = 0; q <= (int)std::get<2>(*iter).upper_line.size() - 1; q++)
					{
						cv::Point point_tmp;
						point_tmp.x = std::get<2>(*iter).upper_line[q].x;
						point_tmp.y = std::get<2>(*iter).upper_line[q].y;
						float index_distance_tmp = hypotf((robot_pos.y - point_tmp.y), (robot_pos.x - point_tmp.x));

						if (index_distance_tmp > grid_spacing_as_int * 2)
						{
							front_start_index = q;
							break;
						}
					}
				}

				if (distance_end < distance_start)
				{
					cv::Point point_tmp;
					for (int k = end_start_index; k >= 0; k--)
					{
						point_tmp.x = std::get<2>(*iter).upper_line[k].x;
						point_tmp.y = std::get<2>(*iter).upper_line[k].y;
						float index_distance_tmp = hypotf((robot_pos.y - point_tmp.y), (robot_pos.x - point_tmp.x));

						current_fov_path.push_back(point_tmp);
					}

					robot_pos.x = point_tmp.x;
					robot_pos.y = point_tmp.y;
				}
				else
				{
					cv::Point point_tmp;
					for (int k = front_start_index; k <= (int)std::get<2>(*iter).upper_line.size() - 1; k++)
					{

						point_tmp.x = std::get<2>(*iter).upper_line[k].x;
						point_tmp.y = std::get<2>(*iter).upper_line[k].y;
						float index_distance_tmp = hypotf((robot_pos.y - point_tmp.y), (robot_pos.x - point_tmp.x));
						current_fov_path.push_back(point_tmp);
					}

					robot_pos.x = point_tmp.x;
					robot_pos.y = point_tmp.y;
				}

				// remap the fov path to the originally rotated cell and add the found points to the global path
				std::vector<cv::Point2f> fov_middlepoint_path_part;
				for (std::vector<cv::Point>::iterator point = current_fov_path.begin(); point != current_fov_path.end(); ++point)
				{
					fov_middlepoint_path_part.push_back(cv::Point2f(point->x, point->y));
				}

				cv::Mat R_cell_inv;
				for (int m = 0; m < (int)gird_lines_all_R_cell_inv.size(); m++)
				{
					if (std::get<0>(gird_lines_all_R_cell_inv[m]) == miss_out_index)
					{
						R_cell_inv = std::get<1>(gird_lines_all_R_cell_inv[m]);
					}
				}

				cv::transform(fov_middlepoint_path_part, fov_middlepoint_path_part, R_cell_inv);

				missOutPath pathTemp;
				pathTemp.index = numMissOut;
				pathTemp.cellNum = miss_out_index;

				for (int i = 0; i < (int)fov_middlepoint_path_part.size(); i++)
				{
					planning_utils::Pose poseTemp;
					poseTemp.pt.x = fov_middlepoint_path_part[i].x;
					poseTemp.pt.y = fov_middlepoint_path_part[i].y;
					pathTemp.current_path.add(poseTemp);
				}

				missOutData.push_back(pathTemp);

				numMissOut++;
			}

			i++;
		}
	}
}

void BoustrophedonExplorer::downsamplePath(const std::vector<cv::Point> &original_path,
										   std::vector<cv::Point> &downsampled_path,
										   cv::Point &robot_pos, const double path_eps)
{
	// downsample path
	for (size_t path_point = 0; path_point < original_path.size(); ++path_point)
	{
		if (cv::norm(robot_pos - original_path[path_point]) >= path_eps)
		{
			downsampled_path.push_back(original_path[path_point]);
			robot_pos = original_path[path_point];
		}
	}
	// add last element
	if (original_path.size() > 0)
	{
		downsampled_path.push_back(original_path.back());
		robot_pos = original_path.back();
	}
}

void BoustrophedonExplorer::downsamplePathReverse(const std::vector<cv::Point> &original_path, std::vector<cv::Point> &downsampled_path,
												  cv::Point &robot_pos, const double path_eps)
{
	// downsample path
	for (size_t path_point = original_path.size() - 1;; --path_point)
	{
		if (cv::norm(robot_pos - original_path[path_point]) >= path_eps)
		{
			downsampled_path.push_back(original_path[path_point]);
			robot_pos = original_path[path_point];
		}
		if (path_point == 0)
			break;
	}
	// add last element
	if (original_path.size() > 0)
	{
		downsampled_path.push_back(original_path[0]);
		robot_pos = original_path[0];
	}
}

void BoustrophedonExplorer::printCells(std::map<int, boost::shared_ptr<BoustrophedonCell>> &cell_index_mapping)
{

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BoustrophedonVariantExplorer
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BoustrophedonVariantExplorer::mergeCellsSelection(cv::Mat &cell_map, cv::Mat &cell_map_labels, std::map<int, boost::shared_ptr<BoustrophedonCell>> &cell_index_mapping,
													   const double min_cell_area, const int min_cell_width)
{
	// iteratively merge cells
	//todo:
	// - take one major cell (the largest) and its major direction
	// - merge every other cell into the major cell, except
	//   - the width along the major direction is too small and the cell is sufficiently large
	//   - the bounding box orientation (side length ratio) deviates strongly from the major direction
	//   - the cell main direction is not well aligned with the major direction (skew, 90 deg)

	RoomRotator room_rotator;
	//double rotation_angle = room_rotator.computeRoomMainDirection(cell_map, map_resolution);

	// merge small cells below min_cell_area with their largest neighboring cell
	std::multimap<double, boost::shared_ptr<BoustrophedonCell>> area_to_region_id_mapping; // maps the area of each cell --> to the respective cell
	for (std::map<int, boost::shared_ptr<BoustrophedonCell>>::iterator itc = cell_index_mapping.begin(); itc != cell_index_mapping.end(); ++itc)
		area_to_region_id_mapping.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell>>(itc->second->area_, itc->second));
	for (std::multimap<double, boost::shared_ptr<BoustrophedonCell>>::iterator it = area_to_region_id_mapping.begin(); it != area_to_region_id_mapping.end();)
	{
		// abort if no cells below min_cell_area remain unmerged into bigger cells
		if (it->first >= min_cell_area && it->second->bounding_box_.width >= min_cell_width && it->second->bounding_box_.height >= min_cell_width)
		{
			++it;
			continue;
		}

		// skip segments which have no neighbors
		if (it->second->neighbors_.size() == 0)
		{
			// std::cout << "WARN: BoustrophedonExplorer::mergeCells: skipping small cell without neighbors." << std::endl;
			++it;
			continue;
		}

		// determine the largest neighboring cell
		const BoustrophedonCell &small_cell = *(it->second);
		std::multimap<double, boost::shared_ptr<BoustrophedonCell>, std::greater<double>> area_sorted_neighbors;
		for (BoustrophedonCell::BoustrophedonCellSetIterator itn = small_cell.neighbors_.begin(); itn != small_cell.neighbors_.end(); ++itn)
			area_sorted_neighbors.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell>>((*itn)->area_, *itn));
		BoustrophedonCell &large_cell = *(area_sorted_neighbors.begin()->second);

		// merge the cells
		mergeTwoCells(cell_map, cell_map_labels, small_cell, large_cell, cell_index_mapping);

		// update area_to_region_id_mapping
		area_to_region_id_mapping.clear();
		for (std::map<int, boost::shared_ptr<BoustrophedonCell>>::iterator itc = cell_index_mapping.begin(); itc != cell_index_mapping.end(); ++itc)
			area_to_region_id_mapping.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell>>(itc->second->area_, itc->second));
		it = area_to_region_id_mapping.begin();
	}
	// label remaining border pixels with label of largest neighboring region label
	for (int v = 1; v < cell_map.rows - 1; ++v)
	{
		for (int u = 1; u < cell_map.cols - 1; ++u)
		{
			if (cell_map.at<uchar>(v, u) == BORDER_PIXEL_VALUE)
			{
				std::set<int> neighbor_labels;
				for (int dv = -1; dv <= 1; ++dv)
				{
					for (int du = -1; du <= 1; ++du)
					{
						const int &val = cell_map_labels.at<int>(v + dv, u + du);
						if (val > 0)
							neighbor_labels.insert(val);
					}
				}
				if (neighbor_labels.size() > 0)
				{
					int new_label = -1;
					for (std::multimap<double, boost::shared_ptr<BoustrophedonCell>>::reverse_iterator it = area_to_region_id_mapping.rbegin(); it != area_to_region_id_mapping.rend(); ++it)
					{
						if (neighbor_labels.find(it->second->label_) != neighbor_labels.end())
						{
							cell_map_labels.at<int>(v, u) = it->second->label_;
							break;
						}
					}
				}
				// else
				// 	std::cout << "WARN: BoustrophedonExplorer::mergeCells: border pixel has no labeled neighbors." << std::endl;
			}
		}
	}
}
