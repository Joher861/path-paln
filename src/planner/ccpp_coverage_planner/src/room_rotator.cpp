/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2017 \n
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
 * Author: Richard Bormann
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 02.2017
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


#include <room_rotator.h>

void RoomRotator::rotateRoom(const cv::Mat& room_map, cv::Mat& rotated_room_map, const cv::Mat& R, const cv::Rect& bounding_rect)
{
	// rotate the image
	cv::warpAffine(room_map, rotated_room_map, R, bounding_rect.size(), cv::INTER_AREA);//仿射变换

	// apply a binary filter to create a binary image, also use a closing operator to smooth the output (the rotation might produce
	// black pixels reaching into the white area that were not there before, causing new, wrong cells to open)
	cv::threshold(rotated_room_map, rotated_room_map, 127, 255, CV_THRESH_BINARY);//以127为阈值将图片分为2值图
// this should not be used because it removes smaller obstacles like thin walls from the room and hence lets a planner generate paths through walls
//	cv::dilate(rotated_room_map, rotated_room_map, cv::Mat(), cv::Point(-1,-1), 1);
//	cv::erode(rotated_room_map, rotated_room_map, cv::Mat(), cv::Point(-1,-1), 1);
}

// compute the affine rotation matrix for rotating a room into parallel alignment with x-axis (longer side of the room is aligned with x-axis)
// R is the transform
// bounding_rect is the ROI of the warped image
double RoomRotator::computeRoomRotationMatrix(const cv::Mat& room_map, cv::Mat& R, cv::Rect& bounding_rect,
		const double map_resolution, const cv::Point* center, const double rotation_offset)
{
	// rotation angle of the map s.t. the most occurring gradient is in 90 degree to the x-axis
	double rotation_angle = computeRoomMainDirection(room_map, map_resolution) + rotation_offset;//得到主方向
	// std::cout << "RoomRotator::computeRoomRotationMatrix: main rotation angle: " << rotation_angle << std::endl;

	// get rotation matrix R for rotating the image around the center of the room contour
	//	Remark: rotation angle in degrees for opencv
	cv::Point center_of_rotation;//得到旋转的中心点
	if (center == 0)
	{
		cv::Point min_room, max_room;
		getMinMaxCoordinates(room_map, min_room, max_room);
		center_of_rotation.x = 0.5*(min_room.x+max_room.x);
		center_of_rotation.y = 0.5*(min_room.y+max_room.y);
	}
	else
		center_of_rotation = *center;

	// compute rotation
	R = cv::getRotationMatrix2D(center_of_rotation, (rotation_angle*180)/CV_PI, 1.0);//由旋转方向及旋转中心点及压缩系数1得到旋转矩阵，2*3

	// determine bounding rectangle to find the size of the new image
	bounding_rect = cv::RotatedRect(center_of_rotation, room_map.size(), (rotation_angle*180)/CV_PI).boundingRect();//确定边界矩形以查找新图像的大小
	// adjust transformation matrix//调整转换矩阵
	R.at<double>(0,2) += 0.5*bounding_rect.width - center_of_rotation.x;
	R.at<double>(1,2) += 0.5*bounding_rect.height - center_of_rotation.y;

	return rotation_angle;
}

// computes the major direction of the walls from a map (preferably one room)
// the map (room_map, CV_8UC1) is black (0) at impassable areas and white (255) on drivable areas
double RoomRotator::computeRoomMainDirection(const cv::Mat& room_map, const double map_resolution)
{
	const double map_resolution_inverse = 1./map_resolution;

	// compute Hough transform on edge image of the map
	cv::Mat edge_map;
	cv::Canny(room_map, edge_map, 50, 150, 3);//边缘提取得到边缘地图
	std::vector<cv::Vec4i> lines;
	double min_line_length = 1.0;	// in [m]
	for (; min_line_length > 0.1; min_line_length -= 0.2)//距离由长变短，然后计算霍夫直线，域值，直线数量大于4就退出
	{
		cv::HoughLinesP(edge_map, lines, 1, CV_PI/180, min_line_length*map_resolution_inverse, min_line_length*map_resolution_inverse, 1.5*min_line_length*map_resolution_inverse);
		cv::Mat room_hough = edge_map.clone();
		for (size_t i=0; i<lines.size(); ++i)
		{
			cv::Point p1(lines[i][0], lines[i][1]), p2(lines[i][2], lines[i][3]);
			cv::line(room_hough, p1, p2, cv::Scalar(128), 2);
		}
		//cv::imshow("room_hough", room_hough);
		if (lines.size() >= 4)
			break;
	}
	// setup a histogram on the line directions weighted by their length to determine the major direction
	Histogram<double> direction_histogram(0, CV_PI, 36);//最长线的方向作为主方向，范围为0-pi
	for (size_t i=0; i<lines.size(); ++i)
	{
		double dx = lines[i][2] - lines[i][0];
		double dy = lines[i][3] - lines[i][1];
		if(dy*dy+dx*dx > 0.0)
		{
			double current_direction = std::atan2(dy, dx);
			while (current_direction < 0.)
				current_direction += CV_PI;
			while (current_direction > CV_PI)
				current_direction -= CV_PI;
			direction_histogram.addData(current_direction, sqrt(dy*dy+dx*dx));
			//std::cout << " dx=" << dx << "   dy=" << dy << "   dir=" << current_direction << "   len=" << sqrt(dy*dy+dx*dx) << std::endl;
		}
	}
	return direction_histogram.getMaxBinPreciseVal();
}




// cv::transform() 给出了对于点的仿射函数。

// void cv::transform(
// 	cv::InputArray src, // Input N-by-1 array (Ds channels)
// 	cv::OutputArray dst, // Output N-by-1 array (Dd channels)
// 	cv::InputArray mtx // Transform matrix (Ds-by-Dd)
// );
// 其中 N 为点数，而点的维数分别为 Ds 和 Dd。

// cv::invertAffineTransform() 实现了逆仿射变换，其通过给定仿射矩阵，给出其逆仿射矩阵。

// void cv::invertAffineTransform(
// 	cv::InputArray M, // Input 2-by-3 matrix
// 	cv::OutputArray iM // Output also a 2-by-3 matrix
// );
void RoomRotator::transformPathBackToOriginalRotation(const std::vector<cv::Point2f>& fov_middlepoint_path, 
							std::vector<ipa_building_msgs::Point_x_y_theta>& path_fov_poses, const cv::Mat& R)
{
	path_fov_poses.clear();

	// transform the calculated path back to the originally rotated map
	cv::Mat R_inv;
	// 计算反变换矩阵
	cv::invertAffineTransform(R, R_inv);
	std::vector<cv::Point2f> fov_middlepoint_path_transformed;
	cv::transform(fov_middlepoint_path, fov_middlepoint_path_transformed, R_inv);

	// 加入theta角  //create poses with an angle
	transformPointPathToPosePath(fov_middlepoint_path_transformed, path_fov_poses);
}


// 加入theta角
void RoomRotator::transformPointPathToPosePath(const std::vector<cv::Point2f>& point_path, std::vector<ipa_building_msgs::Point_x_y_theta>& pose_path)
{
	// create poses with an angle
	for(size_t point_index = 0; point_index < point_path.size(); ++point_index)
	{
		// get the vector from the previous to the current point
		const cv::Point2f& current_point = point_path[point_index];

		// add the next navigation goal to the path
		ipa_building_msgs::Point_x_y_theta current_pose;
		current_pose.x = current_point.x;
		current_pose.y = current_point.y;
		current_pose.theta = 0.;
		cv::Point2f vector(0,0);
		if (point_index > 0)
		{
			// compute the direction as the line from the previous point to the current point
			vector = current_point - point_path[point_index-1];
		}
		else if (point_path.size() >= 2)
		{
			// for the first point take the direction between first and second point
			vector = point_path[point_index+1] - current_point;
		}
		// only sample different points
		if (vector.x!=0 || vector.y!=0)
		{
			current_pose.theta = std::atan2(vector.y, vector.x);
			pose_path.push_back(current_pose);
		}
	}
}

void RoomRotator::transformPointPathToPosePath(std::vector<ipa_building_msgs::Point_x_y_theta>& pose_path)
{
	// create point vector
	std::vector<cv::Point2f> point_path;
	for (size_t i=0; i<pose_path.size(); ++i)
		point_path.push_back(cv::Point2f(pose_path[i].x, pose_path[i].y));

	// create poses with an angle
	pose_path.clear();
	transformPointPathToPosePath(point_path, pose_path);
}

void RoomRotator::getMinMaxCoordinates(const cv::Mat& map, cv::Point& min_room, cv::Point& max_room)
{
	min_room.x = std::numeric_limits<int>::max();
	min_room.y = std::numeric_limits<int>::max();
	max_room.x = 0;
	max_room.y = 0;
	for (int v=0; v<map.rows; ++v)
	{
		for (int u=0; u<map.cols; ++u)
		{
			if (map.at<uchar>(v,u)==255)
			{
				min_room.x = std::min(min_room.x, u);
				min_room.y = std::min(min_room.y, v);
				max_room.x = std::max(max_room.x, u);
				max_room.y = std::max(max_room.y, v);
			}
		}
	}
}
