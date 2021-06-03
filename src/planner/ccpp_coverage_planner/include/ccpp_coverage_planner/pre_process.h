#ifndef PRE_PROCESS_H
#define PRE_PROCESS_H
#include "RoomExplorationActionGoal.h"
// #include "boustrophedon_explorator.h"
#include "ccpp_data.h"
using namespace std;

struct param
{

    std::vector<double> start_pos = {500, 500, 0};
    double resolution = 0.05;    
    double robot_radius = 0.5;
    double grid_spacing_in_meter = 0.5;

    double coverage_radius = 0.5;
    float min_line_cleaning_distance = 1;

    float complementary_path_distance = 2;
    float need_hybrid_distance = 5;
    float distance_to_boundary = 1;
    double min_cell_area_ = 5;

    int planMode = 0; 
    int replanIndex = 0;
    std::vector<ccppCellData> ccppDataInput;
    std::vector<missOutPath> missOutDataInput;
};


cv::Mat remove_noise_points(param param_ccpp);

bool removeUnconnectedRoomParts(cv::Mat &room_map);

ipa_building_msgs::RoomExplorationActionGoal Generate_goal(param param_ccpp);

DataCCPP explore_room(ipa_building_msgs::RoomExplorationActionGoal goal, string name_path, param param_ccpp);

void getBorderPath(const int map_size_x, const int map_size_y, const cv::Mat &room_map, std::vector<missOutPath> &missOutData, const float map_resolution,
							cv::Point2f &starting_position, const cv::Point2d map_origin);

#endif