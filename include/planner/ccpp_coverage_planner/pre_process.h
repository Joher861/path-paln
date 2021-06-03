#ifndef PRE_PROCESS_H
#define PRE_PROCESS_H
#include "RoomExplorationActionGoal.h"
// #include "boustrophedon_explorator.h"
#include "ccpp_data.h"
using namespace std;

struct param
{
    // 清扫起点
    std::vector<double> start_pos = {500, 500, 0};
    // 地图的分辨率 m
    double resolution = 0.05;
    // 机器人的半径 m
    double robot_radius = 0.5;
    // 进三退二规划器中每一个间隔的距离 m
    double grid_spacing_in_meter = 0.5;
    // 机器人的覆盖半径 m
    double coverage_radius = 0.5;
    // 最小的清扫距离
    float min_line_cleaning_distance = 1;
    // 补充轨迹的距离 m
    float complementary_path_distance = 2;
    // 需要Hybrid A* 的距离 m
    float need_hybrid_distance = 5;
    // 距离边界的距离 m
    float distance_to_boundary = 1;
    // 分割cell的最小面积 m^2  // 清扫的最小面积  m^2
    double min_cell_area_ = 5;

    int planMode = 0; //模式0 ccpp规划 模式1 对指定区域重新规划
    int replanIndex = 0;
    std::vector<ccppCellData> ccppDataInput;
    std::vector<missOutPath> missOutDataInput;
};

//去除噪声点
cv::Mat remove_noise_points(param param_ccpp);

// 把图片中不连通的区域置黑
bool removeUnconnectedRoomParts(cv::Mat &room_map);

// 读取地图，设置清扫相关的内容
ipa_building_msgs::RoomExplorationActionGoal Generate_goal(param param_ccpp);

DataCCPP explore_room(ipa_building_msgs::RoomExplorationActionGoal goal, string name_path, param param_ccpp);

void getBorderPath(const int map_size_x, const int map_size_y, const cv::Mat &room_map, std::vector<missOutPath> &missOutData, const float map_resolution,
							cv::Point2f &starting_position, const cv::Point2d map_origin);

#endif