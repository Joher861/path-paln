#include "pre_process.h"
#include "boustrophedon_explorator.h"
#include "map/clean_map/clean_map.h"
using namespace std;
cv::Mat remove_noise_points(param param_ccpp)
{
    CvSeq *contour = NULL;
    double minarea = 200;
    double tmparea = 0.0;
    cv::Mat map_src(g_clean_map.getMapInfo().size.x, g_clean_map.getMapInfo().size.y, CV_8UC1, cv::Scalar(0, 0, 0));
    if (param_ccpp.planMode == 0)
    {
        MapSize map_size = g_clean_map.getMapInfo().size;
        for (int i = 0; i < map_size.x; i++)
        {
            for (int j = 0; j < map_size.y; j++)
            {
                RobotGrid grid;
                unsigned char temp;
                grid.x = i;
                grid.y = j;
                g_clean_map.get("slam", grid, temp);
                if (temp == 255)
                    map_src.at<unsigned char>(map_size.x - 1 - i, map_size.y - 1 - j) = 255;
                else
                    map_src.at<unsigned char>(map_size.x - 1 - i, map_size.y - 1 - j) = 0;
            }
        }

        for (int i = 0; i < 330; i++)
        {
            for (int j = 0; j < 250; j++)
            {
                map_src.at<unsigned char>(i, j) = 0;
            }
        }
        for (int row = 630; row <= 675; row++)
        {
            for (int col = 550; col <= 620; col++)
            {
                map_src.at<unsigned char>(map_size.x - col, row) = 255;
            }
        }
        // int erodeGrid = 4;
        // cv::imshow("a", map_src);
        // cv::waitKey(3000);
        cv::morphologyEx(map_src, map_src, cv::MORPH_ERODE,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(40, 40)));
        // cv::imshow("a", map_src);
        // cv::waitKey(10000);
        cv::morphologyEx(map_src, map_src, cv::MORPH_DILATE,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(40, 40)));
        // cv::imshow("a", map_src);
        // cv::waitKey(10000);
        // cv::morphologyEx(map_src, map_src, cv::MORPH_DILATE,
        //                  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5)));
        //         cv::morphologyEx(map_src, map_src, cv::MORPH_ERODE,
        //                  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        cv::imwrite("border2.png", map_src);
        // int erodeGrid = 1.7 / g_clean_map.getMapInfo().resolution * 2 + 1;
        // cv::morphologyEx(map_src, map_src, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erodeGrid, erodeGrid)));
    }
    else
    {
        std::vector<cv::Point> point;
        std::vector<std::vector<cv::Point>> vertices(1, point);

        for (int i = 0; i < (int)param_ccpp.ccppDataInput.size(); i++)
        {
            if (param_ccpp.ccppDataInput[i].index == param_ccpp.replanIndex)
            {
                for (int j = 0; j < (int)param_ccpp.ccppDataInput[i].vertices.size(); j++)
                {
                    cv::Point p;
                    p.x = param_ccpp.ccppDataInput[i].vertices[j].x;
                    p.y = param_ccpp.ccppDataInput[i].vertices[j].y;
                    vertices[0].push_back(p);
                }
                break;
            }
        }
        fillPoly(map_src, vertices, cv::Scalar(255));
        cv::imwrite("map_src_-1.png", map_src);

        for (int i = 0; i < map_src.rows; i++)
        {
            for (int j = 0; j < map_src.cols; j++)
            {
                if (map_src.at<unsigned char>(i, j) == 255)
                {
                    RobotGrid grid;
                    grid.x = g_clean_map.getMapInfo().size.x - 1 - i;
                    grid.y = g_clean_map.getMapInfo().size.y - 1 - j;
                    unsigned char scalar = 255;
                    unsigned char scalar2 = 255;
                    g_clean_map.get("clean", grid, scalar2);
                    g_clean_map.get("local", grid, scalar);
                    if (scalar == 255 || scalar2 == 2)
                        map_src.at<unsigned char>(i, j) = 0;
                }
            }
        }
    }
    cv::imwrite("map_src.png", map_src);

    vector<vector<cv::Point>> contoursRaw;
    vector<cv::Vec4i> hierarchyRaw;
    cv::findContours(map_src, contoursRaw, hierarchyRaw, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0)); //找到轮廓
    int contourIndex = 0;

    if (contoursRaw.size() > 0)
    {
        for (size_t i = 0; i < contoursRaw.size(); i++)
        {
            CvRect rect;
            double area = cv::contourArea(contoursRaw[i], 0);
            rect = cv::boundingRect(contoursRaw[i]);
            if (area < minarea)
            {
                if (map_src.at<unsigned char>(rect.y + rect.height / 2, rect.x + rect.width / 2) == 0)
                {
                    for (int y = rect.y; y < rect.y + rect.height; y++)
                    {
                        for (int x = rect.x; x < rect.x + rect.width; x++)
                        {
                            map_src.at<unsigned char>(y, x) = 255;
                        }
                    }
                }
            }
        }
    }
    cv::imwrite("map_src2.png", map_src);
    return map_src;
}

// 把图片中不连通的区域置黑
bool removeUnconnectedRoomParts(cv::Mat &room_map)
{
    // cv::imwrite("map_src7.png", room_map);

    // // create new map with segments labeled by increasing labels from 1,2,3,...
    // cv::Mat room_map_int(room_map.rows, room_map.cols, CV_32SC1);
    // for (int v = 0; v < room_map.rows; ++v)
    // {
    //     for (int u = 0; u < room_map.cols; ++u)
    //     {
    //         if (room_map.at<uchar>(v, u) == 255)
    //             room_map_int.at<int32_t>(v, u) = -100;
    //         else
    //         {
    //             room_map_int.at<int32_t>(v, u) = 0;
    //         }
    //     }
    // }

    // std::map<int, int> area_to_label_map; // maps area=number of segment pixels (keys) to the respective label (value)
    // int label = 1;
    // for (int v = 0; v < room_map_int.rows; ++v)
    // {
    //     for (int u = 0; u < room_map_int.cols; ++u)
    //     {
    //         if (room_map_int.at<int32_t>(v, u) == -100)
    //         { // 用给定的颜色填充一个连通区域
    //             const int area = cv::floodFill(room_map_int, cv::Point(u, v), cv::Scalar(label), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
    //             area_to_label_map[area] = label;
    //             ++label;
    //         }
    //     }
    // }

    // // abort if area_to_label_map.size() is empty
    // if (area_to_label_map.size() == 0)
    //     return false;

    // // remove all room pixels from room_map which are not accessible
    // const int label_of_biggest_room = area_to_label_map.rbegin()->second;
    // for (int v = 0; v < room_map.rows; ++v)
    //     for (int u = 0; u < room_map.cols; ++u)
    //         if (room_map_int.at<int32_t>(v, u) != label_of_biggest_room)
    //             room_map.at<uchar>(v, u) = 0;
    // cv::imwrite("map_src8.png", room_map);

    return true;
}

// 读取地图，设置清扫相关的内容
ipa_building_msgs::RoomExplorationActionGoal Generate_goal(param param_ccpp)
{
    // 解析地图

    // 去除噪点
    cv::Mat map = remove_noise_points(param_ccpp);
    cv::imwrite("map_src5.png", map);

    cv::Mat labeling = map.clone();
    cv::Mat labeling_erode = map.clone();

    int erodeGrid = 1.7 / g_clean_map.getMapInfo().resolution * 2 + 1;
    cv::morphologyEx(labeling_erode, labeling_erode, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erodeGrid, erodeGrid)));
    // 地图的起点
    std::vector<double> origin(3, 0); // (0,0,0)
    ipa_building_msgs::Point_x_y_z map_origin;
    map_origin.x = g_clean_map.getMapInfo().origin_pt.x;
    map_origin.y = g_clean_map.getMapInfo().origin_pt.y;
    map_origin.z = 0;

    // 清扫起点
    ipa_building_msgs::Point_x_y_theta starting_position;
    starting_position.x = param_ccpp.start_pos[0];
    starting_position.y = param_ccpp.start_pos[1];
    starting_position.theta = param_ccpp.start_pos[2];

    // 地图的分辨率 m
    double resolution = param_ccpp.resolution;
    // 机器人的半径 m
    double robot_radius = param_ccpp.robot_radius;
    // 进三退二规划器中每一个间隔的距离 m
    double grid_spacing_in_meter_ = param_ccpp.grid_spacing_in_meter;
    // 机器人的覆盖半径 m
    double coverage_radius = param_ccpp.coverage_radius;
    // 最小的清扫距离
    float min_line_cleaning_distance_ = param_ccpp.min_line_cleaning_distance;
    // 补充轨迹的距离 m
    float complementary_path_distance_ = param_ccpp.complementary_path_distance;
    // 需要Hybrid A* 的距离 m
    float need_hybrid_distance_ = param_ccpp.need_hybrid_distance;
    // 距离边界的距离 m
    float distance_to_boundary = param_ccpp.distance_to_boundary;
    // 分割cell的最小面积 m^2  // 清扫的最小面积  m^2
    double min_cell_area_ = param_ccpp.min_cell_area_;

    int planning_mode = 2; // viewpoint planning

    // 组帧为goal的数据结构
    ipa_building_msgs::RoomExplorationActionGoal goal;
    goal.input_map = labeling;
    goal.input_map_after_erode = labeling_erode;
    goal.map_resolution = resolution;
    goal.map_origin = map_origin;
    goal.robot_radius = robot_radius; // turtlebot, used for sim 0.177, 0.4
    goal.coverage_radius = coverage_radius;
    goal.starting_position = starting_position;
    goal.planning_mode = planning_mode;
    goal.min_line_cleaning_distance = min_line_cleaning_distance_;
    goal.complementary_path_index = complementary_path_distance_ / resolution;
    goal.need_hybrid_index = need_hybrid_distance_ / resolution;
    goal.index_to_boundary = distance_to_boundary / resolution;
    goal.min_cell_area = min_cell_area_;
    goal.grid_spacing_in_meter = grid_spacing_in_meter_;
    return goal;
}
void getBorderPath(const int map_size_x, const int map_size_y, const cv::Mat &room_map, std::vector<missOutPath> &missOutData, const float map_resolution,
                   cv::Point &starting_position, const cv::Point2d map_origin)
{
    vector<vector<cv::Point>> pathBorder;
    cv::Mat src = room_map.clone();
    float safe_distance = 0.2;
    float clean_diff = 0.5;
    float carWidth = 0.7;
    int borderNum = 3;

    for (int i = 0; i < borderNum; i++)
    {
        vector<vector<cv::Point>> cellsi;
        int erodeGrid = (carWidth / 2 + i * clean_diff + safe_distance) / map_resolution * 2 + 1;
        cv::Mat subTemp;
        cv::morphologyEx(src, subTemp, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erodeGrid, erodeGrid)));
        cv::findContours(subTemp, cellsi, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
        if (cellsi.size() > 0)
        {
            for (int j = 0; j < (int)cellsi.size(); j++)
                if ((int)cellsi[j].size() > 1)
                {
                    if (j != 0)
                        pathBorder.push_back(cellsi[j]);
                    else
                        pathBorder.insert(pathBorder.begin() + i, cellsi[j]);
                }
        }
    }

    for (int i = 0; i < pathBorder.size(); i++)
    {
        for (int j = 0; j < pathBorder[i].size(); j++)
        {
            int startIndex, endIndex;
            if (j == pathBorder[i].size() - 1)
            {
                startIndex = j - 1;
                endIndex = j;
            }
            else
            {
                startIndex = j;
                endIndex = j + 1;
            }

            float yaw = atan2(pathBorder[i][endIndex].y - pathBorder[i][startIndex].y, pathBorder[i][endIndex].x - pathBorder[i][startIndex].x);
            int length = 1.5 / map_resolution;
            int width = 0.7 / map_resolution;
            if (src.at<unsigned char>(pathBorder[i][endIndex].y + length * sin(yaw) + width / 2 * cos(yaw), pathBorder[i][endIndex].x + length * cos(yaw) - width / 2 * sin(yaw)) == 0 || src.at<unsigned char>(pathBorder[i][endIndex].y + length * sin(yaw) - width / 2 * cos(yaw), pathBorder[i][endIndex].x + length * cos(yaw) + width / 2 * sin(yaw)) == 0 || src.at<unsigned char>(pathBorder[i][endIndex].y - length * sin(yaw) + width / 2 * cos(yaw), pathBorder[i][endIndex].x - length * cos(yaw) - width / 2 * sin(yaw)) == 0 || src.at<unsigned char>(pathBorder[i][endIndex].y - length * sin(yaw) - width / 2 * cos(yaw), pathBorder[i][endIndex].x - length * cos(yaw) + width / 2 * sin(yaw)) == 0 || src.at<unsigned char>(pathBorder[i][endIndex].y + width / 2 * cos(yaw), pathBorder[i][endIndex].x - width / 2 * sin(yaw)) == 0 || src.at<unsigned char>(pathBorder[i][endIndex].y - width / 2 * cos(yaw), pathBorder[i][endIndex].x + width / 2 * sin(yaw)) == 0)
            {
                pathBorder[i].erase(pathBorder[i].begin() + j);
                j--;
            }
        }
    }

    for (int i = 0; i < pathBorder.size(); i++)
    {
        int pointLength = -1;
        int pointLastX = 0;
        int pointLastY = 0;
        for (int j = 0; j < pathBorder[i].size(); j++)
        {
            if (pointLength == -1)
            {
                pointLastX = pathBorder[i][j].x;
                pointLastY = pathBorder[i][j].y;
                pointLength++;
            }
            else
            {
                float dis = abs(pathBorder[i][j].x - pointLastX) + abs(pathBorder[i][j].y - pointLastY);
                if (dis > 5 || j == pathBorder[i].size() - 1)
                {
                    if (pointLength > 30)
                    {
                        pointLength = 0;
                        pointLastX = pathBorder[i][j].x;
                        pointLastY = pathBorder[i][j].y;
                    }
                    else
                    {
                        if (j == pathBorder[i].size() - 1)
                        {
                            pathBorder[i].erase(pathBorder[i].begin() + j - pointLength - 1, pathBorder[i].begin() + j + 1);
                        }
                        else
                        {
                            pathBorder[i].erase(pathBorder[i].begin() + j - pointLength - 1, pathBorder[i].begin() + j);
                            j = j - pointLength - 1;
                            pointLength = 0;
                            pointLastX = pathBorder[i][j].x;
                            pointLastY = pathBorder[i][j].y;
                        }
                    }
                }
                else
                {
                    pointLastX = pathBorder[i][j].x;
                    pointLastY = pathBorder[i][j].y;
                    pointLength++;
                }
            }
        }
    }

    int allowGrip = 1.5 / map_resolution;
    for (int i = 0; i < borderNum; i++)
    {
        for (int j = 0; j < pathBorder[i].size(); j++)
        {
            int startIndex = -1;
            for (int k = j + 1; k < pathBorder[i].size(); k++)
            {
                if (abs(pathBorder[i][k].y - pathBorder[i][j].y) + abs(pathBorder[i][k].x - pathBorder[i][j].x) > allowGrip)
                {
                    startIndex = k;
                    break;
                }
            }
            if (startIndex != -1)
            {
                for (int k = pathBorder[i].size() - 45; k >= startIndex; k--)
                {
                    if (abs(pathBorder[i][k].y - pathBorder[i][j].y) + abs(pathBorder[i][k].x - pathBorder[i][j].x) <= allowGrip)
                    {
                        pathBorder[i].erase(pathBorder[i].begin() + startIndex, pathBorder[i].begin() + k);
                        j = j - 1;
                        break;
                    }
                }
            }
        }
    }

    float originX = g_clean_map.getMapInfo().origin_pt.x;
    float originY = g_clean_map.getMapInfo().origin_pt.y;
    float resolution = g_clean_map.getMapInfo().resolution;
    vector<vector<cv::Point2f>> pathTemp;
    for (int i = borderNum; i < pathBorder.size(); i++)
    {
        vector<cv::Point2f> pointTemp;

        for (int j = 0; j < pathBorder[i].size(); j++)
        {
            cv::Point2f Temp;
            Temp.x = originX + (map_size_x - pathBorder[i][j].y) * resolution;
            Temp.y = originY + (map_size_y - pathBorder[i][j].x) * resolution;
            pointTemp.push_back(Temp);
        }
        pathTemp.push_back(pointTemp);
    }

    vector<vector<cv::Point2f>> pathTempTemp;
    vector<cv::Point2f> startpoint;
    vector<int> sortIndex;
    for (int i = 0; i < (int)pathTemp.size(); i++)
    {
        cv::Point2f point = pathTemp[i].front();
        startpoint.push_back(point);
        sortIndex.push_back(i);
    }
    cv::Point2f robotPos;
    robotPos.x = originX + (map_size_x - starting_position.y) * resolution;
    robotPos.y = originY + (map_size_y - starting_position.x) * resolution;
    while ((int)sortIndex.size() != 0)
    {
        float dis = 10000000;
        int index = -1;
        for (int i = 0; i < (int)startpoint.size(); i++)
        {
            float disTemp = abs(startpoint[i].x - robotPos.x) + abs(startpoint[i].y - robotPos.y);
            if (disTemp < dis)
            {
                dis = disTemp;
                index = i;
            }
        }
        if (index != -1)
        {
            robotPos.x = pathTemp[sortIndex[index]].back().x;
            robotPos.y = pathTemp[sortIndex[index]].back().y;

            pathTempTemp.push_back(pathTemp[sortIndex[index]]);
            startpoint.erase(startpoint.begin() + index);
            sortIndex.erase(sortIndex.begin() + index);
        }
        else
        {
            break;
        }
    }
    pathTemp = pathTempTemp;

    const int temp = min(borderNum, (int)pathBorder.size() - 1);
    int sortExtern[temp][2];

    for (int i = 0; i < temp; i++)
    {
        int startIndex = -1;
        int endIndex = -1;
        sortExtern[i][0] = (int)pathTemp.size();
        vector<cv::Point2f> pointTemp;
        for (int j = 0; j < pathBorder[i].size(); j++)
        {
            cv::Point2f Temp;
            if (startIndex == -1)
            {
                startIndex = j;
                Temp.x = originX + (map_size_x - pathBorder[i][j].y) * resolution;
                Temp.y = originY + (map_size_y - pathBorder[i][j].x) * resolution;
                pointTemp.push_back(Temp);
            }
            else
            {
                if (abs(pathBorder[i][j].x - pathBorder[i][j - 1].x) + abs(pathBorder[i][j].y - pathBorder[i][j - 1].y) > 20 || j == pathBorder[i].size() - 1 || (int)pointTemp.size() > 100)
                {
                    endIndex = j - 1;
                    pathTemp.push_back(pointTemp);
                    j--;
                    startIndex = -1;
                    endIndex = -1;
                    pointTemp.clear();
                }
                else
                {
                    Temp.x = originX + (map_size_x - pathBorder[i][j].y) * resolution;
                    Temp.y = originY + (map_size_y - pathBorder[i][j].x) * resolution;
                    pointTemp.push_back(Temp);
                }
            }
        }
        sortExtern[i][1] = (int)pathTemp.size() - 1;
    }

    pathTempTemp.clear();
    pathTempTemp = pathTemp;
    for (int i = 0; i < temp; i++)
    {
        int startIndex = sortExtern[i][0];
        int endIndex = sortExtern[i][1];
        float mindis = 100000000;
        float minIndex = -1;
        for (int j = startIndex; j <= endIndex; j++)
        {
            float disTemp = abs(pathTempTemp[j].front().x - robotPos.x) + abs(pathTempTemp[j].front().y - robotPos.y);
            if (disTemp < mindis)
            {
                mindis = disTemp;
                minIndex = j;
            }
        }

        int indexTemp = 0;
        for (int j = minIndex; j <= endIndex; j++)
        {
            pathTemp[startIndex + indexTemp] = pathTempTemp[j];
            robotPos.x = pathTempTemp[j].back().x;
            robotPos.y = pathTempTemp[j].back().y;

            indexTemp++;
        }
        for (int j = startIndex; j < minIndex; j++)
        {
            pathTemp[startIndex + indexTemp] = pathTempTemp[j];
            robotPos.x = pathTempTemp[j].back().x;
            robotPos.y = pathTempTemp[j].back().y;
            indexTemp++;
        }
    }

    pathTempTemp.clear();
    pathTempTemp = pathTemp;
    for (int i = 0; i < (int)pathTemp.size(); i++)
    {
        int filiterNum = 10;
        for (int j = 0; j < pathTemp[i].size(); j++)
        {
            int startIndex = max(0, j - filiterNum);
            int endIndex = min(j + filiterNum, (int)pathTemp[i].size() - 1);
            int num = endIndex - startIndex + 1;
            float sumX = 0;
            float sumY = 0;
            for (int k = startIndex; k <= endIndex; k++)
            {
                sumX = pathTempTemp[i][k].x + sumX;
                sumY = pathTempTemp[i][k].y + sumY;
            }
            pathTemp[i][j].x = sumX / num;
            pathTemp[i][j].y = sumY / num;
        }
    }

    for (int i = 0; i < (int)pathTemp.size(); i++)
    {
        missOutPath pathTemp2;
        pathTemp2.index = i;
        for (int j = 0; j < pathTemp[i].size(); j++)
        {
            RobotPose pointTemp;
            pointTemp.pt.x = pathTemp[i][j].x;
            pointTemp.pt.y = pathTemp[i][j].y;
            if (j == (int)pathTemp[i].size() - 1)
            {
                pointTemp.theta = pathTemp2.current_path.back().theta;
            }
            else
            {
                pointTemp.theta = atan2(pathTemp[i][j + 1].y - pathTemp[i][j].y, pathTemp[i][j + 1].x - pathTemp[i][j].x);
            }

            pathTemp2.current_path.add(pointTemp);
        }
        missOutData.push_back(pathTemp2);
    }
}
DataCCPP explore_room(ipa_building_msgs::RoomExplorationActionGoal goal, string name_path, param param_ccpp)
{

    //  I. read the given parameters out of the goal
    // todo: this is only correct if the map is not rotated
    const cv::Point2d map_origin(goal.map_origin.x, goal.map_origin.y); //地图原点
    const float map_resolution = goal.map_resolution;                   // in [m/cell]//精度
    const float map_resolution_inverse = 1. / map_resolution;           //一米多少个格子
    const float min_line_cleaning_distance = goal.min_line_cleaning_distance;

    // pose to grid
    const float robot_radius = goal.robot_radius;                      //机器半径
    const int robot_radius_in_pixel = (robot_radius / map_resolution); //机器半径占的格子
    // const cv::Point starting_position(goal.input_map.cols - 1 - (goal.starting_position.y - map_origin.y) / map_resolution,
    //                                   (goal.input_map.rows - 1 - (goal.starting_position.x - map_origin.x) / map_resolution)); //起始坐标的位置
    cv::Point starting_position((goal.input_map.rows - 1 - (goal.starting_position.x - map_origin.x) / map_resolution),
                                goal.input_map.cols - 1 - (goal.starting_position.y - map_origin.y) / map_resolution); //起始坐标的位置
    int PLAN_FOR_FOOTPRINT = 1;
    int PLAN_FOR_FOV = 2;
    int planning_mode_ = goal.planning_mode;

    // 接受地图
    cv::Mat room_map = goal.input_map_after_erode;
    cv::imwrite("map_src6.png", room_map);
    // 场地的面积
    int area_px = 0; // room area in pixels
    for (int v = 0; v < room_map.rows; ++v)
        for (int u = 0; u < room_map.cols; ++u)
            if (room_map.at<uchar>(v, u) >= 250)
                area_px++;

    // closing operation to neglect inaccessible areas and map errors/artifacts
    // 腐蚀与膨胀(Eroding and Dilating)
    // if (param_ccpp.planMode == 0)
    // {
    //     int map_correction_closing_neighborhood_size_ = 10;
    //     cv::Mat temp;
    //     cv::erode(room_map, temp, cv::Mat(), cv::Point(-1, -1), map_correction_closing_neighborhood_size_);
    //     cv::dilate(temp, room_map, cv::Mat(), cv::Point(-1, -1), map_correction_closing_neighborhood_size_);
    // }
    // remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
    const bool room_not_empty = removeUnconnectedRoomParts(room_map); //保留最大区域，改成保留机器所在区域?

    // get the grid size, to check the areas that should be revisited later
    double grid_spacing_in_meter = 0.0;                              // is the square grid cell side length that fits into the circle with the robot's coverage radius or fov coverage radius
    Eigen::Matrix<float, 2, 1> fitting_circle_center_point_in_meter; // this is also considered the center of the field of view, because around this point the maximum radius incircle can be found that is still inside the fov

    grid_spacing_in_meter = goal.grid_spacing_in_meter;
    const double grid_spacing_in_pixel = grid_spacing_in_meter / map_resolution; // is the square grid cell side length that fits into the circle with the robot's coverage radius or fov coverage radius, multiply with sqrt(2) to receive the whole working width

    // II. plan the path using the wanted planner
    // todo: consider option to provide the inflated map or the robot radius to the functions instead of inflating with half cell size there

    DataCCPP dataCCPP;
    double grid_obstacle_offset_ = 0;
    double path_eps_ = 1; // distance between two points when generating the (line) path,[pixel], where one pixel is the size of one cell in the navigation grid map
    int cell_visiting_order_ = 1;
    double min_cell_area_ = goal.min_cell_area;
    int max_deviation_from_track_ = -1;
    int complementary_path_index_ = goal.complementary_path_index;
    int need_hybrid_index_ = goal.need_hybrid_index;
    int index_to_boundary_ = goal.index_to_boundary;

    BoustrophedonExplorer BoustrophedonExplorer_;

    int map_size_x = g_clean_map.getMapInfo().size.x;
    int map_size_y = g_clean_map.getMapInfo().size.y;

    // boustrophedon_explorator // 主函数
    BoustrophedonExplorer_.getExplorationPath(map_size_x, map_size_y, room_map, dataCCPP, map_resolution,
                                              starting_position, map_origin,
                                              grid_spacing_in_pixel,
                                              grid_obstacle_offset_, path_eps_, cell_visiting_order_,
                                              false, fitting_circle_center_point_in_meter,
                                              min_cell_area_, max_deviation_from_track_,
                                              min_line_cleaning_distance, complementary_path_index_,
                                              need_hybrid_index_, index_to_boundary_, name_path

    );

    if (param_ccpp.planMode == 1)
    {
        std::vector<ccppCellData> ccppDataInputTemp;
        std::vector<missOutPath> missOutDataInputTemp;
        for (int i = 0; i < param_ccpp.replanIndex; i++)
            ccppDataInputTemp.push_back(param_ccpp.ccppDataInput[i]);
        for (int i = 0; i < (int)dataCCPP.ccppData.size(); i++)
        {
            ccppDataInputTemp.push_back(dataCCPP.ccppData[i]);
            ccppDataInputTemp.back().index = ccppDataInputTemp.size() - 1;
        }
        for (int i = param_ccpp.replanIndex + 1; i < (int)param_ccpp.ccppDataInput.size(); i++)
        {
            ccppDataInputTemp.push_back(param_ccpp.ccppDataInput[i]);
            ccppDataInputTemp.back().index = ccppDataInputTemp.size() - 1;
        }

        dataCCPP.ccppData = ccppDataInputTemp;
        dataCCPP.missOutData = param_ccpp.missOutDataInput;
    }
    else
    {
        getBorderPath(map_size_x, map_size_y, goal.input_map, dataCCPP.missOutData, map_resolution, starting_position, map_origin);
    }

    return dataCCPP;
}
