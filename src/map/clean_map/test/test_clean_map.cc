#include <iostream>
#include <thread>

#include "log/log_manager.h"
#include "timer/timer.h"
#include "misc/planning_common_config.h"
#include "misc/robot_config.h"
#include "clean_map.h"

using namespace planning_map;
using namespace planning_utils;
using namespace std;

DEFINE_CONFIG_TYPE(CONFIG_PLANNING, Planing);
DEFINE_CONFIG_TYPE(CONFIG_ROBOT, Robot);

ConfigRobot *g_robot_cfg;

int main()
{
    ConfigManager cfg_mgr;
    // cfg_mgr.LoadConfig("../../../../../config/planning.yaml");
    cfg_mgr.LoadConfig("code/pms/config/robot.yaml");
    cfg_mgr.LoadConfig("code/planning/config/planning.yaml");

    g_robot_cfg = dynamic_cast<ConfigRobot *>(cfg_mgr.GetSubConfig(CONFIG_ROBOT));
    g_robot_cfg->setFoortprint();

    g_clean_map.init(
        cfg_mgr, []() {}, [](uint8_t *map) { return 0; });

    OPEN_LOG_EXCEPT();

    g_clean_map.loadMapFromFile("slam", g_clean_map.getMapInfo().grid_range,
         "下载/slam.bmp", CV_8UC3);

    MapSize map_size = g_clean_map.getMapInfo().size;
    uint8_t **map = new uint8_t *[map_size.x];
    for (size_t i = 0; i < map_size.x; i++)
    {
        map[i] = new uint8_t[map_size.y];
    }

    g_clean_map.inflateMap();

    g_clean_map.saveMapToFile("/tmp/log/test/inflate_map.bmp", "inflate",
                              SAVE_WHOLE, RobotGridRect{});

    g_clean_map.getInflateMap(CleanMap::NORMAL_INFLATE_RADIUS, map);
    g_clean_map.getLocalMap(map);

    cv::Mat mat(map_size.x, map_size.y, CV_8UC1);
    for (size_t i = 0; i < map_size.x; i++)
    {
        uint8_t *row = mat.ptr<uint8_t>(i);
        for (int32_t j = 0; j < map_size.y; j++)
        {
            uint8_t *ptr = row + j;
            *ptr = map[map_size.x - i - 1][map_size.y - j - 1];
        }
    }

    cv::imwrite("/tmp/log/test/final_map.bmp", mat);
    return 0;
}