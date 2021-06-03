#include "local_layer.h"
#include "data_center/data_center.h"
#include "sys/stat.h"
#include "timer/timer.h"
#include "unistd.h"


#define LOCALTEST 0
#define SAVELOCALMAP 1

#if LOCALTEST
std::string saveFileAdress = "data/";
ofstream SaveFile_localLayer(saveFileAdress + "localLayer.txt");
#else
std::string saveFileAdress = "/tmp/log/localMap/";
std::ofstream SaveFile_localLayer(saveFileAdress + "localLayer.txt");

#endif
namespace planning_map
{

  void LocalLayer::getLocalMap2(uint8_t **dest)
  {
    for (size_t i = 0; i < m_total_size; ++i)
    {
      RobotGrid grid = parseIndex(i);
      dest[grid.x][grid.y] = m_map[i];
    }
  }

  void LocalLayer::update()
  {
      
    // planning_utils::g
    g_dc.getData<DataSlam>(m_slam_data);
    g_dc.getData<DataAllChassisInfo>(m_chassis_data);
    g_dc.getData<DataLocalMap>(m_localmap_data);

    const int localSizeX = m_map_info->localSize.x;
    const int localSizeY = m_map_info->localSize.y;

    uint16_t grid[localSizeX][localSizeY];

    memcpy(grid, m_localmap_data.grid, sizeof(m_localmap_data.grid));

    SaveFile_localLayer << m_slam_data.pose.pt.x << " " << m_slam_data.pose.pt.y << " " << m_slam_data.pose.theta << std::endl;
    for (int i = 0; i < localSizeX; i++)
    {
      for (int j = 0; j < localSizeY; j++)
      {
        RobotPose relativePoseTemp;
        localGridToRelativePose(i, j, relativePoseTemp.pt.x, relativePoseTemp.pt.y);
        RobotPose globalPoseTemp;
        relativePoseToGlobalPose(relativePoseTemp.pt.x, relativePoseTemp.pt.y, globalPoseTemp.pt.x, globalPoseTemp.pt.y);
        Point point;
        point.x = globalPoseTemp.pt.x;
        point.y = globalPoseTemp.pt.y;
        if (m_map_info->range.inRange(point))
        {
          RobotGrid gridTemp = poseToGrid(globalPoseTemp);
          if (grid[i][j] != 0 && gridTemp.x >= 0 && gridTemp.x < m_map_info->size.x && gridTemp.y >= 0 && gridTemp.y < m_map_info->size.y)
            set(gridTemp, OBSTACLE);
        }
      }
    }
#if LOCALTEST
    this->test();
#endif

#if SAVELOCALMAP
    long curTime = planning_utils::Timer::getTimestampUS();
    std::string time = std::to_string(curTime);
    if (access(saveFileAdress.c_str(), 00) != 0)
    {
      mkdir(saveFileAdress.c_str(), 0777);
      chmod(saveFileAdress.c_str(), 0777);
    }

    // std::ofstream SaveFile_localMap(saveFileAdress + time + "localMap.txt");
    std::string SaveFileImg = saveFileAdress + time + "image.png";
    cv::Mat localMap(m_map_info->size.x, m_map_info->size.y, CV_8UC1, cv::Scalar(0, 0, 0));
    // cv::line(localMap, cv::Point(m_map_info->origin_grid.y, 0), cv::Point(m_map_info->origin_grid.y, m_map_info->size.x), cv::Scalar(50, 50, 50), 2, cv::LINE_AA);
    // cv::line(localMap, cv::Point(0, m_map_info->origin_grid.x), cv::Point(m_map_info->size.y, m_map_info->origin_grid.x), cv::Scalar(50, 50, 50), 2, cv::LINE_AA);

    for (int i = 0; i < m_map_info->size.x; i++)
    {
      for (int j = 0; j < m_map_info->size.y; j++)
      {
        if (m_map[getIndex(i, j)] == OBSTACLE)
          localMap.at<unsigned char>(m_map_info->size.x - 1 - i, m_map_info->size.y - 1 - j) = OBSTACLE;
      }
    }

    cv::imwrite(SaveFileImg, localMap);
    // showWriter<<localMap;
#endif
  }

  inline void LocalLayer::localGridToRelativePose(int i, int j, float &xTemp, float &yTemp)
  {
    xTemp = -(m_map_info->localResolution * (m_map_info->localSize.x - 1)) + m_map_info->localResolution * (m_map_info->local_origin_grid.x) + i * (m_map_info->localResolution);
    yTemp = -(m_map_info->localResolution * (m_map_info->localSize.y - 1)) + m_map_info->localResolution * (m_map_info->local_origin_grid.y) + j * (m_map_info->localResolution);
  }

  inline void LocalLayer::relativePoseToGlobalPose(float xRaw, float yRaw, float &xTemp, float &yTemp)
  {
    float agvPoseX = m_slam_data.pose.pt.x;
    float agvPoseY = m_slam_data.pose.pt.y;
    float agvPoseYaw = m_slam_data.pose.theta;

    xTemp = xRaw + agvPoseX;
    yTemp = yRaw + agvPoseY;
  }

  inline void LocalLayer::test()
  {
  }

  inline void LocalLayer::set(int32_t x, int32_t y, const uint8_t &cell)
  {
    uint8_t &target_cell = m_map[getIndex(x, y)];
    target_cell = cell;
  }

  inline void LocalLayer::set(const RobotGrid &grid, const uint8_t &cell)
  {
    set(grid.x, grid.y, cell);
  }

  inline bool LocalLayer::isDefaultValueZero() const
  {
    return m_default_value == 0;
  }

  inline bool LocalLayer::isOccupied(const RobotGrid &grid) const
  {
    return m_map[getIndex(grid)] != UNKNOWN;
  }

  inline void LocalLayer::setMapGridToMatCell(const RobotGrid &grid,
                                              uint8_t *mat_cell, uint8_t channel) const
  {
    if (channel == CV_8UC1)
    {
      *mat_cell = m_map[getIndex(grid)];
    }
    else if (channel == CV_8UC3)
    {
      *mat_cell = m_map[getIndex(grid)];
      *(mat_cell + 1) = m_map[getIndex(grid)];
      *(mat_cell + 2) = m_map[getIndex(grid)];
    }
  }

  inline void LocalLayer::setMapGridFromMatCell(const RobotGrid &grid,
                                                const uint8_t *mat_cell, uint8_t channel) const
  {
    if (channel == CV_8UC1)
    {
      m_map[getIndex(grid)] = *mat_cell;
    }
    else if (channel == CV_8UC3)
    {
      m_map[getIndex(grid)] = *mat_cell;
      m_map[getIndex(grid)] = *(mat_cell + 1);
      m_map[getIndex(grid)] = *(mat_cell + 2);
    }
  }
}
    