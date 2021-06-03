#ifndef MAP_LOCAL_LAYER_H
#define MAP_LOCAL_LAYER_H

#include "map/layer_map.h"
#include "data/slam_data.h"
#include "data/chassis_data.h"
#include "data/local_map_data.h"

namespace planning_map
{
  class LocalLayer : public LayerMap<uint8_t>
  {
  public:
    enum LocalStatus
    {
      UNKNOWN = 254,
      FREE = 0,
      OBSTACLE = 255,
    };

    using LayerMap<uint8_t>::LayerMap;

    void test();
    void update();
    void set(int32_t x, int32_t y, const uint8_t &cell);
    void set(const RobotGrid &grid, const uint8_t &cell);
    void localGridToRelativePose(int i, int j, float &xTemp, float &yTemp);
    void relativePoseToGlobalPose(float xRaw, float yRaw, float &xTemp, float &yTemp);

    void getLocalMap2(uint8_t **dest);

  private:
    virtual bool isDefaultValueZero() const;
    virtual bool isOccupied(const RobotGrid &grid) const;
    virtual void setMapGridToMatCell(const RobotGrid &grid,
                                     uint8_t *mat_cell, uint8_t channel) const;
    virtual void setMapGridFromMatCell(const RobotGrid &grid,
                                       const uint8_t *mat_cell, uint8_t channel) const;

    DataSlam m_slam_data;
    DataAllChassisInfo m_chassis_data;
    DataLocalMap m_localmap_data;
  };

  using LocalLayerPtr = std::shared_ptr<LocalLayer>;
} // namespace planning_map

#endif // MAP_LOCAL_LAYER_H