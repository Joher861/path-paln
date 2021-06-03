#ifndef _TEB_LOCAL_MAP_
#define _TEB_LOCAL_MAP_
#include "data/local_map_data.h"
#include "data_center/data_center.h"
#include "geometry/geometry_func.h"
#include "misc/planning_typedefs.h"
#include "costmap_2d/cost_values.h"


#define TEB_LOCAL_MAP_WINDOW_SIZE 161

class TebLocalMap
{
private:
    /* data */
    DataLocalMap m_local_map_data;
    RobotGrid m_origin_grid{80,80};
    float m_resolution = 0.05;
    int m_size_x = TEB_LOCAL_MAP_WINDOW_SIZE;
    int m_size_y = TEB_LOCAL_MAP_WINDOW_SIZE;

public:
    TebLocalMap(/* args */);
    ~TebLocalMap();

    void update();
    bool world2map(float wx, float wy, unsigned int* mx, unsigned int* my) const;
    bool map2world(unsigned int mx, unsigned int my, float* wx, float* wy) const;
    unsigned char getCost(unsigned int mx, unsigned int my) const;
    int size_x() const {return m_size_x;}
    int size_y() const {return m_size_y;}
    DataLocalMap* local_map_data() {return &m_local_map_data;}
    float resolution() const {return m_resolution;}

};



#endif