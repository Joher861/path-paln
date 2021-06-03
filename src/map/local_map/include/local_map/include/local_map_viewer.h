

#ifndef __LOCAL_MAP_VIEWER_H_
#define __LOCAL_MAP_VIEWER_H_

#include "local_map/include/local_map_log.h"
#include "misc/planning_typedefs.h"
#include "local_map/local_uint8_layer.h"
#include "local_map/include/sdf_map.h"

#include "MessageQueueWrapper.h"
#include "SharedMemoryWrapper.h"
#include "SemaphoreWrapper.h"
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>



namespace planning_map
{
  


#define MASSAGE_GLOBAL_PLAN_TYPE  15
#define MASSAGE_LOCAL_PLAN_TYPE   16
#define MASSAGE_ROBOT_POSE_TYPE   17
#define MASSAGE_INFEASIABLE_POSE_TYPE   20

#define LOCAL_PLAN_MESSAGE_KEY 13  
#define ROBOT_POSE_MESSAGE_KEY 12
#define INFEASIABLE_POSE_MESSAGE_KEY 17

//teb信号量所需数据
#define TEB_SEM_KEY 1   //KEY用于创建一个信号量组
#define SIZE_OF_SEM 6   //信号量组包含多少种信号量
#define GLOBAL_PLAN_SEMNUM 0     //信号量组中第几种信号量
#define OBSTACLE_SEMNUM 1     //信号量组中第几种信号量
#define ESDF_SEMNUM 5     //信号量组中第几种信号量
#define VAL 1        //信号量组中某种信号量的个数


//共享内存所需数据 共享内存的大小单位（bytes）
#define GLOBAL_PLAN_SHARED_MEMOTY_KEY 6
#define GLOBAL_PLAN_SHMEMSIZE 8192    

#define OBSTACLE_SHARED_MEMOTY_KEY 7
#define OBSTACLE_SHMEMSIZE 409600

#define ESDF_SHARED_MEMOTY_KEY 11
#define ESDF_SHMEMSIZE 409600

class LocalMapViewer
{
public:
    
  /**
   * @brief Default constructor
   * @remarks do not forget to call initialize()
   */
  LocalMapViewer();

  ~LocalMapViewer() {
    sharedMemoryDetatch(obstacle_shmaddr_);
    sharedMemoryDetatch(global_plan_shmaddr_);
    sharedMemoryDetatch(esdf_shmaddr_);
  }

  
  /**
   * @brief Initializes the class and registers topics.
   * 
   * Call this function if only the default constructor has been called before.
   * @param nh local ros::NodeHandle
   * @param cfg const reference to the TebConfig class for parameters
   */
  void initialize();
  
  
  /** @name Publish to topics */
  //@{
  void publishRobotPose(const RobotPose& pose); 

  void publishInfeasibleRobotPose(const RobotPose& pose);
  /**
   * @brief Publish obstacle positions to the ros topic \e ../../teb_markers
   * @todo Move filling of the marker message to polygon class in order to avoid checking types.
   * @param obstacles Obstacle container
   */
  void publishObstacles(const std::vector<std::pair<RobotGrid, float>>& obstacles,LocalCharBaseLayerPtr layer) const;

  void publishESDF(const MapInfo& map_info, SDFMap::Ptr sdf_map, LocalCharBaseLayerPtr layer) const;

  void publishObstacles(const std::vector<Eigen::Vector2d> & obstacles) const;
  
protected:

  bool printErrorWhenNotInitialized() const;


  
  bool initialized_; //!< Keeps track about the correct initialization of this class

	int qid_;
	Message msg_;
  int semid_;

  int global_plan_shmid_;
  char* global_plan_shmaddr_;
  int obstacle_shmid_;
  char* obstacle_shmaddr_;

  int esdf_shmid;
  char* esdf_shmaddr_;

  int pose_qid_;
  Message pose_msg_;
    
  int infeasiable_pose_qid_;
  Message infeasiable_pose_msg_;
     
};



} // namespace teb_local_planner




#endif /* VISUALIZATION_H_ */
