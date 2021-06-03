

#include <local_map/include/local_map_viewer.h>

#include <stdio.h>
namespace planning_map
{


LocalMapViewer::LocalMapViewer() : initialized_(false)
{
  initialize();
}

void LocalMapViewer::initialize()
{
  if (initialized_)
    LMAP_DEBUG_LOG("LocalMapViewer already initialized. Reinitalizing...");
  
   
  
  //获取消息队列
  clearMessage(&msg_);
  qid_ = messageQueueGet(LOCAL_PLAN_MESSAGE_KEY);
  //获取机器人位置消息队列
  clearMessage(&pose_msg_);
  pose_qid_ = messageQueueGet(ROBOT_POSE_MESSAGE_KEY);
  //获取infeasiable机器人位置消息队列
  clearMessage(&infeasiable_pose_msg_);
  infeasiable_pose_qid_ = messageQueueGet(INFEASIABLE_POSE_MESSAGE_KEY);
  //获取共享内存的id shmid_
	global_plan_shmid_ = sharedMemoryGet(GLOBAL_PLAN_SHARED_MEMOTY_KEY, GLOBAL_PLAN_SHMEMSIZE);
	global_plan_shmaddr_ = (char*) sharedMemoryAttach(global_plan_shmid_);
  LMAP_DEBUG_LOG("globle plan shared memory id=%d,addr=%p.",global_plan_shmid_,global_plan_shmaddr_);

	obstacle_shmid_ = sharedMemoryGet(OBSTACLE_SHARED_MEMOTY_KEY, OBSTACLE_SHMEMSIZE);
	obstacle_shmaddr_ = (char*) sharedMemoryAttach(obstacle_shmid_);
  LMAP_DEBUG_LOG("obstacle shared memory id=%d,addr=%p.",obstacle_shmid_,obstacle_shmaddr_);

  //esdf的共享内存设置
	esdf_shmid = sharedMemoryGet(ESDF_SHARED_MEMOTY_KEY, ESDF_SHMEMSIZE);
	esdf_shmaddr_ = (char*) sharedMemoryAttach(esdf_shmid);
  LMAP_DEBUG_LOG("esdf shared memory id=%d,addr=%p.",esdf_shmid,esdf_shmaddr_);

  printf("message id=%d obstacle_shmid=%d ",qid_,obstacle_shmid_);
  //获取信息量组
  semid_ = semGet(TEB_SEM_KEY, SIZE_OF_SEM);

  initialized_ = true; 
}

void LocalMapViewer::publishRobotPose(const RobotPose& pose) 
{
    int i = 0;
    clearMessage(&pose_msg_);
    
    pose_msg_.mtype = MASSAGE_ROBOT_POSE_TYPE;

    float tmp_x,tmp_y,tmp_theta;

    tmp_x = pose.pt.x;
    tmp_y = pose.pt.y;
    tmp_theta = pose.theta;
    memcpy(pose_msg_.buffer,&tmp_x,sizeof(float));
    memcpy(pose_msg_.buffer+4,&tmp_y,sizeof(float));
    memcpy(pose_msg_.buffer+8,&tmp_theta,sizeof(float));

    pose_msg_.len = 12;

    // setMessage(&msg, (char*)&data, MESSAGE_SIZE, mtype);
    // messageQueueSend(qid_, &pose_msg_);//队列满时会堵塞程序
    messageQueueSend_nowait(pose_qid_, &pose_msg_);


}



void LocalMapViewer::publishInfeasibleRobotPose(const RobotPose& pose) 
{
    int i = 0;
    clearMessage(&infeasiable_pose_msg_);
    
    infeasiable_pose_msg_.mtype = MASSAGE_INFEASIABLE_POSE_TYPE;

    float tmp_x,tmp_y,tmp_theta;

    tmp_x = pose.pt.x;
    tmp_y = pose.pt.y;
    tmp_theta = pose.theta;
    memcpy(infeasiable_pose_msg_.buffer,&tmp_x,sizeof(float));
    memcpy(infeasiable_pose_msg_.buffer+4,&tmp_y,sizeof(float));
    memcpy(infeasiable_pose_msg_.buffer+8,&tmp_theta,sizeof(float));

    infeasiable_pose_msg_.len = 12;

    // setMessage(&msg, (char*)&data, MESSAGE_SIZE, mtype);
    // messageQueueSend(qid_, &infeasiable_pose_msg_);//队列满时会堵塞程序
    messageQueueSend_nowait(infeasiable_pose_qid_, &infeasiable_pose_msg_);
}

void LocalMapViewer::publishObstacles(const std::vector<Eigen::Vector2d> & obstacles) const
{
  if ( obstacles.empty() || printErrorWhenNotInitialized() )
    return;
  
  int i,ret;
  // LMAP_DEBUG_LOG("send obstacle");
  ret = semWait_timeout(semid_, OBSTACLE_SEMNUM, 10000);//等10ms
  if(ret == -1)
  {
    LMAP_DEBUG_LOG(" obstacles Can't wait a sem[%d].",semGetValue(semid_, OBSTACLE_SEMNUM));
    return;
  }//等待超时就退出
  sharedMemoryLock(obstacle_shmid_);
  uint32_t state = 0;
  i = 0;
  float tmp_x,tmp_y;
  for(auto &obs : obstacles)  
  {
    tmp_x = obs(0);
    tmp_y = obs(1);
    memcpy(obstacle_shmaddr_+4+8*i,&tmp_x,sizeof(float));
    memcpy(obstacle_shmaddr_+4+8*i+4,&tmp_y,sizeof(float));
    ++i;
    if(8*i >= OBSTACLE_SHMEMSIZE){break;}
  }
  state = i;
  state = state | 0x80000000;//置最高位为1，表示数据有更新
  memcpy(obstacle_shmaddr_,&state,sizeof(uint32_t));
  sharedMemoryUnlock(obstacle_shmid_);
  semIncrement(semid_, OBSTACLE_SEMNUM, 1);
  // LMAP_DEBUG_LOG("obstacle plan size=%d ",8*i);
}



void LocalMapViewer::publishObstacles(const std::vector<std::pair<RobotGrid, float>>& obstacles,LocalCharBaseLayerPtr layer) const
{
  if ( obstacles.empty() || printErrorWhenNotInitialized() )
    return;
  
  int i,ret;
  // LMAP_DEBUG_LOG("send obstacle");
  ret = semWait_timeout(semid_, OBSTACLE_SEMNUM, 10000);//等10ms
  if(ret == -1)
  {
    LMAP_DEBUG_LOG(" obstacles Can't wait a sem[%d].",semGetValue(semid_, OBSTACLE_SEMNUM));
    return;
  }//等待超时就退出
  sharedMemoryLock(obstacle_shmid_);
  uint32_t state = 0;
  i = 0;
  float tmp_x,tmp_y;
  for(auto &[grid, radius] : obstacles)  
  {
    RobotPose pose = layer->gridToPose(grid);
    tmp_x = pose.pt.x;
    tmp_y = pose.pt.y;
    memcpy(obstacle_shmaddr_+4+8*i,&tmp_x,sizeof(float));
    memcpy(obstacle_shmaddr_+4+8*i+4,&tmp_y,sizeof(float));
    ++i;
    if(8*i >= OBSTACLE_SHMEMSIZE){break;}
  }
  state = i;
  state = state | 0x80000000;//置最高位为1，表示数据有更新
  memcpy(obstacle_shmaddr_,&state,sizeof(uint32_t));
  sharedMemoryUnlock(obstacle_shmid_);
  semIncrement(semid_, OBSTACLE_SEMNUM, 1);
  // LMAP_DEBUG_LOG("obstacle plan size=%d ",8*i);
}


void LocalMapViewer::publishESDF(const MapInfo& map_info, SDFMap::Ptr sdf_map, LocalCharBaseLayerPtr layer) const
{
  
  int count,ret;
  // LMAP_DEBUG_LOG("send obstacle");
  ret = semWait_timeout(semid_, ESDF_SEMNUM, 10000);//等10ms
  if(ret == -1)
  {
    LMAP_DEBUG_LOG(" esdf Can't wait a sem[%d].",semGetValue(semid_, ESDF_SEMNUM));
    return;
  }//等待超时就退出
  sharedMemoryLock(esdf_shmid);
  uint32_t state = 0;
  count = 0;
  for(int i = 0;i < map_info.size.x;++i)
  {
    for(int j = 0;j < map_info.size.y;++j)
    {
      RobotPose pose = layer->gridToPose(RobotGrid{i,j});
      float dis = sdf_map->getDistance(pose.pt);
      memcpy(esdf_shmaddr_+4+12*count,&pose.pt.x,sizeof(float));
      memcpy(esdf_shmaddr_+4+12*count+4,&pose.pt.y,sizeof(float));
      memcpy(esdf_shmaddr_+4+12*count+8,&dis,sizeof(float));
      ++count;
      if(12*count >= ESDF_SHMEMSIZE){
        LMAP_DEBUG_LOG("exceed total container.");
        break;
      }
      // LMAP_DEBUG_LOG("Grid{%d,%d} pose{%f,%f} dis=%f",i,j,pose.pt.x,pose.pt.y,dis);
    }
  }

  state = count;
  state = state | 0x80000000;//置最高位为1，表示数据有更新
  memcpy(esdf_shmaddr_,&state,sizeof(uint32_t));
  sharedMemoryUnlock(esdf_shmid);
  semIncrement(semid_, ESDF_SEMNUM, 1);
  LMAP_DEBUG_LOG("esdf size=%d size_x=%d size_y=%d",12*count,map_info.size.x,map_info.size.y);
}

bool LocalMapViewer::printErrorWhenNotInitialized() const
{
  if (!initialized_)
  {
    LMAP_DEBUG_LOG("LocalMapViewer class not initialized. You must call initialize or an appropriate constructor");
    return true;
  }
  return false;
}

} // namespace teb_local_planner
