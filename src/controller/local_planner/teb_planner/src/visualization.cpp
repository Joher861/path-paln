/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/visualization.h>
#include <teb_local_planner/optimal_planner.h>
// #include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/teb_log.h>
#include <stdio.h>
namespace planning_controller
{

TebVisualization::TebVisualization() : initialized_(false)
{
}

TebVisualization::TebVisualization(const TebConfig& cfg) : initialized_(false)
{
  initialize(cfg);
}

void TebVisualization::initialize(const TebConfig& cfg)
{
  if (initialized_)
    TEB_WARN_LOG("TebVisualization already initialized. Reinitalizing...");
  
  // set config
  cfg_ = &cfg;
  
  
  //获取消息队列
  clearMessage(&msg_);
  qid_ = messageQueueGet(LOCAL_PLAN_MESSAGE_KEY);
  //获取机器人位置消息队列
  clearMessage(&pose_msg_);
  pose_qid_ = messageQueueGet(ROBOT_POSE_MESSAGE_KEY);
  //获取共享内存的id shmid_
	global_plan_shmid_ = sharedMemoryGet(GLOBAL_PLAN_SHARED_MEMOTY_KEY, GLOBAL_PLAN_SHMEMSIZE);
	global_plan_shmaddr_ = (char*) sharedMemoryAttach(global_plan_shmid_);
  TEB_INFO_LOG("globle plan shared memory id=%d,addr=%p.",global_plan_shmid_,global_plan_shmaddr_);

	obstacle_shmid_ = sharedMemoryGet(OBSTACLE_SHARED_MEMOTY_KEY, OBSTACLE_SHMEMSIZE);
	obstacle_shmaddr_ = (char*) sharedMemoryAttach(obstacle_shmid_);
  TEB_INFO_LOG("obstacle shared memory id=%d,addr=%p.",obstacle_shmid_,obstacle_shmaddr_);

  //获取共享内存的id shmid_
	teb_container_shmid_ = sharedMemoryGet(TEBCONTAINER_SHARED_MEMOTY_KEY, TEBCONTAINER_SHMEMSIZE);
	teb_container_shmaddr_ = (char*) sharedMemoryAttach(teb_container_shmid_);
  TEB_INFO_LOG("teb container shared memory id=%d,addr=%p.",teb_container_shmid_,teb_container_shmaddr_);


  // printf("message id=%d obstacle_shmid=%d ",qid_,obstacle_shmid_);
  //获取信息量组
  semid_ = semGet(TEB_SEM_KEY, SIZE_OF_SEM);





  initialized_ = true; 
}



void TebVisualization::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) 
{
  if ( printErrorWhenNotInitialized() ) return;

  int i,ret;
  // TEB_INFO_LOG("send global plan");
  ret = semWait_timeout(semid_, GLOBAL_PLAN_SEMNUM, 10000);//等10ms
  if(ret == -1)
  {
    TEB_INFO_LOG("global plan Can't wait a sem[%d].",semGetValue(semid_, GLOBAL_PLAN_SEMNUM));
    return;
  }//等待超时就退出
  sharedMemoryLock(global_plan_shmid_);
  uint32_t state = 0;

  float tmp_x,tmp_y;
  for(i = 0;i < global_plan.size();)
  {
    tmp_x = global_plan[i].pose.position.x;
    tmp_y = global_plan[i].pose.position.y;
    memcpy(global_plan_shmaddr_+4+8*i,&tmp_x,sizeof(float));
    memcpy(global_plan_shmaddr_+4+8*i+4,&tmp_y,sizeof(float));
    ++i;
    if(8*i >= GLOBAL_PLAN_SHMEMSIZE){break;}
  }
  state = i;
  state = state | 0x80000000;//置最高位为1，表示数据有更新
  memcpy(global_plan_shmaddr_,&state,sizeof(uint32_t));
  sharedMemoryUnlock(global_plan_shmid_);
  semIncrement(semid_, GLOBAL_PLAN_SEMNUM, 1);

  // TEB_INFO_LOG("global plan size=%d ",8*i);

}


void TebVisualization::publishRobotPose(const PoseSE2& pose) 
{
    int i = 0;
    clearMessage(&pose_msg_);
    
    pose_msg_.mtype = MASSAGE_ROBOT_POSE_TYPE;

    float tmp_x,tmp_y,tmp_theta;

    tmp_x = pose.x();
    tmp_y = pose.y();
    tmp_theta = pose.theta();
    memcpy(pose_msg_.buffer,&tmp_x,sizeof(float));
    memcpy(pose_msg_.buffer+4,&tmp_y,sizeof(float));
    memcpy(pose_msg_.buffer+8,&tmp_theta,sizeof(float));

    pose_msg_.len = 12;

    // setMessage(&msg, (char*)&data, MESSAGE_SIZE, mtype);
    // messageQueueSend(qid_, &pose_msg_);//队列满时会堵塞程序
    messageQueueSend_nowait(pose_qid_, &pose_msg_);


}



void TebVisualization::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& local_plan) const
{
  // if ( printErrorWhenNotInitialized() )
  //   return;
  // base_local_planner::publishPlan(local_plan, local_plan_pub_); 
}

void TebVisualization::publishLocalPlanAndPoses(const TimedElasticBand& teb) 
{
  
  if ( printErrorWhenNotInitialized() || teb.sizePoses() == 0 )
    return;
  
    int i = 0;
    clearMessage(&msg_);
    
    msg_.mtype = MASSAGE_LOCAL_PLAN_TYPE;

    float tmp_x,tmp_y,tmp_theta;
    for(i = 0;i < teb.sizePoses();)
    {
      tmp_x = teb.Pose(i).x();
      tmp_y = teb.Pose(i).y();
      tmp_theta = teb.Pose(i).theta();
      memcpy(msg_.buffer+8*i,&tmp_x,sizeof(float));
      memcpy(msg_.buffer+8*i+4,&tmp_y,sizeof(float));
      ++i;
      if(8*i >= MESSAGE_SIZE){break;}
      // printf("cordinate (%f,%f)\r\n",tmp_x,tmp_y);
    }
    msg_.len = 8*i;
    // printf("msg_.len=%d size=%d\r\n",msg_.len,teb.sizePoses());
    // setMessage(&msg, (char*)&data, MESSAGE_SIZE, mtype);
    // int ret = messageQueueSend(qid_, &msg_);//队列满时会堵塞程序
    int ret = messageQueueSend_nowait(qid_, &msg_);  
    TEB_DEBUG_LOG("publishLocalPlanAndPoses ret=%d",ret);
}



void TebVisualization::publishRobotFootprintModel(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model, const std::string& ns,
                                                  const std_msgs::ColorRGBA &color)
{
  // if ( printErrorWhenNotInitialized() )
  //   return;
  
  // std::vector<visualization_msgs::Marker> markers;
  // robot_model.visualizeRobot(current_pose, markers, color);
  // if (markers.empty())
  //   return;
  
  // int idx = 1000000;  // avoid overshadowing by obstacles
  // for (std::vector<visualization_msgs::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
  // {
  //   marker_it->header.frame_id = cfg_->map_frame;
  //   marker_it->header.stamp = ros::Time::now();
  //   marker_it->action = visualization_msgs::Marker::ADD;
  //   marker_it->ns = ns;
  //   marker_it->id = idx;
  //   marker_it->lifetime = ros::Duration(2.0);
  //   teb_marker_pub_.publish(*marker_it);
  // }
  
}

void TebVisualization::publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model)
{
  // publishRobotFootprintModel(current_pose, robot_model, "InfeasibleRobotPoses", toColorMsg(0.5, 0.8, 0.0, 0.0));
}


void TebVisualization::publishObstacles(const ObstContainer& obstacles) const
{
  if ( obstacles.empty() || printErrorWhenNotInitialized() )
    return;
  
  int i,ret;
  // TEB_INFO_LOG("send obstacle");
  ret = semWait_timeout(semid_, OBSTACLE_SEMNUM, 10000);//等10ms
  if(ret == -1)
  {
    TEB_INFO_LOG(" obstacles Can't wait a sem[%d].",semGetValue(semid_, OBSTACLE_SEMNUM));
    return;
  }//等待超时就退出
  sharedMemoryLock(obstacle_shmid_);
  uint32_t state = 0;
  i = 0;
  float tmp_x,tmp_y;
  for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
  {
    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(*obst);      
    if (!pobst)
      continue;
    tmp_x = pobst->x();
    tmp_y = pobst->y();
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
  // TEB_INFO_LOG("obstacle plan size=%d ",8*i);
}


void TebVisualization::publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
{
//   if ( via_points.empty() || printErrorWhenNotInitialized() )
//     return;
  
//   visualization_msgs::Marker marker;
//   marker.header.frame_id = cfg_->map_frame;
//   marker.header.stamp = ros::Time::now();
//   marker.ns = ns;
//   marker.id = 0;
//   marker.type = visualization_msgs::Marker::POINTS;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.lifetime = ros::Duration(2.0);
  
//   for (std::size_t i=0; i < via_points.size(); ++i)
//   {
//     geometry_msgs::Point point;
//     point.x = via_points[i].x();
//     point.y = via_points[i].y();
//     point.z = 0;
//     marker.points.push_back(point);
//   }
  
//   marker.scale.x = 0.1;
//   marker.scale.y = 0.1;
//   marker.color.a = 1.0;
//   marker.color.r = 0.0;
//   marker.color.g = 0.0;
//   marker.color.b = 1.0;

//   teb_marker_pub_.publish( marker );
 }



void TebVisualization::publishTebContainer(const TebOptPlannerContainer& teb_planner)
{
  if ( printErrorWhenNotInitialized() )
      return;

  int i,ret;
  // TEB_INFO_LOG("send global plan");
  ret = semWait_timeout(semid_, TEBCONTAINER_SEMNUM, 10000);//等10ms
  if(ret == -1)
  {
    TEB_INFO_LOG("teb container Can't wait a sem[%d].",semGetValue(semid_, TEBCONTAINER_SEMNUM));
    return;
  }//等待超时就退出
  sharedMemoryLock(teb_container_shmid_);
  uint32_t state = 0;
  uint32_t teb_count = 0;

  float tmp_x,tmp_y;
  i = 0;

  TEB_DEBUG_LOG("teb_planner Container size = %d",teb_planner.size());
  // Iterate through teb pose sequence
  for( TebOptPlannerContainer::const_iterator it_teb = teb_planner.begin(); it_teb != teb_planner.end(); ++it_teb )
  {	  
    // iterate single poses
    PoseSequence::const_iterator it_pose = it_teb->get()->teb().poses().begin();
    TimeDiffSequence::const_iterator it_timediff = it_teb->get()->teb().timediffs().begin();
    PoseSequence::const_iterator it_pose_end = it_teb->get()->teb().poses().end();
    std::advance(it_pose_end, -1); // since we are interested in line segments, reduce end iterator by one. 注释该句会报segdefault
    double time = 0;

    while (it_pose != it_pose_end)
    {

      tmp_x = (*it_pose)->x();
      tmp_y = (*it_pose)->y();
      memcpy(teb_container_shmaddr_+4+8*i,&tmp_x,sizeof(float));
      memcpy(teb_container_shmaddr_+4+8*i+4,&tmp_y,sizeof(float));
      ++i;
      if(8*i >= TEBCONTAINER_SHMEMSIZE)
      {
        TEB_WARN_LOG("teb container size GREATER than TEBCONTAINER_SHMEMSIZE");
        break;
      }

      time += (*it_timediff)->dt();

      ++it_pose;
      ++it_timediff;
      // TEB_DEBUG_LOG("tmp_x =%f tmp_y=%f",tmp_x,tmp_y);
    }

    ++teb_count;
    TEB_DEBUG_LOG("publish teb[%d] total time=%f",teb_count,time);
    uint32_t uint_tmp_x = 0xFFFFFFFF;
    uint32_t uint_tmp_y = 0xFFFFFFFF;
    memcpy(teb_container_shmaddr_+4+8*i,&uint_tmp_x,sizeof(uint32_t));
    memcpy(teb_container_shmaddr_+4+8*i+4,&uint_tmp_y,sizeof(uint32_t));
    ++i;
  }

  state = i;
  state = state | 0x80000000;//置最高位为1，表示数据有更新
  memcpy(teb_container_shmaddr_,&state,sizeof(uint32_t));
  sharedMemoryUnlock(teb_container_shmid_);
  semIncrement(semid_, TEBCONTAINER_SEMNUM, 1);

  
}

void TebVisualization::publishFeedbackMessage(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planners,
                                              unsigned int selected_trajectory_idx, const ObstContainer& obstacles)
{
  // FeedbackMsg msg;
  // msg.header.stamp = ros::Time::now();
  // msg.header.frame_id = cfg_->map_frame;
  // msg.selected_trajectory_idx = selected_trajectory_idx;
  
  
  // msg.trajectories.resize(teb_planners.size());
  
  // // Iterate through teb pose sequence
  // std::size_t idx_traj = 0;
  // for( TebOptPlannerContainer::const_iterator it_teb = teb_planners.begin(); it_teb != teb_planners.end(); ++it_teb, ++idx_traj )
  // {   
  //   msg.trajectories[idx_traj].header = msg.header;
  //   it_teb->get()->getFullTrajectory(msg.trajectories[idx_traj].trajectory);
  // }
  
  // // add obstacles
  // msg.obstacles_msg.obstacles.resize(obstacles.size());
  // for (std::size_t i=0; i<obstacles.size(); ++i)
  // {
  //   msg.obstacles_msg.header = msg.header;

  //   // copy polygon
  //   msg.obstacles_msg.obstacles[i].header = msg.header;
  //   obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

  //   // copy id
  //   msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

  //   // orientation
  //   //msg.obstacles_msg.obstacles[i].orientation =; // TODO

  //   // copy velocities
  //   obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  // }
  
  // feedback_pub_.publish(msg);
}

void TebVisualization::publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles)
{
  // FeedbackMsg msg;
  // msg.header.stamp = ros::Time::now();
  // msg.header.frame_id = cfg_->map_frame;
  // msg.selected_trajectory_idx = 0;
  
  // msg.trajectories.resize(1);
  // msg.trajectories.front().header = msg.header;
  // teb_planner.getFullTrajectory(msg.trajectories.front().trajectory);
 
  // // add obstacles
  // msg.obstacles_msg.obstacles.resize(obstacles.size());
  // for (std::size_t i=0; i<obstacles.size(); ++i)
  // {
  //   msg.obstacles_msg.header = msg.header;

  //   // copy polygon
  //   msg.obstacles_msg.obstacles[i].header = msg.header;
  //   obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

  //   // copy id
  //   msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

  //   // orientation
  //   //msg.obstacles_msg.obstacles[i].orientation =; // TODO

  //   // copy velocities
  //   obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  // }
  
  // feedback_pub_.publish(msg);
}

std_msgs::ColorRGBA TebVisualization::toColorMsg(double a, double r, double g, double b)
{
  std_msgs::ColorRGBA color;
  color.a = a;
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

bool TebVisualization::printErrorWhenNotInitialized() const
{
  if (!initialized_)
  {
    TEB_ERROR_LOG("TebVisualization class not initialized. You must call initialize or an appropriate constructor");
    return true;
  }
  return false;
}

} // namespace teb_local_planner
