/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
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

#include <teb_planner/teb_planner.h>
#include "thread_pool/ThreadPool.h"
#include <thread>
#include <chrono> //秒表
// #include <interactive_markers/interactive_marker_server.h>
// #include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "timer/sleep_timer.h"
#include "timer/timer.h"
#include <stdint.h>
#include <stdio.h>
#include "misc/robot_config.h"

using namespace planning_planner; // it is ok here to import everything for testing purposes
using namespace planning_utils; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;

// ros::Subscriber custom_obst_sub;
// ros::Subscriber via_points_sub;
// ros::Subscriber clicked_points_sub;
// std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

// =========== Function declarations =============
// void CB_mainCycle(const ros::TimerEvent& e);
// void CB_publishCycle(const ros::TimerEvent& e);

// void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
// void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
// void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
// void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
ConfigRobot *g_robot_cfg;




// =============== Main function =================
int main( int argc, char** argv )
{
  // ros::init(argc, argv, "test_optim_node");
  // ros::NodeHandle n("~");
 
 
  // ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  // ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  

  

  
  // setup callback for clicked points (in rviz) that are considered as via-points
  // clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);
  


  // interactive marker server for simulated dynamic obstacles
  // // interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");
  // for(int i = 0;i < 20;i++)
  // {
  //   obst_vector.push_back( boost::make_shared<PointObstacle>(-4+0.1*i,1) );
  //   // obst_vector.push_back( boost::make_shared<PointObstacle>(6,2) );
  //   obst_vector.push_back( boost::make_shared<PointObstacle>(0+0.1*i,0.1) );


  // }
  obst_vector.clear();
   obst_vector.push_back( boost::make_shared<PointObstacle>(-3,1) );
    obst_vector.push_back( boost::make_shared<PointObstacle>(6,2) );
    obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );


//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,1.5,1,-1.5) ); //90 deg
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,0,-1,0) ); //180 deg
//  obst_vector.push_back( boost::make_shared<PointObstacle>(-1.5,-0.5) );

  // Dynamic obstacles
  // Eigen::Vector2d vel (0.1, -0.3);
  // obst_vector.at(0)->setCentroidVelocity(vel);
  // vel = Eigen::Vector2d(-0.3, -0.2);
  // obst_vector.at(1)->setCentroidVelocity(vel);

  /*
  PolygonObstacle* polyobst = new PolygonObstacle;
  polyobst->pushBackVertex(1, -1);
  polyobst->pushBackVertex(0, 1);
  polyobst->pushBackVertex(1, 1);
  polyobst->pushBackVertex(2, 1);
 
  polyobst->finalizePolygon();
  obst_vector.emplace_back(polyobst);
  */
  
  // for (unsigned int i=0; i<obst_vector.size(); ++i)
  // {

  //   //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);  
  //   // Add interactive markers for all point obstacles
  //   boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
  //   if (pobst)
  //   {
  //     CreateInteractiveMarker(pobst->x(),pobst->y(),i, config.map_frame, &marker_server, &CB_obstacle_marker);  
  //   }
  // }
  // marker_server.applyChanges();
  via_points.clear();
  // via_points.push_back( Eigen::Vector2d( 0, 0 ) );


  ConfigManager cfg_mgr;
  // cfg_mgr.LoadConfig("/home/kevin/k905/planning/config/planning.yaml");
  cfg_mgr.LoadConfig("/tmp/config/planning.yaml");

  config.init(cfg_mgr);
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(config));
  
  // Setup robot shape model
  RobotFootprintModelPtr robot_model = TebPlanner::getRobotFootprintFromParamServer();
  printf("it's here.\r\n");
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
  {
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
    printf("it's here2.\r\n");
  }
  else
  {
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
    printf("it's here3.\r\n");
  }
  
  printf("fist obstacla num=%d \r\n",no_fixed_obstacles);
  no_fixed_obstacles = obst_vector.size();
  printf("obstacla num=%d \r\n",no_fixed_obstacles);
  // SleepTimer t(5);
  uint8_t count = 0;

  // ThreadPool pool(5);

  // TebPlannerPtr g_teb_planner = TebPlanner::getInstance(&pool);
  // g_teb_planner->init(cfg_mgr);

  printf("planning=%d\r\n",config.hcp.enable_homotopy_class_planning);
  while(1)
  {
    count++;
    // uint64_t last_ts = Timer::getSystemTimestampUS();
    struct timeval tv;
    gettimeofday(&tv,NULL);
    uint64_t timer3 = tv.tv_sec*1000 + tv.tv_usec/1000;

    planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0));

    gettimeofday(&tv,NULL);
    uint64_t timer4 = tv.tv_sec*1000 + tv.tv_usec/1000;

      // uint64_t ts = Timer::getSystemTimestampUS();


    printf("consume time=%ld ms\r\n",timer4-timer3);
    // if(count > 10)
    // {
    //   count = 0;
      planner->visualize();
      visual->publishObstacles(obst_vector);
      
    // }

    printf("it is running.\r\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // t.sleep();
  }


  // ros::spin();

  return 0;
}

// Planning loop
// void CB_mainCycle(const ros::TimerEvent& e)
// {
//   planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0)); // hardcoded start and goal for testing purposes
// }



// void testpublishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
// {
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
//  }



// Visualization loop
// void CB_publishCycle(const ros::TimerEvent& e)
// {
//   planner->visualize();
//   visual->publishObstacles(obst_vector);
//   // visual->publishViaPoints(via_points);
// }


// void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
// {
//   // create an interactive marker for our server
//   visualization_msgs::InteractiveMarker i_marker;
//   i_marker.header.frame_id = frame;
//   i_marker.header.stamp = ros::Time::now();
//   std::ostringstream oss;
//   //oss << "obstacle" << id;
//   oss << id;
//   i_marker.name = oss.str();
//   i_marker.description = "Obstacle";
//   i_marker.pose.position.x = init_x;
//   i_marker.pose.position.y = init_y;
//   i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

//   // create a grey box marker
//   visualization_msgs::Marker box_marker;
//   box_marker.type = visualization_msgs::Marker::CUBE;
//   box_marker.id = id;
//   box_marker.scale.x = 0.2;
//   box_marker.scale.y = 0.2;
//   box_marker.scale.z = 0.2;
//   box_marker.color.r = 0.5;
//   box_marker.color.g = 0.5;
//   box_marker.color.b = 0.5;
//   box_marker.color.a = 1.0;
//   box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

//   // create a non-interactive control which contains the box
//   visualization_msgs::InteractiveMarkerControl box_control;
//   box_control.always_visible = true;
//   box_control.markers.push_back( box_marker );

//   // add the control to the interactive marker
//   i_marker.controls.push_back( box_control );

//   // create a control which will move the box, rviz will insert 2 arrows
//   visualization_msgs::InteractiveMarkerControl move_control;
//   move_control.name = "move_x";
//   move_control.orientation.w = 0.707107f;
//   move_control.orientation.x = 0;
//   move_control.orientation.y = 0.707107f;
//   move_control.orientation.z = 0;
//   move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


//   // add the control to the interactive marker
//   i_marker.controls.push_back(move_control);

//   // add the interactive marker to our collection
//   marker_server->insert(i_marker);
//   marker_server->setCallback(i_marker.name,feedback_cb);
// }

// void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
// {
//   std::stringstream ss(feedback->marker_name);
//   unsigned int index;
//   ss >> index;
  
//   if (index>=no_fixed_obstacles) 
//     return;
//   PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
//   pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
// }



// void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
// {
//   // we assume for simplicity that the fixed frame is already the map/planning frame
//   // consider clicked points as via-points
//   via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
//   ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
//   if (config.optim.weight_viapoint<=0)
//     ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
// }



