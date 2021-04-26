/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
extern "C" {
  #include "dubins.h"
}
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#include <vector>
#include "../include/carrot_planner.h"
#include <pluginlib/class_list_macros.h>

using namespace std;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(carrot_planner::CarrotPlanner, nav_core::BaseGlobalPlanner)

//Dubins Curves Utils
vector<vector<double>> vec;
vector<vector<double>> result_vec;

int appendresult(double q[3], double x, void* user_data) {
  vector<double> v={q[0],q[1],q[2]};
  vec.push_back(v);
  return 0;
}
vector<vector<double>> gen_path(double q0[] , double q1[], int turning_radius){
    DubinsPath path;
    dubins_shortest_path(&path, q0, q1, 1.0);
    
    printf("#x,y,theta,t\n");
    // dubins_path_sample_many(&path,  0.1, printConfiguration, NULL);
    dubins_path_sample_many(&path,  0.3, appendresult, NULL);
    return vec;
}  

namespace carrot_planner {

  CarrotPlanner::CarrotPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  CarrotPlanner::CarrotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }
  
  void CarrotPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
//   double CarrotPlanner::footprintCost(double x_i, double y_i, double theta_i){
//     if(!initialized_){
//       ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
//       return -1.0;
//     }

//     std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    // if we have no footprint... do nothing
//     if(footprint.size() < 3)
//       return -1.0;

    //check if the footprint is legal
//     double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
//     return footprint_cost;
//   }
    //Utils functions
    double* q2rpy(const geometry_msgs::PoseStamped& point)
    {
      double roll, pitch, yaw;
      tf::Quaternion q(
        point.pose.orientation.x,
        point.pose.orientation.y,
        point.pose.orientation.z,
        point.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        static double orient_arr[] = {roll,pitch,yaw};
        return orient_arr;
    }
    void print_plan(std::vector<geometry_msgs::PoseStamped>& plan)
    {
      double roll,pitch,yaw;
      double *orient_ptr;
      for(int i=0;i<plan.size();++i)
      {
        cout<<"Translation"<<endl;
        cout<<plan[i].pose.position.x<<" "<<plan[i].pose.position.y<<" "<<plan[i].pose.position.z<<endl;
        cout<<"Orientation"<<endl;
        orient_ptr = q2rpy(plan[i]);
        cout<<orient_ptr[0]<<" "<<orient_ptr[1]<<" "<<orient_ptr[2]<<endl; //Corresponds to yaw, pitch and roll
      }
    }

    bool CarrotPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        plan.push_back(start);

        //Extract yaw from quaternion information
        double start_roll,start_pitch,start_yaw;
        double goal_roll,goal_pitch,goal_yaw;
        
        tf::Quaternion start_q(
        start.pose.orientation.x,
        start.pose.orientation.y,
        start.pose.orientation.z,
        start.pose.orientation.w);
        tf::Matrix3x3 start_m(start_q);
        start_m.getRPY(start_roll, start_pitch, start_yaw);

        tf::Quaternion goal_q(
        goal.pose.orientation.x,
        goal.pose.orientation.y,
        goal.pose.orientation.z,
        goal.pose.orientation.w);
        tf::Matrix3x3 goal_m(goal_q);
        goal_m.getRPY(goal_roll, goal_pitch, goal_yaw);
        // cout<<"yaw angle"<<yaw<<endl;

        //Start position
        double q0[] = {start.pose.position.x,start.pose.position.y,start_yaw};
        //Goal position
        double q1[] = {goal.pose.position.x,goal.pose.position.y,goal_yaw};
        result_vec = gen_path(q0,q1,1.0);//TODO: Don't hardcode the turning radius
        geometry_msgs::PoseStamped new_goal = goal; 
        //Output
        for(int i=0;i<result_vec.size();++i)
        {
          new_goal.pose.position.x = result_vec[i][0];
          new_goal.pose.position.y = result_vec[i][1];
          new_goal.pose.position.z = 0.0;

          //New Goal Orientation
          tf::Quaternion goal_quat = tf::createQuaternionFromYaw(result_vec[i][2]);
          new_goal.pose.orientation.x = goal_quat.x();
          new_goal.pose.orientation.y = goal_quat.y();
          new_goal.pose.orientation.z = goal_quat.z();
          new_goal.pose.orientation.w = goal_quat.w();

          plan.push_back(new_goal);

        }

        // for (int i=0; i<20; i++){
        // geometry_msgs::PoseStamped new_goal = goal;
        // tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
    
        // new_goal.pose.position.x = -2.5+(0.05*i);
        // new_goal.pose.position.y = -3.5+(0.05*i);
    
        // new_goal.pose.orientation.x = goal_quat.x();
        // new_goal.pose.orientation.y = goal_quat.y();
        // new_goal.pose.orientation.z = goal_quat.z();
        // new_goal.pose.orientation.w = goal_quat.w();
    
        // plan.push_back(new_goal);
        // }
        plan.push_back(goal);
        print_plan(plan);
    return true;
    }
  };