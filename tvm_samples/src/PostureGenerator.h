/* Author: Masaki Murooka */

#ifndef POSTURE_GENERATOR_
#define POSTURE_GENERATOR_

#include <string>
#include <iostream>
#include <cstdio>
#include <vector>
#include <memory>
#include <map>

#include <ros/ros.h>
#include <ros/console.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <jsk_rviz_plugins/OverlayText.h>

#include "TvmHeaders.h"


namespace tvm_samples
{
inline constexpr double radToDeg(double rad)
{
  return (180.0 / M_PI) * rad;
}

inline constexpr double degToRad(double deg)
{
  return (M_PI / 180.0) * deg;
}

inline geometry_msgs::Pose toRosMsg(const sva::PTransformd& trans)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(trans.translation(), pose_msg.position);
  // rotation should be inversed: https://github.com/jrl-umi3218/tvm/issues/14#issuecomment-654075023
  tf::quaternionEigenToMsg(Eigen::Quaterniond(trans.rotation()).inverse(), pose_msg.orientation);
  return pose_msg;
}

/** \brief Build a cube as a set of planes from a given origin and size */
inline std::vector<tvm::geometry::PlanePtr> makeCube(
    const Eigen::Vector3d & origin,
    double x_size,
    double y_size,
    double z_size)
{
  return {
    std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{1, 0, 0}, origin + Eigen::Vector3d{-x_size, 0, 0}),
        std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{-1, 0, 0}, origin + Eigen::Vector3d{x_size, 0, 0}),
        std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{0, 1, 0}, origin + Eigen::Vector3d{0, -y_size, 0}),
        std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{0, -1, 0}, origin + Eigen::Vector3d{0, y_size, 0}),
        std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{0, 0, 1}, origin + Eigen::Vector3d{0, 0, -z_size}),
        std::make_shared<tvm::geometry::Plane>(Eigen::Vector3d{0, 0, -1}, origin + Eigen::Vector3d{0, 0, z_size})
        };
}

class PostureGenerator
{
  using OriFuncPtr = std::shared_ptr<tvm::robot::OrientationFunction>;
  using PosFuncPtr = std::shared_ptr<tvm::robot::PositionFunction>;

 public:
  PostureGenerator()
  {
    setupRos(); // setupRos should be called before setupRobot for initial_q_
    setupRobot();
    setupTask();
  }

  void resetRobot();

  bool run();

  void publishPoseArray() const;

  void publishRobotState() const;

  void publishTaskState() const;

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_st_msg);

  bool resetCallback(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &res);


  inline void pushToFrameMap(const std::shared_ptr<tvm::robot::Frame>& frame)
  {
    frame_map_[frame->name()] = frame;
  }

  inline void pushToContactMap(const std::string& name,
                               const std::shared_ptr<tvm::robot::Contact>& contact)
  {
    contact_map_[name] = contact;
  }

  inline void pushToEefFuncMap(const std::string& name)
  {
    auto ori_fn = std::make_shared<tvm::robot::OrientationFunction>(frame_map_[name]);
    auto pos_fn = std::make_shared<tvm::robot::PositionFunction>(frame_map_[name]);
    eef_func_map_[name] = std::make_pair(ori_fn, pos_fn);
  }

  inline void pushToTaskMap(const std::string& name,
                            const tvm::TaskWithRequirementsPtr& task)
  {
    task_map_[name] = task;
  }

  void printFramesInfo() const;

  void printTasksInfo() const;

 protected:
  void setupRobot();

  void setupTask();

  void setupRos();

 public:
  tvm::ControlProblem pb_;
  std::shared_ptr<tvm::Clock> clock_;

  tvm::RobotPtr robot_;
  tvm::RobotPtr env_;

  std::map<std::string, std::vector<double> > initial_q_;

  std::map<std::string, std::shared_ptr<tvm::robot::Frame> > frame_map_;

  std::map<std::string, std::shared_ptr<tvm::robot::Contact> > contact_map_;

  std::map<std::string, std::pair<OriFuncPtr, PosFuncPtr> > eef_func_map_;

  std::map<std::string, tvm::TaskWithRequirementsPtr> task_map_;

  bool solve_result_ = false;

 protected:
  // todo: understand what dt means in the case of the kinematics only problem like this class
  double dt_ = 1.0; // [sec]
  bool publish_in_loop_ = true;
  double sleep_duration_in_loop_ = -1;
  double ori_thre_ = degToRad(1);
  double pos_thre_ = 1e-3;

  bool reset_ = false;

  ros::NodeHandle nh_;
  ros::Publisher pose_arr_pub_;
  ros::Publisher robot_state_pub_;
  ros::Publisher text_pub_;
  ros::Subscriber pose_sub_;
  ros::ServiceServer reset_srv_;
};
}

#endif
