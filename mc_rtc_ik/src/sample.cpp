#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <mc_solver/TasksQPSolver.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TransformTask.h>
#include <optmotiongen_msgs/RobotStateArray.h>

int main(int argc, char **argv)
{
  // Setup ROS
  ros::init(argc, argv, "mc_rtc_ik");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<optmotiongen_msgs::RobotStateArray>("robot_state_arr", 1, true);

  // Setup robot
  std::string robotName = "JVRC1";
  std::string urdfContent;
  nh.getParam("robot_description", urdfContent);
  mc_rbdyn::RobotsPtr robots = mc_rbdyn::loadRobotFromUrdf(
      robotName,
      urdfContent,
      rbd::parsers::ParserParameters().fixed(false));
  {
    std::vector<std::string> jname_list = {"R_HIP_P",      "R_HIP_R",      "R_HIP_Y",      "R_KNEE",       "R_ANKLE_R", "R_ANKLE_P",
                                           "L_HIP_P",      "L_HIP_R",      "L_HIP_Y",      "L_KNEE",       "L_ANKLE_R", "L_ANKLE_P",
                                           "WAIST_Y",      "WAIST_P",      "WAIST_R",      "NECK_Y",       "NECK_R",    "NECK_P",
                                           "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",    "R_ELBOW_Y", "R_WRIST_R",
                                           "R_WRIST_Y",    "R_UTHUMB",     "R_LTHUMB",     "R_UINDEX",     "R_LINDEX",  "R_ULITTLE",
                                           "R_LLITTLE",    "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_P", "L_ELBOW_Y",
                                           "L_WRIST_R",    "L_WRIST_Y",    "L_UTHUMB",     "L_LTHUMB",     "L_UINDEX",  "L_LINDEX",
                                           "L_ULITTLE",    "L_LLITTLE"};
    std::vector<double> jpos_list = {-0.38, -0.01, 0., 0.72, -0.01, -0.33,  -0.38, 0.02, 0.,    0.72, -0.02, -0.33, 0.,
                                     0.13,  0.,    0., 0.,   0.,    -0.052, -0.17, 0.,   -0.52, 0.,   0.,    0.,    0.,
                                     0.,    0.,    0., 0.,   0.,    -0.052, 0.17,  0.,   -0.52, 0.,   0.,    0.,    0.,
                                     0.,    0.,    0., 0.,   0.,    0.,     0.,    0.,   0.,    0.,   0.};
    const auto & mb = robots->robot().mb();
    auto & mbc = robots->robot().mbc();
    for(size_t i = 0; i < jname_list.size(); i++)
    {
      int jointIdx = mb.jointIndexByName(jname_list[i]);
      mbc.q[jointIdx][0] = jpos_list[i];
    }
    robots->robot().forwardKinematics();
    robots->robot().forwardVelocity();
  }

  // Setup solver
  constexpr double timeStep = 0.005; // [sec]
  std::shared_ptr<mc_solver::QPSolver> qpSolver = std::make_shared<mc_solver::TasksQPSolver>(robots, timeStep);
  if(qpSolver->backend() == mc_solver::QPSolver::Backend::Tasks)
  {
    mc_solver::tasks_solver(*qpSolver).updateNrVars();
  }

  // Add tasks
  const auto & postureTask = std::make_shared<mc_tasks::PostureTask>(*qpSolver, 0, 1.0, 1.0);
  const auto & leftFootTask = std::make_shared<mc_tasks::TransformTask>(
      robots->robot().frame("L_ANKLE_P_S"), 1000.0, 1000.0);
  const auto & rightFootTask = std::make_shared<mc_tasks::TransformTask>(
      robots->robot().frame("R_ANKLE_P_S"), 1000.0, 1000.0);
  const auto & rightHandTask = std::make_shared<mc_tasks::TransformTask>(
      robots->robot().frame("r_wrist"), 1000.0, 1000.0);
  qpSolver->addTask(postureTask);
  qpSolver->addTask(leftFootTask);
  qpSolver->addTask(rightFootTask);
  qpSolver->addTask(rightHandTask);
  leftFootTask->target(sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(15.0)), Eigen::Vector3d(0.0, 0.1, 0.0)));
  rightFootTask->target(sva::PTransformd(sva::RotZ(mc_rtc::constants::toRad(-15.0)), Eigen::Vector3d(0.0, -0.1, 0.0)));
  Eigen::VectorXd rightHandTaskWeight(6);
  rightHandTaskWeight << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
  rightHandTask->dimWeight(rightHandTaskWeight);

  // Loop
  ros::Rate rate(1000);
  int iter = 0;
  while(ros::ok())
  {
    double t = static_cast<double>(iter) * qpSolver->dt();

    rightHandTask->target(
        sva::PTransformd(Eigen::Vector3d(0.4 + 0.2 * std::cos(t), 0.2 + 0.2 * std::sin(t), 0.8)));

    // Solve IK
    if(!qpSolver->run())
    {
      std::cerr << "[mc_rtc_ik] QP failed." << std::endl;
    }

    // Publish robot
    {
      const auto & mb = robots->robot().mb();
      const auto & mbc = robots->robot().mbc();

      optmotiongen_msgs::RobotState robotStateMsg;
      robotStateMsg.name = robots->robot().name();

      // Set the joint positions
      for (const auto & joint : mb.joints()) {
        if (joint.params() == 1) {
          int jointIdx = mb.jointIndexByName(joint.name());
          double jpos = mbc.q[jointIdx][0];
          robotStateMsg.joint_name_list.push_back(joint.name());
          robotStateMsg.joint_pos_list.push_back(jpos);
        }
      }

      // Set the root pose
      robotStateMsg.root_pose.header.frame_id = "world";
      // Assuming that the 0th joint is the root joint (is this correct?), get the pose of its child link.
      // A robot with a fixed base has fixed joint 0th, so we can get pose in the same way.
      const sva::PTransformd & pose = mbc.bodyPosW[mb.successor(0)];
      geometry_msgs::Pose & poseMsg = robotStateMsg.root_pose.pose;
      tf::pointEigenToMsg(pose.translation(), poseMsg.position);
      tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.rotation()).normalized(), poseMsg.orientation);

      // Publish
      optmotiongen_msgs::RobotStateArray robotStateArrMsg;
      robotStateArrMsg.robot_states.push_back(robotStateMsg);
      pub.publish(robotStateArrMsg);
    }

    rate.sleep();

    iter++;
  }
}
