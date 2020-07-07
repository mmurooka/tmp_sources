/* Author: Masaki Murooka */

#include <tvm/Robot.h>
#include <tvm/robot/internal/GeometricContactFunction.h>
#include <tvm/robot/internal/DynamicFunction.h>
#include <tvm/robot/CollisionFunction.h>
#include <tvm/robot/CoMFunction.h>
#include <tvm/robot/CoMInConvexFunction.h>
#include <tvm/robot/ConvexHull.h>
#include <tvm/robot/JointsSelector.h>
#include <tvm/robot/OrientationFunction.h>
#include <tvm/robot/PositionFunction.h>
#include <tvm/robot/PostureFunction.h>
#include <tvm/robot/utils.h>
#include <tvm/Task.h>

#include <tvm/Clock.h>
#include <tvm/ControlProblem.h>
#include <tvm/LinearizedControlProblem.h>
#include <tvm/function/IdentityFunction.h>
#include <tvm/hint/Substitution.h>
#include <tvm/scheme/WeightedLeastSquares.h>
#include <tvm/solver/defaultLeastSquareSolver.h>
#include <tvm/solver/QuadprogLeastSquareSolver.h>
#include <tvm/task_dynamics/None.h>
#include <tvm/task_dynamics/Proportional.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>
#include <tvm/task_dynamics/VelocityDamper.h>
#include <tvm/utils/sch.h>

#include <RBDyn/parsers/urdf.h>

#include <RBDyn/ID.h>
#include <RBDyn/EulerIntegration.h>

#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <moveit_msgs/DisplayRobotState.h>


class TvmIkSample
{
 public:
  TvmIkSample():
      clock_(std::make_shared<tvm::Clock>(dt_))
  {
    setupRobot();
    setupTask();
    setupRos();
  }

  void run()
  {
    solve();
  }

 protected:
  void setupRobot()
  {
    // setup robot
    if (!nh_.hasParam("robot_model_path")) {
      throw std::invalid_argument("robot_model_path is not set.");
    }
    std::string robot_model_path;
    nh_.getParam("robot_model_path", robot_model_path);

    if (!nh_.hasParam("robot/initial_q")) {
      throw std::invalid_argument("robot/initial_q is not set.");
    }
    std::map<std::string, std::vector<double>> initial_q;
    XmlRpc::XmlRpcValue initial_q_param;
    nh_.getParam("robot/initial_q", initial_q_param);
    for (int i = 0; i < initial_q_param.size(); i++) {
      XmlRpc::XmlRpcValue initial_q_param_sub = initial_q_param[i];
      std::vector<double> q;
      if (initial_q_param_sub[1].getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int j = 0; j < initial_q_param_sub[1].size(); j++) {
          q.push_back(static_cast<double>(initial_q_param_sub[1][j]));
        }
      } else {
        q.push_back(static_cast<double>(initial_q_param_sub[1]));
      }
      std::string name = static_cast<std::string>(initial_q_param_sub[0]);
      initial_q[name] = q;
    }

    if (!nh_.hasParam("robot/filtered_links")) {
      throw std::invalid_argument("robot/filtered_links is not set.");
    }
    std::vector<std::string> filtered_link_names;
    nh_.getParam("robot/filtered_links", filtered_link_names);

    robot_ = tvm::robot::fromURDF(
        *clock_,
        "robot",
        robot_model_path,
        false,
        filtered_link_names,
        initial_q);

    env_ = tvm::robot::fromURDF(
        *clock_,
        "env",
        ros::package::getPath("mc_env_description") + "/urdf/ground.urdf",
        true,
        {},
        {});

    // setup frame
    std::string left_foot_link_name;
    std::string right_foot_link_name;
    std::string left_hand_link_name;
    std::string right_hand_link_name;
    if (!(nh_.hasParam("frame/left_foot_link")
          || nh_.hasParam("frame/right_foot_link")
          || nh_.hasParam("frame/left_hand_link")
          || nh_.hasParam("frame/right_hand_link"))) {
      throw std::invalid_argument("foot and hand link is not set.");
    }
    nh_.getParam("frame/left_foot_link", left_foot_link_name);
    nh_.getParam("frame/right_foot_link", right_foot_link_name);
    nh_.getParam("frame/left_hand_link", left_hand_link_name);
    nh_.getParam("frame/right_hand_link", right_hand_link_name);

    Eigen::Vector3d left_foot_offset = Eigen::Vector3d::Zero();
    if (nh_.hasParam("frame/left_foot_offset")) {
      std::vector<double> left_foot_offset_vec;
      nh_.getParam("frame/left_foot_offset", left_foot_offset_vec);
      left_foot_offset = Eigen::Vector3d(left_foot_offset_vec.data());
    }
    Eigen::Vector3d right_foot_offset = Eigen::Vector3d::Zero();
    if (nh_.hasParam("frame/right_foot_offset")) {
      std::vector<double> right_foot_offset_vec;
      nh_.getParam("frame/right_foot_offset", right_foot_offset_vec);
      right_foot_offset = Eigen::Vector3d(right_foot_offset_vec.data());
    }
    Eigen::Vector3d left_hand_offset = Eigen::Vector3d::Zero();
    if (nh_.hasParam("frame/left_hand_offset")) {
      std::vector<double> left_hand_offset_vec;
      nh_.getParam("frame/left_hand_offset", left_hand_offset_vec);
      left_hand_offset = Eigen::Vector3d(left_hand_offset_vec.data());
    }
    Eigen::Vector3d right_hand_offset = Eigen::Vector3d::Zero();
    if (nh_.hasParam("frame/right_hand_offset")) {
      std::vector<double> right_hand_offset_vec;
      nh_.getParam("frame/right_hand_offset", right_hand_offset_vec);
      right_hand_offset = Eigen::Vector3d(right_hand_offset_vec.data());
    }

    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "LeftFootFrame",
            robot_,
            left_foot_link_name,
            sva::PTransformd{left_foot_offset}));
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "RightFootFrame",
            robot_,
            right_foot_link_name,
            sva::PTransformd{right_foot_offset}));
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "LeftHand",
            robot_,
            left_hand_link_name,
            sva::PTransformd{left_hand_offset}));
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "RightHand",
            robot_,
            right_hand_link_name,
            sva::PTransformd{right_hand_offset}));
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "GroundFrame",
            env_,
            "ground",
            sva::PTransformd::Identity()));

    // setup contact
    pushToContactMap(
        "LeftFootGround",
        std::make_shared<tvm::robot::Contact>(
            frame_map_["LeftFootFrame"], frame_map_["GroundFrame"], std::vector<sva::PTransformd>{
              {Eigen::Vector3d(0.1, -0.07, 0.)},
              {Eigen::Vector3d(0.1, 0.07, 0.)},
              {Eigen::Vector3d(-0.1, 0.07, 0.)},
              {Eigen::Vector3d(-0.1, -0.07, 0.)}
            })) ;
    pushToContactMap(
        "RightFootGround",
        std::make_shared<tvm::robot::Contact>(
            frame_map_["RightFootFrame"], frame_map_["GroundFrame"], std::vector<sva::PTransformd>{
              {Eigen::Vector3d(-0.1, 0.07, 0.)},
              {Eigen::Vector3d(-0.1, -0.07, 0.)},
              {Eigen::Vector3d(0.1, -0.07, 0.)},
              {Eigen::Vector3d(0.1, 0.07, 0.)}
            }));
  }

  void setupTask()
  {
    // define function
    auto left_foot_contact_fn = std::make_shared<tvm::robot::internal::GeometricContactFunction>(
        contact_map_["LeftFootGround"], Eigen::Matrix6d::Identity());
    auto right_foot_contact_fn = std::make_shared<tvm::robot::internal::GeometricContactFunction>(
        contact_map_["RightFootGround"], Eigen::Matrix6d::Identity());
    auto posture_fn = std::make_shared<tvm::robot::PostureFunction>(robot_);

    auto com_in_fn = std::make_shared<tvm::robot::CoMInConvexFunction>(robot_);
    auto cube = makeCube(robot_->com(), 0.05, 0.05, 1.0);
    for (const auto& p : cube) {
      com_in_fn->addPlane(p);
    }

    left_hand_ori_fn_ = std::make_shared<tvm::robot::OrientationFunction>(frame_map_["LeftHand"]);
    left_hand_pos_fn_ = std::make_shared<tvm::robot::PositionFunction>(frame_map_["LeftHand"]);

    // add task to problem
    pb_.add(left_foot_contact_fn == 0.,
            tvm::task_dynamics::P(1.),
      {tvm::requirements::PriorityLevel(0)});
    pb_.add(right_foot_contact_fn == 0.,
            tvm::task_dynamics::P(1.),
      {tvm::requirements::PriorityLevel(0)});
    pb_.add(posture_fn == 0.,
            tvm::task_dynamics::P(1.),
      {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1e-4)});
    pb_.add(left_hand_ori_fn_ == 0.,
            tvm::task_dynamics::P(1.),
      {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1.)});
    pb_.add(left_hand_pos_fn_ == 0.,
            tvm::task_dynamics::P(1.),
      {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1.)});
    pb_.add(com_in_fn >= 0.,
            tvm::task_dynamics::VelocityDamper({0.005, 0.001, 0}, tvm::constant::big_number),
            {tvm::requirements::PriorityLevel(0)});

    // set bounds
    pb_.add(robot_->lQBound() <= robot_->qJoints() <= robot_->uQBound(),
            tvm::task_dynamics::VelocityDamper({0.01, 0.001, 0}, tvm::constant::big_number),
            {tvm::requirements::PriorityLevel(0)});
    // pb_.add(robot_->lTauBound() <= robot_->tau() <= robot_->uTauBound(),
    //         tvm::task_dynamics::None(),
    //         {tvm::requirements::PriorityLevel(0)});

    // regularization
    pb_.add(dot(robot_->qFreeFlyer()) == 0.,
      {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1e-6)});
    pb_.add(dot(robot_->qJoints()) == 0.,
      {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1e-6)});
  }

  void setupRos()
  {
    robot_state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);
    pose_arr_pub_ = nh_.advertise<geometry_msgs::PoseArray>("robot_pose_array", 1);

    pose_sub_ = nh_.subscribe("interactive_marker_pose", 1, &TvmIkSample::poseCallback, this);
  }

  void solve()
  {
    tvm::LinearizedControlProblem lpb(pb_);

    tvm::scheme::WeightedLeastSquares solver(tvm::solver::DefaultLSSolverOptions{});

    // loop
    ros::Rate rate(static_cast<int>(1.0 / dt_));
    while (ros::ok()) {
      // solve
      bool res = solver.solve(lpb);
      if (!res) {
        ROS_ERROR("Solver failed");
        break;
      }

      // integrate
      // Clock::advance is available only for the second order case (i.e. ddot(q) is variable).
      // clock_->advance();
      auto dq = dot(robot_->q()).value();
      rbd::vectorToParam(dq, robot_->mbc().alpha);
      rbd::eulerIntegration(robot_->mb(), robot_->mbc(), clock_->dt());
      auto q = robot_->q().value();
      rbd::paramToVector(robot_->mbc().q, q);

      // process ROS
      publishRobotState(*robot_);
      publishPose(*robot_);
      ros::spinOnce();
      // rate.sleep();
    }
  }

  void publishRobotState(const tvm::Robot& robot)
  {
    moveit_msgs::DisplayRobotState robot_state_msg;

    // set joint position
    for (const auto& joint : robot.mb().joints()) {
      if (joint.dof() == 1) {
        int joint_idx = robot.mb().jointIndexByName(joint.name());
        double joint_pos = robot.mbc().q[joint_idx][0];

        // printf("%s: %lf\n", joint.name().c_str(), joint_pos);
        robot_state_msg.state.joint_state.name.push_back(joint.name());
        robot_state_msg.state.joint_state.position.push_back(joint_pos);
      }
    }

    // set root position
    const std::vector<double>& root_q = robot.mbc().q[0];
    robot_state_msg.state.multi_dof_joint_state.header.frame_id = "world";
    geometry_msgs::Transform root_trans_msg;
    root_trans_msg.rotation.w = root_q[0];
    root_trans_msg.rotation.x = root_q[1];
    root_trans_msg.rotation.y = root_q[2];
    root_trans_msg.rotation.z = root_q[3];
    root_trans_msg.translation.x = root_q[4];
    root_trans_msg.translation.y = root_q[5];
    root_trans_msg.translation.z = root_q[6];
    robot_state_msg.state.multi_dof_joint_state.transforms.push_back(root_trans_msg);
    robot_state_msg.state.multi_dof_joint_state.joint_names.push_back("world_joint");

    // publish
    robot_state_pub_.publish(robot_state_msg);
  }

  void publishPose(const tvm::Robot& robot)
  {
    geometry_msgs::PoseArray pose_arr_msg;
    pose_arr_msg.header.frame_id = "world";

    // set left hand
    geometry_msgs::Pose left_hand_pose_msg;
    const Eigen::Vector3d& left_hand_pos = frame_map_["LeftHand"]->position().translation();
    Eigen::Quaterniond left_hand_quat(frame_map_["LeftHand"]->position().rotation().inverse());
    left_hand_pose_msg.position.x = left_hand_pos.x();
    left_hand_pose_msg.position.y = left_hand_pos.y();
    left_hand_pose_msg.position.z = left_hand_pos.z();
    left_hand_pose_msg.orientation.w = left_hand_quat.w();
    left_hand_pose_msg.orientation.x = left_hand_quat.x();
    left_hand_pose_msg.orientation.y = left_hand_quat.y();
    left_hand_pose_msg.orientation.z = left_hand_quat.z();
    pose_arr_msg.poses.push_back(left_hand_pose_msg);

    // set com
    geometry_msgs::Pose com_pose_msg;
    const Eigen::Quaterniond com_quat(frame_map_["LeftHand"]->position().rotation());
    com_pose_msg.position.x = robot.com().x();
    com_pose_msg.position.y = robot.com().y();
    com_pose_msg.position.z = robot.com().z();
    com_pose_msg.orientation.w = 1;
    com_pose_msg.orientation.x = 0;
    com_pose_msg.orientation.y = 0;
    com_pose_msg.orientation.z = 0;
    pose_arr_msg.poses.push_back(com_pose_msg);

    // publish
    pose_arr_pub_.publish(pose_arr_msg);
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_st_msg)
  {
    // update target
    left_hand_ori_fn_->orientation(
        Eigen::Quaterniond{
          pose_st_msg->pose.orientation.w,
              pose_st_msg->pose.orientation.x,
              pose_st_msg->pose.orientation.y,
              pose_st_msg->pose.orientation.z
              }.inverse().toRotationMatrix());
    left_hand_pos_fn_->position(
        Eigen::Vector3d{
          pose_st_msg->pose.position.x,
              pose_st_msg->pose.position.y,
              pose_st_msg->pose.position.z});
  }

  void pushToFrameMap(const std::shared_ptr<tvm::robot::Frame>& frame)
  {
    frame_map_[frame->name()] = frame;
  }

  void pushToContactMap(const std::string& name,
                        const std::shared_ptr<tvm::robot::Contact>& contact)
  {
    contact_map_[name] = contact;
  }

  // Build a cube as a set of planes from a given origin and size
  std::vector<tvm::geometry::PlanePtr> makeCube(const Eigen::Vector3d & origin,
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


  double dt_ = 0.005; // [sec]

  tvm::ControlProblem pb_;
  std::shared_ptr<tvm::Clock> clock_;

  tvm::RobotPtr robot_;
  tvm::RobotPtr env_;

  std::map<std::string, std::shared_ptr<tvm::robot::Frame> > frame_map_;
  std::map<std::string, std::shared_ptr<tvm::robot::Contact> > contact_map_;

  std::shared_ptr<tvm::robot::OrientationFunction> left_hand_ori_fn_;
  std::shared_ptr<tvm::robot::PositionFunction> left_hand_pos_fn_;

  ros::NodeHandle nh_;
  ros::Publisher robot_state_pub_;
  ros::Publisher pose_arr_pub_;
  ros::Subscriber pose_sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TvmIkSample");

  TvmIkSample sample;

  sample.run();
}
