/* Author: Masaki Murooka */

#include "PostureGenerator.h"


using namespace tvm_samples;

void PostureGenerator::setupRos()
{
  // setup publisher
  pose_arr_pub_ = nh_.advertise<geometry_msgs::PoseArray>("posture_gen/pose_array", 1);
  robot_state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("posture_gen/display_robot_state", 1);
  text_pub_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("posture_gen/task_text", 1);

  // setup subscriber
  pose_sub_ = nh_.subscribe("interactive_marker_pose", 1, &PostureGenerator::poseCallback, this);

  // setup service server
  reset_srv_ = nh_.advertiseService("posture_gen/reset", &PostureGenerator::resetCallback, this);

  // setup param
  nh_.param("posture_gen/dt", dt_, dt_);
  nh_.param("posture_gen/publish_in_loop", publish_in_loop_, publish_in_loop_);
  nh_.param("posture_gen/sleep_duration_in_loop", sleep_duration_in_loop_, sleep_duration_in_loop_);
  nh_.param("posture_gen/ori_thre", ori_thre_, ori_thre_);
  nh_.param("posture_gen/pos_thre", pos_thre_, pos_thre_);

  if (!nh_.hasParam("robot/initial_q")) {
    throw std::invalid_argument("robot/initial_q is not set.");
  }
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
    initial_q_[name] = q;
  }
}

void PostureGenerator::setupRobot()
{
  // setup clock
  clock_ = std::make_shared<tvm::Clock>(dt_);

  // setup robot
  {
    if (!nh_.hasParam("robot_model_path")) {
      throw std::invalid_argument("robot_model_path is not set.");
    }
    std::string robot_model_path;
    nh_.getParam("robot_model_path", robot_model_path);

    std::vector<std::string> filtered_link_names;
    if (nh_.hasParam("robot/filtered_links")) {
      nh_.getParam("robot/filtered_links", filtered_link_names);
    }

    robot_ = tvm::robot::fromURDF(
        *clock_,
        "robot",
        robot_model_path,
        false,
        filtered_link_names,
        initial_q_);

    // make the environment as empty robot for making the fixed frames
    rbd::MultiBodyGraph empty_mbg;
    sva::RBInertiad empty_inertia;
    rbd::Body root_body(empty_inertia, "root");
    empty_mbg.addBody(root_body);
    rbd::MultiBody empty_mb = empty_mbg.makeMultiBody("root", true);
    rbd::MultiBodyConfig empty_mbc(empty_mb);
    env_ = std::make_shared<tvm::Robot>(
        *clock_,
        "env",
        empty_mbg,
        empty_mb,
        empty_mbc);
  }

  // setup frame
  XmlRpc::XmlRpcValue frames_param;
  nh_.getParam("robot/frames", frames_param);
  for (int i = 0; i < frames_param.size(); i++) {
    XmlRpc::XmlRpcValue frames_param_sub = frames_param[i];
    std::string name = static_cast<std::string>(frames_param_sub["name"]);
    std::string link_name = static_cast<std::string>(frames_param_sub["link"]);
    // pos offset
    Eigen::Vector3d pos_offset(Eigen::Vector3d::Zero());
    if (frames_param_sub.hasMember("pos_offset")) {
      if (frames_param_sub["pos_offset"].getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::invalid_argument("robot/frames/pos_offset should be array");
      } else if (frames_param_sub["pos_offset"].size() != 3) {
        throw std::invalid_argument("size of robot/frames/pos_offset should be 3");
      } else {
        for (int j = 0; j < 3; j++) {
          if (frames_param_sub["pos_offset"][j].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            pos_offset[j] = static_cast<int>(frames_param_sub["pos_offset"][j]);
          } else if (frames_param_sub["pos_offset"][j].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            pos_offset[j] = static_cast<double>(frames_param_sub["pos_offset"][j]);
          } else {
            throw std::invalid_argument("array element of robot/frames/pos_offset should be float");
          }
        }
      }
    }
    // rot offset
    Eigen::Vector3d rot_offset_axis(Eigen::Vector3d::Zero());
    double rot_offset_angle = 0;
    if (frames_param_sub.hasMember("rot_offset")) {
      if (frames_param_sub["rot_offset"].getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::invalid_argument("robot/frames/rot_offset should be array");
      } else if (frames_param_sub["rot_offset"].size() != 4) {
        throw std::invalid_argument("size of robot/frames/rot_offset should be 4 (axis and angle)");
      } else {
        for (int j = 0; j < 4; j++) {
          if (frames_param_sub["rot_offset"][j].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            if (j == 3) {
              rot_offset_angle = static_cast<int>(frames_param_sub["rot_offset"][j]);
            } else {
              rot_offset_axis[j] = static_cast<int>(frames_param_sub["rot_offset"][j]);
            }
          } else if (frames_param_sub["rot_offset"][j].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            if (j == 3) {
              rot_offset_angle = static_cast<double>(frames_param_sub["rot_offset"][j]);
            } else {
              rot_offset_axis[j] = static_cast<double>(frames_param_sub["rot_offset"][j]);
            }
          } else {
            throw std::invalid_argument("array element of robot/frames/rot_offset should be float");
          }
        }
      }
    }
    // push
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            name,
            robot_,
            link_name,
            sva::PTransformd(
                Eigen::AngleAxis(rot_offset_angle, rot_offset_axis).toRotationMatrix(),
                pos_offset)));
  }

  pushToFrameMap(
      std::make_shared<tvm::robot::Frame>(
          "GroundFrame",
          env_,
          "root",
          sva::PTransformd::Identity()));

  printFramesInfo();

  // setup contact
  // todo: set contact vertices from rosparam
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

  // setup eef
  // todo: add & remove eef function and task depending on target hand
  pushToEefFuncMap("LeftHandFrame");
  // pushToEefFuncMap("RightHandFrame");
}

void PostureGenerator::setupTask()
{
  // add contact task
  auto left_foot_contact_fn = std::make_shared<tvm::robot::internal::GeometricContactFunction>(
      contact_map_["LeftFootGround"], Eigen::Matrix6d::Identity());
  pushToTaskMap("LeftFootContactTask",
                pb_.add(left_foot_contact_fn == 0.,
                        tvm::task_dynamics::P(1.),
                        {tvm::requirements::PriorityLevel(1)}));

  auto right_foot_contact_fn = std::make_shared<tvm::robot::internal::GeometricContactFunction>(
      contact_map_["RightFootGround"], Eigen::Matrix6d::Identity());
  pushToTaskMap("RightFootContactTask",
                pb_.add(right_foot_contact_fn == 0.,
                        tvm::task_dynamics::P(1.),
                        {tvm::requirements::PriorityLevel(1)}));

  // add posture task
  auto posture_fn = std::make_shared<tvm::robot::PostureFunction>(robot_);
  pushToTaskMap("PostureTask",
                pb_.add(posture_fn == 0.,
                        tvm::task_dynamics::P(1.),
                        {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1e-4)}));

  // add com task
  auto com_in_fn = std::make_shared<tvm::robot::CoMInConvexFunction>(robot_);
  auto cube = makeCube(robot_->com(), 0.05, 0.05, 1.0);
  for (const auto& p : cube) {
    com_in_fn->addPlane(p);
  }
  pushToTaskMap("ComInTask",
                pb_.add(com_in_fn >= 0.,
                        tvm::task_dynamics::VelocityDamper({1, 0, 1}, tvm::constant::big_number),
                        {tvm::requirements::PriorityLevel(0)}));

  // add eef task
  for (const auto& eef_func : eef_func_map_) {
    pushToTaskMap(eef_func.first+"OriTask",
                  pb_.add(std::get<0>(eef_func.second) == 0.,
                          tvm::task_dynamics::P(1.),
                          {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1.)}));
    pushToTaskMap(eef_func.first+"PosTask",
                  pb_.add(std::get<1>(eef_func.second) == 0.,
                          tvm::task_dynamics::P(1.),
                          {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1.)}));
  }

  // add chest orientation task if chest frame exists
  if (frame_map_.count("ChestFrame")) {
    auto chest_ori_fn = std::make_shared<tvm::robot::OrientationFunction>(frame_map_.at("ChestFrame"));
    pushToTaskMap("ChestOriTask",
                  pb_.add(chest_ori_fn == 0.,
                          tvm::task_dynamics::P(1.),
                          {tvm::requirements::PriorityLevel(1),
                                tvm::requirements::AnisotropicWeight(Eigen::Vector3d(1e-2,1e-2,1e-6))}));
  }

  // set bounds
  // the bound task for qFreeFlyer cause the following exception:
  // terminate called after throwing an instance of 'std::runtime_error'
  //   what():  We allow linear function only on Euclidean variables.
  // pushToTaskMap("FreeFlyerBoundTask",
  //               pb_.add(-1. <= robot_->qFreeFlyer() <= 1.,
  //                       tvm::task_dynamics::VelocityDamper({1, 0, 1}, tvm::constant::big_number),
  //                       {tvm::requirements::PriorityLevel(0)}));
  pushToTaskMap("JointsBoundTask",
                pb_.add(robot_->lQBound() <= robot_->qJoints() <= robot_->uQBound(),
                        tvm::task_dynamics::VelocityDamper({1, 0, 1}, tvm::constant::big_number),
                        {tvm::requirements::PriorityLevel(0)}));

  // regularization
  pushToTaskMap("FreeFlyerRegularizationTask",
                pb_.add(dot(robot_->qFreeFlyer()) == 0.,
                        {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1e-6)}));
  pushToTaskMap("JointsRegularizationTask",
                pb_.add(dot(robot_->qJoints()) == 0.,
                        {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1e-6)}));

  printTasksInfo();
}

void PostureGenerator::resetRobot()
{
  // based on https://github.com/jrl-umi3218/tvm/blob/8eeaabd46ab7e5f34b78d3421d94d6d44a366a6f/src/robot/utils.cpp#L58-L74
  auto& mbc_q = robot_->mbc().q; // reference(&) is necessary to overwrite joint position
  const auto& jIndexByName = robot_->mb().jointIndexByName();
  for (const auto & q : initial_q_) {
    if (!jIndexByName.count(q.first)) {
      // ROS_DEBUG("joint %s: not found", q.first.c_str());
      continue;
    }
    auto jIndex = jIndexByName.at(q.first);
    if (mbc_q[jIndex].size() != q.second.size()) {
      ROS_ERROR_STREAM("Joint " << q.first << ": provided configuration has " << q.second.size() <<
                       " params but loaded robot has " << mbc_q[jIndex].size() << " params.");
      continue;
    }
    mbc_q[jIndex] = q.second;
  }

  auto robot_q = robot_->q().value();
  rbd::paramToVector(mbc_q, robot_q);
}

bool PostureGenerator::run()
{
  tvm::LinearizedControlProblem lpb(pb_);

  tvm::scheme::WeightedLeastSquares solver(tvm::solver::DefaultLSSolverOptions{});

  resetRobot();
  solve_result_ = false;

  // loop
  ros::Rate rate(static_cast<int>(1.0 / sleep_duration_in_loop_));
  while (ros::ok()) {
    // reset
    if (reset_) {
      resetRobot();
      reset_ = false;
    }

    // solve
    bool res = solver.solve(lpb);
    if (!res) {
      ROS_ERROR("Solver failed");
      break;
    }

    // integrate
    {
      // Clock::advance is available only for the second order case (i.e. ddot(q) is variable).
      // clock_->advance();
      auto robot_dq = dot(robot_->q()).value();
      rbd::vectorToParam(robot_dq, robot_->mbc().alpha);
      rbd::eulerIntegration(robot_->mb(), robot_->mbc(), clock_->dt());
      auto robot_q = robot_->q().value();
      rbd::paramToVector(robot_->mbc().q, robot_q);
    }

    // check convergence
    // todo: treat left and right equally
    double ori_err = std::get<0>(eef_func_map_.at("LeftHandFrame"))->value().norm();
    double pos_err = std::get<1>(eef_func_map_.at("LeftHandFrame"))->value().norm();
    if ((ori_err < ori_thre_) && (pos_err < pos_thre_)) {
      solve_result_ = true;
    } else {
      solve_result_ = false;
    }

    // publish
    if (publish_in_loop_) {
      publishRobotState();
      publishPoseArray();
      publishTaskState();
    }
    ros::spinOnce();

    // sleep
    if (sleep_duration_in_loop_ > 0) {
      rate.sleep();
    }
  }

  return true;
}

void PostureGenerator::publishRobotState() const
{
  moveit_msgs::DisplayRobotState robot_state_msg;

  // set joint position
  for (const auto& joint : robot_->mb().joints()) {
    if (joint.dof() == 1) {
      int joint_idx = robot_->mb().jointIndexByName(joint.name());
      double joint_pos = robot_->mbc().q[joint_idx][0];

      // printf("%s: %lf\n", joint.name().c_str(), joint_pos);
      robot_state_msg.state.joint_state.name.push_back(joint.name());
      robot_state_msg.state.joint_state.position.push_back(joint_pos);
    }
  }

  // set root position
  const std::vector<double>& root_q = robot_->mbc().q[0];
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

void PostureGenerator::publishPoseArray() const
{
  geometry_msgs::PoseArray pose_arr_msg;
  pose_arr_msg.header.frame_id = "world";

  // set eef
  for (const auto& eef_func : eef_func_map_) {
    const std::string& eef_name = eef_func.first;
    const geometry_msgs::Pose& eef_pose_msg = toRosMsg(frame_map_.at(eef_name)->position());
    pose_arr_msg.poses.push_back(eef_pose_msg);
  }

  // set com
  geometry_msgs::Pose com_pose_msg;
  com_pose_msg.position.x = robot_->com().x();
  com_pose_msg.position.y = robot_->com().y();
  com_pose_msg.position.z = robot_->com().z();
  com_pose_msg.orientation.w = 1;
  com_pose_msg.orientation.x = 0;
  com_pose_msg.orientation.y = 0;
  com_pose_msg.orientation.z = 0;
  pose_arr_msg.poses.push_back(com_pose_msg);

  // publish
  pose_arr_pub_.publish(pose_arr_msg);
}

void PostureGenerator::publishTaskState() const
{
  // todo: treat left and right equally
  double ori_err = std::get<0>(eef_func_map_.at("LeftHandFrame"))->value().norm();
  double pos_err = std::get<1>(eef_func_map_.at("LeftHandFrame"))->value().norm();

  jsk_rviz_plugins::OverlayText text_msg;
  std_msgs::ColorRGBA color;
  if (solve_result_) {
    color.r = 0.3568627450980392;
    color.g = 0.7529411764705882;
    color.b = 0.8705882352941177;
    color.a = 1.0;
  } else {
    color.r = 0.8509803921568627;
    color.g = 0.3254901960784314;
    color.b = 0.30980392156862746;
    color.a = 1.0;
  }
  text_msg.text =
      "pos err: " + std::to_string(pos_err) + "\nori error: " + std::to_string(ori_err);
  text_msg.width = 1000;
  text_msg.height = 1000;
  text_msg.top = 10;
  text_msg.left = 10;
  text_msg.bg_color.a = 0.0;
  text_msg.fg_color = color;
  text_msg.text_size = 24;

  // publish
  text_pub_.publish(text_msg);
}

void PostureGenerator::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_st_msg)
{
  // update target
  std::get<0>(eef_func_map_.at("LeftHandFrame"))->orientation(
      Eigen::Quaterniond{
        pose_st_msg->pose.orientation.w,
            pose_st_msg->pose.orientation.x,
            pose_st_msg->pose.orientation.y,
            pose_st_msg->pose.orientation.z
            }.inverse().toRotationMatrix());
  std::get<1>(eef_func_map_.at("LeftHandFrame"))->position(
      Eigen::Vector3d{
        pose_st_msg->pose.position.x,
            pose_st_msg->pose.position.y,
            pose_st_msg->pose.position.z});
}

bool PostureGenerator::resetCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  reset_ = true;
  return true;
}

void PostureGenerator::printFramesInfo() const
{
  ROS_INFO("PostureGenerator frames:");
  for (const auto& frame_map_i : frame_map_) {
    if (frame_map_i.first != frame_map_i.second->name()) {
      throw std::runtime_error("frame name in frame_map_ is not consistent");
    }

    const auto& frame = frame_map_i.second;
    ROS_INFO_STREAM("  name: " << frame->name()
                    << " robot: " << frame->robot().name()
                    << "  body: " << frame->body()
                    << "  pos: " << frame->position().translation().transpose()
                    << "  rot: " << Eigen::Quaterniond(frame->position().rotation()).coeffs().transpose());
  }
}

void PostureGenerator::printTasksInfo() const
{
  ROS_INFO("PostureGenerator tasks:");
  for (const auto& task_map_i : task_map_) {
    const auto& frame = task_map_i.second;
    ROS_INFO_STREAM("  name: " << task_map_i.first);
  }
}
