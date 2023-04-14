#include <algorithm>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/gui/Transform.h>

#include "FrameVisualizationController.h"

FrameVisualizationController::FrameVisualizationController(mc_rbdyn::RobotModulePtr rm,
                                                           double dt,
                                                           const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  mc_rtc::log::success("[FrameVisualizationController] Constructed.");
}

bool FrameVisualizationController::run()
{
  return mc_control::MCController::run();
}

void FrameVisualizationController::reset(const mc_control::ControllerResetData & resetData)
{
  mc_control::MCController::reset(resetData);

  frameNameList_ = robot().frames();
  std::sort(frameNameList_.begin(), frameNameList_.end());
  frameNameList_.insert(frameNameList_.begin(), "");
  gui()->addElement(
      {"FrameVisualization"},
      mc_rtc::gui::ComboInput(
          "FrameName", frameNameList_, [this]() -> const std::string & { return visualizeFrameName_; },
          [this](const std::string & v) {
            visualizeFrameName_ = v;
            framePoseWithOffset_ = visualizeFrameName_.empty() ? sva::PTransformd::Identity()
                                                               : robot().frame(visualizeFrameName_).position();
          }),
      mc_rtc::gui::Transform("FramePose",
                             [this]() {
                               return visualizeFrameName_.empty() ? sva::PTransformd::Identity()
                                                                  : robot().frame(visualizeFrameName_).position();
                             }),
      mc_rtc::gui::Transform("FramePoseWithOffset", framePoseWithOffset_),
      mc_rtc::gui::Button("PrintFramePoseWithOffset", [this]() {
        if(visualizeFrameName_.empty())
        {
          mc_rtc::log::warning("[FrameVisualizationController] Specify FrameName.");
          return;
        }
        sva::PTransformd relPose = framePoseWithOffset_ * robot().frame(visualizeFrameName_).position().inv();
        Eigen::Vector3d pos = relPose.translation();
        Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(relPose.rotation());
        mc_rtc::log::info("[FrameVisualizationController] PrintFramePoseWithOffset.\n- name: {}\n  "
                          "parent: {}\n  X_p_f:\n    translation: [{}, {}, {}]\n    rotation: [{}, {}, {}]",
                          visualizeFrameName_ + "_WithOffset", visualizeFrameName_, pos.x(), pos.y(), pos.z(), rpy.x(),
                          rpy.y(), rpy.z());
      }));
}

CONTROLLER_CONSTRUCTOR("FrameVisualizationController", FrameVisualizationController)
