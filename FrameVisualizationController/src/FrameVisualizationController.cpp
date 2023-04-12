#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/gui/Transform.h>

#include "FrameVisualizationController.h"

FrameVisualizationController::FrameVisualizationController(mc_rbdyn::RobotModulePtr rm,
                                                           double dt,
                                                           const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
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
  frameNameList_.insert(frameNameList_.begin(), "");
  gui()->addElement({"FrameVisualization"},
                    mc_rtc::gui::ComboInput(
                        "FrameName", frameNameList_, [this]() -> const std::string & { return visualizeFrameName_; },
                        [this](const std::string & v) { visualizeFrameName_ = v; }),
                    mc_rtc::gui::Transform("FramePose", [this]() {
                      return visualizeFrameName_.empty() ? sva::PTransformd::Identity()
                                                         : robot().frame(visualizeFrameName_).position();
                    }));
}

CONTROLLER_CONSTRUCTOR("FrameVisualizationController", FrameVisualizationController)
