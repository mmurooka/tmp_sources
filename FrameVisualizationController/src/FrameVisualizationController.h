#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"

struct FrameVisualizationController_DLLAPI FrameVisualizationController : public mc_control::MCController
{
  FrameVisualizationController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

protected:
  std::string visualizeFrameName_ = "";
  std::vector<std::string> frameNameList_;
};
