#include "DebugConfig_Initial.h"

#include "../DebugConfig.h"

void DebugConfig_Initial::configure(const mc_rtc::Configuration & config)
{
}

void DebugConfig_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<DebugConfig &>(ctl_);
}

bool DebugConfig_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<DebugConfig &>(ctl_);
  output("OK");
  return true;
}

void DebugConfig_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<DebugConfig &>(ctl_);
}

EXPORT_SINGLE_STATE("DebugConfig_Initial", DebugConfig_Initial)
