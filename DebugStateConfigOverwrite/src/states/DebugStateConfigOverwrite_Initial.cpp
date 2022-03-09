#include "DebugStateConfigOverwrite_Initial.h"

#include "../DebugStateConfigOverwrite.h"

void DebugStateConfigOverwrite_Initial::configure(const mc_rtc::Configuration & config)
{
  mc_rtc::log::success("======== [{}]: Configure ========", name());
  mc_rtc::log::info("{}", config.dump(true));
}

void DebugStateConfigOverwrite_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<DebugStateConfigOverwrite &>(ctl_);
}

bool DebugStateConfigOverwrite_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<DebugStateConfigOverwrite &>(ctl_);
  output("OK");
  return true;
}

void DebugStateConfigOverwrite_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<DebugStateConfigOverwrite &>(ctl_);
}

EXPORT_SINGLE_STATE("DebugStateConfigOverwrite_Initial", DebugStateConfigOverwrite_Initial)
