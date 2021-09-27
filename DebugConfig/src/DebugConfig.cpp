#include "DebugConfig.h"

DebugConfig::DebugConfig(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  const mc_rtc::Configuration defaultConfig = config("sample")("default");
  // Same result for const reference.
  // const mc_rtc::Configuration & defaultConfig = config("sample")("default");
  mc_rtc::log::info("defaultConfig: {}", defaultConfig.dump(true));

  mc_rtc::log::info("## Setup customConfig1 ##");
  mc_rtc::Configuration customConfig1 = defaultConfig;
  customConfig1.load(config("sample")("custom1"));
  mc_rtc::log::info("defaultConfig: {}", defaultConfig.dump(true));
  mc_rtc::log::info("customConfig1: {}", customConfig1.dump(true));

  mc_rtc::log::info("## Setup customConfig2 ##");
  mc_rtc::Configuration customConfig2 = defaultConfig;
  customConfig2.load(config("sample")("custom2"));
  mc_rtc::log::info("defaultConfig: {}", defaultConfig.dump(true));
  mc_rtc::log::info("customConfig2: {}", customConfig2.dump(true));

  mc_rtc::log::success("DebugConfig init done ");
}

bool DebugConfig::run()
{
  return mc_control::fsm::Controller::run();
}

void DebugConfig::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


