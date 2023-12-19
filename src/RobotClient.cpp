#include "RobotClient.h"

namespace mc_kuka_fri
{

RobotClient::RobotClient(AppState & state, const std::string & name)
: state_(state), app_(connection_, *this), name_(name), torques_measured_(7, 0.0), joints_measured_(7, 0.0),
  torques_command_(7, 0.0), joints_command_(7, 0.0)
{
  auto config = state.gc.configuration().config("KukaFRI")(name);
  std::string host = config("host");
  int port = config("port", 30200);
  if(!app_.connect(port, host.c_str()))
  {
    mc_rtc::log::error_and_throw("[MCKukaFRI] Connection to {}:{} failed", host, port);
  } else {
    mc_rtc::log::success("[MCKukaFRI] Connection to {}:{} OK!", host, port);
  }
  
  updateMcRtcInputs();
}

void RobotClient::waitForCommand()
{
  kuka::fri::LBRClient::waitForCommand();

  std::lock_guard<std::mutex> lck(state_.gc_mutex);
  mc_rtc::log::info("[RobotClient] waitForCommand");
  updateMcRtcInputs();
  if(robotState().getClientCommandMode() == kuka::fri::TORQUE) { robotCommand().setTorque(torques_command_.data()); }

}

void RobotClient::updateMcRtcInputs()
{
  mc_rtc::log::info("[RobotClient] Command mode {}", robotState().getClientCommandMode());
  const auto & state = robotState();
  std::memcpy(joints_measured_.data(), state.getMeasuredJointPosition(), 7 * sizeof(double));
  std::memcpy(torques_measured_.data(), state.getMeasuredTorque(), 7 * sizeof(double));
  state_.gc.setEncoderValues(name_, joints_measured_);
  state_.gc.setJointTorques(name_, torques_measured_);
  mc_rtc::log::info("[RobotClient] joints_measured_ {} - {}", joints_measured_[0], joints_measured_[2]);
  // joints_measured_ OK

}

void RobotClient::command()
{
  kuka::fri::LBRClient::command();

  std::lock_guard<std::mutex> lck(state_.gc_mutex);
  mc_rtc::log::info("[RobotClient] command");
  updateKukaCommand();
}

void RobotClient::updateKukaCommand()
{
  mc_rtc::log::info("[RobotClient] updateKukaCommand");
  const auto & robot = state_.gc.robot(name_);
  for(size_t i = 0; i < 7; ++i)
  {
    auto mbcIdx = robot.jointIndexInMBC(i);
    torques_command_[i] = robot.jointTorque()[mbcIdx][0];
    joints_command_[i] = robot.q()[mbcIdx][0];
  }
  mc_rtc::log::info("[RobotClient] joints_command_ {} - {}", joints_command_[0], joints_command_[2]);
  mc_rtc::log::info("[RobotClient] torques_command_ {} - {}", torques_command_[0], torques_command_[2]);
  mc_rtc::log::info("[RobotClient] encoder values {}", robot.encoderValues()[robot.jointIndexInMBC(0)]);
  mc_rtc::log::info("[RobotClient] Command mode {}", robotState().getClientCommandMode());
  robotCommand().setJointPosition(joints_command_.data());
  if(robotState().getClientCommandMode() == kuka::fri::TORQUE) { robotCommand().setTorque(torques_command_.data()); }
}

void RobotClient::startControlThread()
{
  control_thread_ = std::thread(
      [this]()
      {
        bool ok = true;
        while(ok) { ok = app_.step(); }
      });
  mc_rtc::log::info("[RobotClient] startControlThread");
}

void RobotClient::joinControlThread()
{
  mc_rtc::log::info("[RobotClient] joinControlThread");
  if(!control_thread_.joinable()) { return; }
  control_thread_.join();
  app_.disconnect();
}

void MainRobotClient::command()
{
  mc_rtc::log::info("[RobotClient] MainRobotClient command");
  kuka::fri::LBRClient::command();

  std::lock_guard<std::mutex> lck(state_.gc_mutex);
  updateMcRtcInputs(); // update measured encoder positions and joint torques
  state_.gc.run();
  updateKukaCommand();
}

} // namespace mc_kuka_fri
