#include "RobotClient.h"

namespace mc_kuka_fri
{

RobotClient::RobotClient(AppState & state, const std::string & name)
: state_(state), app_(connection_, *this), name_(name), torques_measured_(7, 0.0), joints_measured_(7, 0.0),
  torques_command_(7, 0.0), joints_command_(7, 0.0), vel_estimated_(7, 0.0), joints_measured_prev_(7, 0.0)
{
  begin = std::chrono::steady_clock::now();
  auto config = state.gc.configuration().config("KukaFRI")(name);
  std::string host = config("host");
  int port = config("port", 30200);
  if(!app_.connect(port, host.c_str()))
  {
    mc_rtc::log::error_and_throw("[MCKukaFRI] Connection to {}:{} failed", host, port);
  } else {
    mc_rtc::log::success("[MCKukaFRI] Connection to {}:{} OK!", host, port);
  }
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
  joints_measured_prev_ = joints_measured_;
  // mc_rtc::log::info("[RobotClient] updateMcRtcInputs");
  // mc_rtc::log::info("[RobotClient] Command mode {}", robotState().getClientCommandMode());
  const auto & state = robotState();
  std::memcpy(joints_measured_.data(), state.getMeasuredJointPosition(), 7 * sizeof(double));
  std::memcpy(torques_measured_.data(), state.getMeasuredTorque(), 7 * sizeof(double));
  state_.gc.setEncoderValues(name_, joints_measured_);
  state_.gc.setJointTorques(name_, torques_measured_);

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/1000000.0;
  // mc_rtc::log::info("[RobotClient] loop duration {}", duration);
  for (size_t i = 0; i < 7; i++) {
    if(duration < 0.001 || duration > 0.9) { // avoid inf value at the start
      vel_estimated_[i] = 0;
    } else {
      vel_estimated_[i] = 0.2*(joints_measured_[i] - joints_measured_prev_[i])/(duration) + (1 - 0.2)*vel_estimated_[i];
    }
    begin = std::chrono::steady_clock::now();
  }
  state_.gc.setEncoderVelocities(name_, vel_estimated_);
  // mc_rtc::log::info("[RobotClient] joints_measured_ \n {}  \n {} \n {} \n {} \n {} \n {} \n {}", joints_measured_[0] \
  // , joints_measured_[1], joints_measured_[2], joints_measured_[3], joints_measured_[4], joints_measured_[5], joints_measured_[6]);
  // mc_rtc::log::info("[RobotClient] vel_estimated_ \n {}  \n {} \n {} \n {} \n {} \n {} \n {}", vel_estimated_[0] \
  // , vel_estimated_[1], vel_estimated_[2], vel_estimated_[3], vel_estimated_[4], vel_estimated_[5], vel_estimated_[6]);
  // mc_rtc::log::info("[RobotClient] torques_measured_ \n {}  \n {} \n {} \n {} \n {} \n {} \n {}", torques_measured_[0] \
  // , torques_measured_[1], torques_measured_[2], torques_measured_[3], torques_measured_[4], torques_measured_[5], torques_measured_[6]);


  // joints_measured_ OK

  firstSensorsReceived_ = true;
}

void RobotClient::command()
{
  kuka::fri::LBRClient::command();

  if(!state_.gc.running) { return; }

  std::lock_guard<std::mutex> lck(state_.gc_mutex);
  mc_rtc::log::info("[RobotClient] command");
  updateKukaCommand();
}

void RobotClient::updateKukaCommand()
{
  mc_rtc::log::info("[RobotClient] updateKukaCommand");
  const auto & robot = state_.gc.robot(name_);

  auto& torqueUpperLimits = robot.tu();
  auto& torqueLowerLimits = robot.tl();

  for(size_t i = 0; i < 7; ++i)
  {
    auto mbcIdx = robot.jointIndexInMBC(i);
    accelerationQP(i,0) = robot.mbc().alphaD[mbcIdx][0];
    VelocityQP(i,0) = robot.mbc().alpha[mbcIdx][0];
  }

  fdPtr_ = std::make_shared<rbd::ForwardDynamics>(robot.mb());
  coriolisPtr_= std::make_shared<rbd::Coriolis>(robot.mb());
  fdPtr_->forwardDynamics(state_.gc.robot(name_).mb(), state_.gc.robot(name_).mbc());
  
  massMatrix = fdPtr_->H();  
  auto CoriolisMatrix = coriolisPtr_->coriolis(robot.mb(),robot.mbc());
  massTorque = massMatrix*accelerationQP + CoriolisMatrix*VelocityQP;

  for(size_t i = 0; i < 7; ++i)
  {
    auto mbcIdx = robot.jointIndexInMBC(i);
    torques_command_[i] = massTorque(i,0);
    //torques_command_[i] = robot.jointTorque()[mbcIdx][0];

    //clip Tau commands


    double torqueLowerLimit = torqueLowerLimits[mbcIdx][0];
    double torqueUpperLimit = torqueUpperLimits[mbcIdx][0];
    torques_command_[i] = std::clamp(torques_command_[i] , torqueLowerLimit, torqueUpperLimit);

    joints_command_[i] = state_.gc.realRobot(name_).q()[mbcIdx][0];
    // joints_command_[i] = state_.gc.robot(name_).q()[mbcIdx][0];
  }
  // mc_rtc::log::info("[RobotClient] joints_command_ \n {}  \n {} \n {} \n {} \n {} \n {} \n {}", joints_command_[0] \
  // , joints_command_[1], joints_command_[2], joints_command_[3], joints_command_[4], joints_command_[5], joints_command_[6]);
  // mc_rtc::log::info("[RobotClient] torques_command_ \n {}  \n {} \n {} \n {} \n {} \n {} \n {}", torques_command_[0] \
  // , torques_command_[1], torques_command_[2], torques_command_[3], torques_command_[4], torques_command_[5], torques_command_[6]);

  // mc_rtc::log::info("[RobotClient] encoder values {}", robot.encoderValues()[robot.jointIndexInMBC(0)]);
  // mc_rtc::log::info("[RobotClient] Command mode {}", robotState().getClientCommandMode());

  robotCommand().setJointPosition(joints_command_.data());
  if(robotState().getClientCommandMode() == kuka::fri::TORQUE) { 
    // mc_rtc::log::info(" TORQUE COMMANDS ");
    robotCommand().setTorque(torques_command_.data()); 
    }
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
  mc_rtc::log::info("[RobotClient] MainRobotClient command", state_.gc.running);
  kuka::fri::LBRClient::command();

  if(!state_.gc.running) { return; }

  std::lock_guard<std::mutex> lck(state_.gc_mutex);
  updateMcRtcInputs(); // update measured encoder positions and joint torques
  state_.gc.run();
  updateKukaCommand();
}

} // namespace mc_kuka_fri
