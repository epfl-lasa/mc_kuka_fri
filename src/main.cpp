#include "RobotClient.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <mc_rtc/io_utils.h>

int main(int argc, char * argv[])
{
  std::string conf_file = "";
  po::options_description desc("MCFrankaControl options");
  // clang-format off
  desc.add_options()
    ("help", "Display help message")
    ("conf,f", po::value<std::string>(&conf_file), "Configuration file");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    return 0;
  }

  mc_control::MCGlobalController::GlobalConfiguration gconfig(conf_file, nullptr);
  if(!gconfig.config.has("KukaFRI"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No KukaFRI section in the configuration, see README for an example");
  }
  mc_kuka_fri::AppState state(gconfig);

  auto kukaConfig = gconfig.config("KukaFRI");
  auto ignored = kukaConfig("ignored", std::vector<std::string>{});

  std::vector<mc_kuka_fri::RobotClientPtr> clients;
  for(const auto & r : state.gc.robots())
  {
    if(r.mb().nrDof() == 0 || std::find(ignored.begin(), ignored.end(), r.name()) != ignored.end()) { continue; }
    if(!kukaConfig.has(r.name()))
    {
      mc_rtc::log::error_and_throw("[MCKukaFRI] {} is an active robot but is not configured in KukaFRI, either add a "
                                   "configuration for this robot or add it to the ignored list",
                                   r.name());
    }
    if(clients.empty()) { clients.push_back(std::make_unique<mc_kuka_fri::MainRobotClient>(state, r.name())); }
    else { clients.push_back(std::make_unique<mc_kuka_fri::RobotClient>(state, r.name())); }
  }
  for(auto & client : clients) { client->startControlThread(); }

  // Wait until all clients have received initial sensor data before initializing the control loop
  bool init = false;
  while(!init)
  {
    init = std::all_of(clients.begin(), clients.end(), [](const auto & client) { return client->firstSensorsReceived(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  {
    std::lock_guard<std::mutex> lck(state.gc_mutex);
    std::map<std::string, std::vector<double>> initEncoders;
    for(const auto & client : clients)
    {
      const auto & robotName = client->robotName();
      initEncoders[robotName] = state.gc.controller().robot(robotName).encoderValues();
      mc_rtc::log::info("- Initializing encoders for robot {} to {}", robotName, mc_rtc::io::to_string(initEncoders[robotName]));
    }
    state.gc.init(initEncoders);
    // Robots have all been intialized from sensor values, start sending commands
    state.gc.running = true;
  }

  for(auto & client : clients) { client->joinControlThread(); }
  return 0;
}
