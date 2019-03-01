#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

int main(int _argc, char **_argv) {
  gazebo::client::setup(_argc, _argv);
  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  // Publish to the cart topic
  std::map<std::string, gazebo::transport::PublisherPtr> pm;
  std::vector<std::string> topics = {
      "~/cart/pendulum/pid",
      "~/cart/wheel/pid",
      "~/cart/wheel/tgt"
  };
  for (const auto& t: topics) {
    pm[t] = node->Advertise<gazebo::msgs::Vector3d>(t);
    // Wait for a subscriber to connect to this publisher
    pm[t]->WaitForConnection();
  }

  gazebo::msgs::Vector3d msg;
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(
              std::atof(_argv[2]),
              std::atof(_argv[3]),
              std::atof(_argv[4])));

  // Send the message
  auto res = pm.find(_argv[1]);
  if (res != pm.end()) {
    res->second->Publish(msg);
  } else {
    std::cerr << "Could not find publisher named '" << _argv[1] << "'\n";
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}

