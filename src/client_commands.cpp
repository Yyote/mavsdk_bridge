#include <iostream>
#include <chrono>
#include <thread>

#include <mavsdk/plugins/action/action.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "privyaznik_msgs/msg/command.hpp"
#include "privyaznik_msgs/srv/utm_to_wgs.hpp"
#include "privyaznik_msgs/srv/wgs_to_utm.hpp"
#include "privyaznik_msgs/srv/command.hpp"

#include <mavsdk/plugins/mission_raw/mission_raw.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/mavlink/common/mavlink.h>
using mavsdk::MavlinkPassthrough, mavsdk::Mavsdk, mavsdk::ConnectionResult, mavsdk::Param, mavsdk::Telemetry;
using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

using std::chrono::seconds;
using std::this_thread::sleep_for;


using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

//   if (argc != 3) {
//       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
//       return 1;
//   }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("commands_client");
  rclcpp::Client<privyaznik_msgs::srv::Command>::SharedPtr client = node->create_client<privyaznik_msgs::srv::Command>("commands");


  auto request = std::make_shared<privyaznik_msgs::srv::Command::Request>();
  request->data = {10.5};
  request->cmd = 1;


  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %ld", result.get()->result);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}


