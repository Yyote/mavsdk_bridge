#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/mavlink/common/mavlink.h>
using mavsdk::MavlinkPassthrough, mavsdk::Mavsdk, mavsdk::ConnectionResult, mavsdk::Param, mavsdk::Telemetry;
using std::placeholders::_1;



class MavsdkBridgeNode : public rclcpp::Node
{
    public:
        MavsdkBridgeNode()
        : Node("minimal_subscriber"), mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}}
        {
            ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14550");

            while (connection_result != ConnectionResult::Success) {
                std::cerr << "Connection failed: " << connection_result << '\n';
            }

            system = mavsdk.first_autopilot(3.0);
            while (!system) {
                std::cerr << "Timed out waiting for system\n";
            }

            auto param_handle = Param{*system};
            param_handle.set_param_float("PLND_ENABLED", 1);
            param_handle.set_param_int("PLND_TYPE", 1);
            param_handle.set_param_int("PLND_EST_TYPE", 0);
            param_handle.set_param_int("LAND_SPEED", 20);
            // param_handle.set_param_int("LOG_DISARMED", 1);


            subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("/camera/landing_position", 10, std::bind(&MavsdkBridgeNode::topic_callback, this, _1));
        }

    private:
        Mavsdk mavsdk;
        std::optional<std::shared_ptr<mavsdk::System>> system;

        void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr coords) const // Проблема такого колбэка в том, что const не позволяет модифицировать переменные за пределами функции
        {
            // mavsdk::Telemetry::Position curr_pose;
            // Telemetry telem(*system);
            // mavsdk::Telemetry::PositionHandle pose_handle = telem.subscribe_position(
            // [&curr_pose](Telemetry::Position pose)
            // {
            //     curr_pose = pose;
            // });

            float cx, cy;

            cx = coords->x;
            cy = coords->y;

            // Define the LANDING_TARGET message content
            mavlink_landing_target_t landing_target_msg{};
            landing_target_msg.time_usec = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1); // Replace with actual timestamp
            landing_target_msg.target_num = 0; // Assuming single target
            landing_target_msg.frame = MAV_FRAME_BODY_FRD; // Example frame
            landing_target_msg.angle_x = cx;
            landing_target_msg.angle_y = cy;
            // landing_target_msg.distance = curr_pose.relative_altitude_m;
            // ... (Fill other fields as needed: angle_x, angle_y, distance, etc.) ...

            auto mavlink_passthrough = MavlinkPassthrough{*system};

            // Function to pack and send the message
            auto send_landing_target_message = [this, &mavlink_passthrough, &landing_target_msg](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_message_t message;
                mavlink_msg_landing_target_encode(mavlink_passthrough.get_our_sysid(), channel, &message, &landing_target_msg);
                return message;
            };

            // Send the message using queue_message
            mavsdk::MavlinkPassthrough::Result res = mavlink_passthrough.queue_message(send_landing_target_message);
            switch (res) 
            {
                case mavsdk::MavlinkPassthrough::Result::Success:
                    std::cout << "Landing target message queued successfully." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::Unknown:
                    std::cerr << "An unknown error occurred while sending the message." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::ConnectionError:
                    std::cerr << "Error connecting to the autopilot. Check the connection and try again." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandNoSystem:
                    std::cerr << "Mavlink system not available." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandBusy:
                    std::cout << "System is busy. Please try again later." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandDenied:
                    std::cerr << "Command to send landing target message has been denied." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandUnsupported:
                    std::cerr << "Sending landing target message is not supported by this autopilot." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandTimeout:
                    std::cerr << "Timeout while sending the landing target message. Check connection and retry." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandTemporarilyRejected:
                    std::cout << "Sending landing target message temporarily rejected. Try again later." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandFailed:
                    std::cerr << "Failed to send the landing target message." << std::endl;
                    break;
                // Add cases for other specific error handling if needed...
                default:
                    std::cerr << "Unhandled Mavlink passthrough result: " << static_cast<int>(res) << std::endl;
            }
        }
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavsdkBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
