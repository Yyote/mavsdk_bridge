#include <iostream>
#include <chrono>
#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/mavlink/common/mavlink.h>

using mavsdk::MavlinkPassthrough, mavsdk::Mavsdk, mavsdk::ConnectionResult, mavsdk::Param, mavsdk::Telemetry, mavsdk::Action;
using std::placeholders::_1;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using json = nlohmann::json;

json telemetry_data;

// Initialize ZMQ
zmq::context_t ctx{1};
zmq::socket_t socket{ctx, zmq::socket_type::pub};
std::string connect = "tcp://192.168.128.174:8080"; // Set TCP address

void error_msg (mavsdk::MavlinkPassthrough::Result result) 
{
    switch (result) 
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
            std::cerr << "Unhandled Mavlink passthrough result: " << static_cast<int>(result) << std::endl;
    }
}

// Create JSON message
void create_json() 
{
    telemetry_data = 
    {
        {"Time", 0},
        {"Latitude", 0},
        {"Longtitude", 0},
        {"Altitude", 0},
        {"Roll", 0},
        {"Pitch", 0},
        {"Yaw", 0},
        {"Heading", 0}
    };
}

class MavsdkBridgeNode : public rclcpp::Node
{
    public:
        MavsdkBridgeNode()
        : Node("minimal_subscriber"), mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}}
        {
            ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14550");

            while (connection_result != ConnectionResult::Success) 
            {
                std::cerr << "Connection failed: " << connection_result << '\n';
                throw std::exception();
            }

            system = mavsdk.first_autopilot(3.0);
            while (!system) 
            {
                std::cerr << "Timed out waiting for system\n";
                throw std::exception();
            }

            auto param_handle = Param{*system};
            param_handle.set_param_float("PLND_ENABLED", 1);
            param_handle.set_param_int("PLND_TYPE", 1);
            param_handle.set_param_int("PLND_EST_TYPE", 0);
            param_handle.set_param_int("LAND_SPEED", 20);
            param_handle.set_param_int("PLND_STRICT", 0);
            // param_handle.set_param_int("LOG_DISARMED", 1);

            telemetry_p = std::make_unique<Telemetry>(system.value());
            
            // We want to listen to the altitude of the drone at 1 Hz.
            telemetry_p->set_rate_position_async(1.0, [](Telemetry::Result set_rate_result)
            {
                std::cerr << "Setting position rate info: " << set_rate_result << "\n\n";
            });

            static bool fix_chk = false;
            auto hand = telemetry_p->subscribe_gps_info([](Telemetry::GpsInfo gps) 
            {
                std::cout << "--- GPS info ---" << "\n";
                std::cout << "GPS fix info: " << gps.fix_type << "\n";
                std::cout << "GPS num of satellites: " << gps.num_satellites << "\n\n";
                if (gps.num_satellites > 0) fix_chk = 1;
            });

            while (fix_chk == 0) sleep_for(seconds(3));
            telemetry_p->unsubscribe_gps_info(hand);

            create_json();
            socket.bind(connect);

            telemetry_p->subscribe_position([](Telemetry::Position position) 
            {
                /*std::cout << "Altitude: " << position.relative_altitude_m << "\n\n";
                std::cout << "--- GPS ---" << "\n";
                std::cout << "Latitude: " << position.latitude_deg << " deg\n";
                std::cout << "Longtitude: " << position.longitude_deg << " deg\n\n";*/

                telemetry_data["Altitude"] = position.relative_altitude_m;
                telemetry_data["Latitude"] = position.latitude_deg;
                telemetry_data["Longtitude"] = position.longitude_deg;
            });

            telemetry_p->subscribe_attitude_euler([](Telemetry::EulerAngle orient)
            {
                /*std::cout << "--- Attitude ---" << "\n";
                std::cout << "Roll: " << orient.roll_deg << " deg\n";
                std::cout << "Pitch: " << orient.pitch_deg << " deg\n";
                std::cout << "Yaw: " << orient.yaw_deg << " deg\n\n";*/

                telemetry_data["Roll"] = orient.roll_deg;
                telemetry_data["Pitch"] = orient.pitch_deg;
                telemetry_data["Yaw"] = orient.yaw_deg;
            });

            telemetry_p->subscribe_heading([](Telemetry::Heading hdd)
            {
                //std::cout << "Heading: " << hdd.heading_deg << " deg\n\n";

                telemetry_data["Heading"] = hdd.heading_deg;
            });

            subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("/camera/landing_position", 10, std::bind(&MavsdkBridgeNode::topic_callback, this, _1));
        }

    private:
        Mavsdk mavsdk;
        std::optional<std::shared_ptr<mavsdk::System>> system;
        std::unique_ptr<Telemetry> telemetry_p;

        void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr coords) // Проблема такого колбэка в том, что const не позволяет модифицировать переменные за пределами функции
        {
            auto action = Action{system.value()}; 

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
            auto send_landing_target_message = [this, &mavlink_passthrough, &landing_target_msg](MavlinkAddress mavlink_address, uint8_t channel) 
            {
                mavlink_message_t message;
                mavlink_msg_landing_target_encode(mavlink_passthrough.get_our_sysid(), channel, &message, &landing_target_msg);
                return message;
            };

            std::string serial_data = telemetry_data.dump();
            socket.send(zmq::str_buffer("Telemetry_data_topic"), zmq::send_flags::sndmore);
            socket.send(zmq::buffer(serial_data));

            std::cout << serial_data << "\n";
            std::cout << "\n";

            // Send the message using queue_message
            mavsdk::MavlinkPassthrough::Result res = mavlink_passthrough.queue_message(send_landing_target_message);
            error_msg(res);

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