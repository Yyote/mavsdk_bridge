#include <iostream>
#include <chrono>

#include <mavsdk/plugins/action/action.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "privyaznik_msgs/msg/command.hpp"

#include <mavsdk/plugins/mission_raw/mission_raw.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/mavlink/common/mavlink.h>
using mavsdk::MavlinkPassthrough, mavsdk::Mavsdk, mavsdk::ConnectionResult, mavsdk::Param, mavsdk::Telemetry;
using std::placeholders::_1;

using std::chrono::seconds;
using std::this_thread::sleep_for;

mavsdk::MissionRaw::MissionItem make_mission_item_wp(
    double latitude_deg, double longitude_deg, float relative_altitude_m,
    float speed_m_s, bool is_fly_through, uint16_t &seq, bool is_current)
{
    mavsdk::MissionRaw::MissionItem new_item{};
    new_item.mission_type = MAV_MISSION_TYPE_MISSION;
    new_item.command = MAV_CMD_NAV_WAYPOINT;
    new_item.param1 = 0; 
    new_item.param2 = 1;
    new_item.param3 = 0;
    new_item.param4 = 0;
    new_item.x = int32_t(std::round(latitude_deg * 1e7));
    new_item.y = int32_t(std::round(longitude_deg * 1e7));
    new_item.z = relative_altitude_m;
    new_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    new_item.autocontinue = 1;
    new_item.seq = seq;
    seq++;
    new_item.current = static_cast<uint32_t>(is_current);
    return new_item;
}

/**
 * @param _seq
 * @param _frame
 * @param _command
 * @param _current
 * @param _autocontinue
 * @param _param1
 * @param _param2
 * @param _param3
 * @param _param4
 * @param _x
 * @param _y
 * @param _z
 * @param _mission_type
 * 
*/
mavsdk::MissionRaw::MissionItem create_mission_item(uint32_t _seq, uint32_t _frame, uint32_t _command, uint32_t _current, uint32_t _autocontinue,
                                            float _param1, float _param2, float _param3, float _param4, 
                                            float _x, float _y, float _z, 
                                            uint32_t _mission_type)

{
    mavsdk::MissionRaw::MissionItem new_raw_item_nav{};
    new_raw_item_nav.seq = _seq;
    new_raw_item_nav.frame = _frame; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT 
    new_raw_item_nav.command = _command; // MAV_CMD_NAV_WAYPOINT
    new_raw_item_nav.current = _current;
    new_raw_item_nav.autocontinue = _autocontinue;
    new_raw_item_nav.param1 = _param1; // Hold
    new_raw_item_nav.param2 = _param2; // Accept Radius
    new_raw_item_nav.param3 = _param3; // Pass Radius
    new_raw_item_nav.param4 = _param4; // Yaw
    new_raw_item_nav.x = int32_t(std::round(_x * 1e7));
    new_raw_item_nav.y = int32_t(std::round(_y * 1e7));
    new_raw_item_nav.z = _z;
    new_raw_item_nav.mission_type = 0;
    return new_raw_item_nav;
}

std::vector<mavsdk::MissionRaw::MissionItem> create_mission_raw(float home_alt)
{
    std::vector<mavsdk::MissionRaw::MissionItem> mission_raw_items;

    std::cout << "home_position.absolute_altitude_m: " << home_alt << std::endl;

   
    mission_raw_items.push_back(create_mission_item(0, 0, 16, 1, 1, 0, 0, 0, 0, 47.397742, 8.545594, home_alt, 0));

    // Add Takeoff
    mission_raw_items.push_back(create_mission_item(1, 3, 22, 0, 1, 0, 0, 0, 0, 0, 0, 30, 0));

    // Add Mission Item 2-3
    mission_raw_items.push_back(create_mission_item(2, 3, 16, 0, 1, 0, 0, 0, 0, -36.3634, 149.164, 30, 0));
    //mission_raw_items.push_back(create_mission_item(3, 3, 16, 0, 1, 0, 0, 0, 0, -35.3635, 149.165, 30, 0));

    // Return to Launch
    //mission_raw_items.push_back(create_mission_item(4, 3, 20, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0));
    return mission_raw_items;
}


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

            mission = std::make_unique<mavsdk::MissionRaw>(system.value());
            action = std::make_unique<mavsdk::Action>(system.value());
            telemetry = std::make_unique<mavsdk::Telemetry>(system.value());

            telemetry->subscribe_home([this](Telemetry::Position home_in){home_position = home_in;});
            telemetry->subscribe_position([this](Telemetry::Position curr_pose){current_position = curr_pose;});

            auto param_handle = Param{*system};
            param_handle.set_param_float("PLND_ENABLED", 1);
            param_handle.set_param_int("PLND_TYPE", 1);
            param_handle.set_param_int("PLND_EST_TYPE", 0);
            param_handle.set_param_int("LAND_SPEED", 20);
            // param_handle.set_param_int("LOG_DISARMED", 1);

            sub_prec_landing = this->create_subscription<geometry_msgs::msg::Vector3>("/camera/landing_position", 10, std::bind(&MavsdkBridgeNode::prec_land_callback, this, _1));
            sub_commands = this->create_subscription<privyaznik_msgs::msg::Command>("/commands", 10, std::bind(&MavsdkBridgeNode::commands_callback, this, _1));
        }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private:
        Mavsdk mavsdk;
        std::optional<std::shared_ptr<mavsdk::System>> system;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_prec_landing;
        rclcpp::Subscription<privyaznik_msgs::msg::Command>::SharedPtr sub_commands;

        std::unique_ptr<mavsdk::MissionRaw> mission;
        std::unique_ptr<mavsdk::Action> action;
        std::unique_ptr<mavsdk::Telemetry> telemetry;
        Telemetry::Position current_position;
        Telemetry::Position home_position;


        void commands_callback(const privyaznik_msgs::msg::Command::SharedPtr command) // Проблема такого колбэка в том, что const не позволяет модифицировать переменные за пределами функции
        {
            std::vector<float> data = command->data;
            switch (command->cmd)
            {
                case privyaznik_msgs::msg::Command::CMD_LAND:
                    try_land(data);
                    break;
                case privyaznik_msgs::msg::Command::CMD_TAKEOFF:
                    try_takeoff(data);
                    break;
                case privyaznik_msgs::msg::Command::CMD_MOVE:
                    try_move(data);
                    break;
                default:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "mavsdk_bridge: unknown command.");
                    throw std::exception();
            }
        }


        void prec_land_callback(const geometry_msgs::msg::Vector3::SharedPtr coords) // Проблема такого колбэка в том, что const не позволяет модифицировать переменные за пределами функции
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

        /**
         * @brief Выполняет попытку сесть, как только получена команда. Если не получилось, пытается еще.
         * @param data std::vector<float> - не несет полезных значений для этой команды
        */
        void try_land(std::vector<float> data)
        {    
            std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n";
            mavsdk::Action::Result result = action->return_to_launch();
            if (result != mavsdk::Action::Result::Success) RCLCPP_ERROR_STREAM(this->get_logger(), "Land failed");
            return;

            //  std::vector<mavsdk::MissionRaw::MissionItem> land_items_plan;

            // land_items_plan.push_back(create_mission_item(0, 0, 16, 1, 1, 0, 0, 0, 0, home_position.latitude_deg, home_position.longitude_deg, home_position.absolute_altitude_m, 0));

            // // Add Mission Item 2-3
            // // land_items_plan.push_back(create_mission_item(2, 3, 16, 0, 1, 0, 0, 0, 0, -35.3634, 149.164, 30, 0));
            // land_items_plan.push_back(create_mission_item(1, 3, 16, 0, 1, 0, 1, 0, 0, current_position.latitude_deg, current_position.longitude_deg, current_position.absolute_altitude_m, 0));

            // // Return to Launch
            // land_items_plan.push_back(create_mission_item(2, 3, 20, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0));



            //  const mavsdk::MissionRaw::Result upload_result = mission->upload_mission(land_items_plan);
             
            //  if (upload_result != mavsdk::MissionRaw::Result::Success) 
            //  {
            //  std::cerr << "Mission upload failed: " << upload_result << ", exiting.\n";
        
            //  }


            // std::atomic<bool> want_to_pause{false};
            // // Before starting the mission, we want to be sure to subscribe to the mission progress.
            // mission->subscribe_mission_progress([&want_to_pause](mavsdk::MissionRaw::MissionProgress mission_progress) 
            // {
            //     std::cout << "Mission status update: " << mission_progress.current << " / "
            //             << mission_progress.total << '\n';
            //     if (mission_progress.current >= 2)
            //     {
            //         // We can only set a flag here. If we do more request inside the callback,
            //         // we risk blocking the system.
            //         want_to_pause = true;
            //     }
            // });

            // mavsdk::MissionRaw::Result start_mission_result = mission->start_mission();
            // if (start_mission_result != mavsdk::MissionRaw::Result::Success) 
            // {
            //     std::cerr << "Starting mission failed: " << start_mission_result << '\n';
            //     std::cout << "Commanding RTL...\n";
            //     const mavsdk::Action::Result rtl_result = action->return_to_launch();
            //     if (rtl_result != mavsdk::Action::Result::Success) {
            //         std::cout << "Failed to command RTL: " << rtl_result << '\n';
                    
            //     }
            // }  
        }



        /**
         * @brief Выполняет попытку подвинуться на заданные смещение и вращение (по рысканию) относительно позиции, в которой получил команду
         * @param data std::vector<float> - {x, y, z, yaw}
        */
        void try_move(std::vector<float> data)
        {
            float z_coor, yaw, start_yaw, x, res;
            double x_coor, y_coor;
            // x_coor = data[0]; 
            // y_coor = data[1]; 
            // z_coor = data[2]; 
            x_coor = data[0]; 
            y_coor = data[1]; 
            z_coor = data[2] + current_position.absolute_altitude_m; 
            yaw = data[4];

            if (yaw>360)
            {
                x = yaw - ( (int(yaw))/360 ) *360;
            }

            else
            {
                if (yaw<-360)
                {
                    x = yaw - ( (int(yaw))/360 ) *360;
                }

                else {x = yaw;}
            }


            if (abs(x + start_yaw) <=180)
            {
                res = x;
            }

            else
            {
                if (x>=0)
                {
                    res = x - 360;
                }
                else
                {
                    res = x + 360;
                }
            }



            std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n";

            mavsdk::Action::Result result = action->goto_location(x_coor, y_coor, z_coor, res);
            if (result != mavsdk::Action::Result::Success) RCLCPP_ERROR_STREAM(this->get_logger(), "Move failed");
            return;
        }


        /**
         * @brief Выполняет попытку взлета на заданную высоту.
         * @param data std::vector<float> - {height}
        */
        void try_takeoff(std::vector<float> data)
        {
            float z_coor;
            z_coor = data[0]; 
            
             while (!telemetry->health_all_ok())
            {
                std::cout << "Waiting for system to be ready\n";
                sleep_for(seconds(1));
            }
            
            std::cout << "System ready\n";
            std::cout << "Creating and uploading mission\n";

        
            Telemetry::Position home_position;
            while (isnan(home_position.absolute_altitude_m) == true) 
            {   
                telemetry->subscribe_position([&home_position](Telemetry::Position position) 
                {   

                    std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
                    std::cout<< "x coordinate "<<position.latitude_deg<<std::endl;
                    std::cout<< "y coordinate "<<position.longitude_deg<<std::endl;
                    home_position = position;
                });

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            uint16_t seq = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           
            std::vector<mavsdk::MissionRaw::MissionItem> land_raw_items;

            land_raw_items.push_back(create_mission_item(0, 0, 16, 1, 1, 0, 0, 0, 0, home_position.latitude_deg, home_position.longitude_deg, home_position.absolute_altitude_m, 0));
            land_raw_items.push_back(create_mission_item(1, 3, 22, 0, 1, 0, 0, 0, 0, 0, 0, z_coor, 0));  
            
            std::vector<mavsdk::MissionRaw::MissionItem> mission_items_plan = land_raw_items;

            
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            std::cout << "Uploading mission...\n";
            
            const mavsdk::MissionRaw::Result upload_result = mission->upload_mission(mission_items_plan);

            if (upload_result != mavsdk::MissionRaw::Result::Success) 
            {
                std::cerr << "Mission upload failed: " << upload_result << ", exiting.\n";
                return;
            }

            std::cout << "Arming...\n";
            const mavsdk::Action::Result arm_result = action->arm();
            if (arm_result != mavsdk::Action::Result::Success) 
            {
                std::cerr << "Arming failed: " << arm_result << '\n';
                return;
            }
            std::cout << "Armed.\n";

            const mavsdk::Action::Result takeoff_result = action->takeoff();
            if (takeoff_result != mavsdk::Action::Result::Success) 
            {
                std::cerr << "Takeoff failed: " << takeoff_result << '\n';
                return;
            }
            std::atomic<bool> want_to_pause{false};
            // Before starting the mission, we want to be sure to subscribe to the mission progress.
            mission->subscribe_mission_progress([&want_to_pause](mavsdk::MissionRaw::MissionProgress mission_progress) 
            {
                std::cout << "Mission status update: " << mission_progress.current << " / "
                        << mission_progress.total << '\n';
                if (mission_progress.current >= 2)
                {
                    // We can only set a flag here. If we do more request inside the callback,
                    // we risk blocking the system.
                    want_to_pause = true;
                }
            });

            mavsdk::MissionRaw::Result start_mission_result = mission->start_mission();
            if (start_mission_result != mavsdk::MissionRaw::Result::Success) 
            {
                std::cerr << "Starting mission failed: " << start_mission_result << '\n';
                std::cout << "Commanding RTL...\n";
                const mavsdk::Action::Result rtl_result = action->return_to_launch();
                if (rtl_result != mavsdk::Action::Result::Success) {
                    std::cout << "Failed to command RTL: " << rtl_result << '\n';
                    
                }
            }    
            
        }

        
};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavsdkBridgeNode>());
    rclcpp::shutdown();
    return 0;
}


// Угол < 360 (-180 -- 180)
// if ( (curr_yaw + add_yaw) > 180 || (curr_yaw + add_yaw) < 180 )
// {
    

// }

// Учесть длину провода
// max_len = (local_x)^2 + (local_y)^2 + (z_coor)^2
// z_coor = ( max_len - ((local_x)^2 + (local_y)^2) ) ^ 1/2

// Высота взлёта
// Возврат домой