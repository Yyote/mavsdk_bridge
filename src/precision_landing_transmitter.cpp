#include <iostream>
#include <chrono>
#include <cinttypes>
#include <functional>
#include <thread>

#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <mavsdk/plugins/action/action.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mavros_msgs/msg/gpsraw.hpp"
#include "privyaznik_msgs/msg/command.hpp"
#include "privyaznik_msgs/srv/utm_to_wgs.hpp"
#include "privyaznik_msgs/srv/wgs_to_utm.hpp"
#include "privyaznik_msgs/srv/command.hpp"
#include "privyaznik_msgs/action/command.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/msg/string.hpp"

#include <mavsdk/plugins/mission_raw/mission_raw.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/mavlink/common/mavlink.h>

#include "rclcpp_action/rclcpp_action.hpp"

using GoalHandlePrivyaznik = rclcpp_action::ServerGoalHandle<privyaznik_msgs::action::Command>;
using mavsdk::MavlinkPassthrough, mavsdk::Mavsdk, mavsdk::ConnectionResult, mavsdk::Param, mavsdk::Telemetry;
using mavsdk::MavlinkPassthrough, mavsdk::Mavsdk, mavsdk::ConnectionResult, mavsdk::Param, mavsdk::Telemetry, mavsdk::Action;
using json = nlohmann::json;

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

using std::chrono::seconds;
using std::this_thread::sleep_for;

// const std::string zmq_connect_address = "tcp://127.0.0.1:8080"; // Set TCP address for ZMQ
std::string mavlink_addr = "udp://:14550"; // Set | type+address+baud | to connect mavsdk to mavlink
//const std::string zmq_connect_address = "tcp://192.168.128.174:8080"; // Set TCP address for ZMQ
// std::string mavlink_addr = "serial:///dev/ttyUSB0:115200"; // Set | type+address+baud | to connect mavsdk to mavlink
const std::string zmq_connect_address = "tcp://192.168.2.91:8080";

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


template <typename Request, typename Response, typename ClientBase>
bool Client(ClientBase &client, Request &req, Response &resp, int timeout_ms, int retry_num=0)
{
    bool success = false;
    do
    {
        auto response_ = client->async_send_request(std::make_shared<Request>(req));
        if (response_.wait_for(std::chrono::milliseconds(timeout_ms)) != std::future_status::ready)
        {
            retry_num--;
        }
        else if (response_.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready)
        {
            success = true;
            resp = *(response_.get());
            break;
        }
    }
    while (retry_num > -1);
    return success;
}



bool position_is_initialized(Telemetry::Position pos)
{
    bool not_initialized = false;
    Telemetry::Position pose = Telemetry::Position();
    not_initialized = (pos.absolute_altitude_m == 0 and pos.latitude_deg == 0 and pos.longitude_deg == 0 and pos.relative_altitude_m) or not_initialized;

    return !not_initialized;
}



/**
 * Normalizes an angle to be between -pi and pi.
 *
 * This function takes an angle in radians as input and returns a normalized angle
 * between -pi and pi. It achieves this by iteratively subtracting or adding 2*pi
 * until the angle falls within the specified range.
 *
 * @param angle The angle in radians to be normalized.
 * @return The normalized angle in radians between -pi and pi.
 */
double normalize_angle(double angle)
{
    if (angle > M_PI)
        angle -= 2 * M_PI;
    if (angle <= -M_PI)
        angle += 2 * M_PI;
    else return angle;

    return normalize_angle(angle);
}


/**
 * Converts a global angle to a local angle.
 *
 * This function assumes a right-handed coordinate system where the positive
 * x-axis points to the right and the positive y-axis points upwards. It
 * assumes that angles are measured counter-clockwise from the positive x-axis.
 *
 * The function converts a global angle to a local angle by subtracting it from
 * pi/2. This means that a zero global angle (pointing to the right) will be
 * converted to a local angle of pi/2 (pointing upwards).
 *
 * @param angle The angle in radians in the global coordinate system.
 * @return The angle in radians in the local coordinate system.
 */
double global_to_local(double angle)
{
    return - angle + M_PI_2;
}

/**
 * Converts a local angle to a global angle.
 *
 * This function assumes a right-handed coordinate system where the positive
 * x-axis points to the right and the positive y-axis points upwards. It
 * assumes that angles are measured counter-clockwise from the positive x-axis.
 *
 * The function converts a local angle to a global angle by subtracting it from
 * pi/2. This means that a local angle of pi/2 (pointing upwards) will be
 * converted to a global angle of zero (pointing to the right).
 *
 * @param angle The angle in radians in the local coordinate system.
 * @return The angle in radians in the global coordinate system.
 */
double local_to_global(double angle)
{
    return - angle + M_PI_2;
}

// JSON
json telemetry_data;

// Initialize ZMQ
zmq::context_t ctx{1};
zmq::socket_t socket{ctx, zmq::socket_type::pub};

void isok(int num) 
{
    std::cout << "All is ok ---> " << num << "\n";
}

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
        {"Heading", 0},
        {"GPSsat1", 0},
        {"GPSsat2", 0}
    };
}





class CommandActionClient : public rclcpp::Node
{
    public:
        using Command = privyaznik_msgs::action::Command;
        using GoalHandleCommand = rclcpp_action::ClientGoalHandle<Command>;

        explicit CommandActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
        : Node("command_action_client", node_options), goal_done_(false)
        {
            cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); 
            sub_options.callback_group = cb_group;
            cmd_string_sub = this->create_subscription<std_msgs::msg::String>("/npu/commands", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&CommandActionClient::commands_from_npu_callback, this, _1), sub_options);
            cmd_string_pub = this->create_publisher<std_msgs::msg::String>("/npu/copter_feedback", 10);
            this->client_ptr_ = rclcpp_action::create_client<Command>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "command_action");
        }

        bool is_goal_done() const
        {
            return this->goal_done_;
        }

        void send_goal(Command::Goal &goal_msg)
        {
            using namespace std::placeholders;

            // this->timer_->cancel();

            RCLCPP_INFO_STREAM(this->get_logger(), "339");

            this->goal_done_ = false;

            json feedback = {
                {"type", 1},
                {"cmd", 0},
                {"height", 0},
                {"result", false},
                {"message", ""}
            };

            RCLCPP_INFO_STREAM(this->get_logger(), "351");

            if (!this->client_ptr_) {
                feedback.at("message") = "Ошибка: сервер дрона не запущен.";
            }
            if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
                feedback.at("message") = "Ошибка: сервер дрона не отвечает.";
                this->goal_done_ = true;
            }

            // if (feedback.at("message") != "")
            // {
                std_msgs::msg::String msg;
                msg.data = feedback.dump();
                this->cmd_string_pub->publish(msg);
                RCLCPP_INFO_STREAM(this->get_logger(), "366");
                // return;
            // }

            RCLCPP_INFO_STREAM(this->get_logger(), "Sending goal");


            auto send_goal_options = rclcpp_action::Client<Command>::SendGoalOptions();
            send_goal_options.goal_response_callback =
            std::bind(&CommandActionClient::goal_response_callback, this, _1);
            send_goal_options.feedback_callback =
            std::bind(&CommandActionClient::feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
            std::bind(&CommandActionClient::result_callback, this, _1);
            auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

    private:
        rclcpp_action::Client<Command>::SharedPtr client_ptr_;
        // rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_string_sub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_string_pub;
        bool goal_done_;
        rclcpp::CallbackGroup::SharedPtr cb_group;
        rclcpp::SubscriptionOptions sub_options;

        void goal_response_callback(GoalHandleCommand::SharedPtr goal_handle)
        {
            json feedback = {
                {"type", 1},
                {"cmd", 0},
                {"height", 0},
                {"result", false},
                {"message", ""}
            };
            if (!goal_handle) {
            // RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            feedback["message"] = "Задача отклонена.";
            } else {
            // RCLCPP_INFO_STREAM(this->get_logger(), "Goal accepted by server, waiting for result");
            feedback["message"] = "Задача принята на выполнение. Ожидайте результата.";
            feedback["result"] = true;
            }
            std_msgs::msg::String msg;
            msg.data = feedback.dump();
            cmd_string_pub->publish(msg);
        }


        void commands_from_npu_callback(std_msgs::msg::String msg)
        {
            // json jmsg = json::get_to(msg.data);
            json jmsg;
            jmsg = json::parse(msg.data);
            // RCLCPP_ERROR_STREAM(this->get_logger(), jmsg.dump());
            json feedback = {
                {"type", 1},
                {"cmd", 0},
                {"height", 0},
                {"result", false},
                {"message", ""},
            };
            // RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(423));
            Command::Goal goal_msg;
            try
            {
                // RCLCPP_WARN_STREAM(this->get_logger(), std::to_string(427));
                if (jmsg.at("type") == 0)
                {
                    if (jmsg.at("result") == true)
                    {
                        feedback.at("message") = "Ошибка: команда сформирована неверно. Поле result = true.";
                    }
                    // RCLCPP_ERROR_STREAM(this->get_logger(), std::to_string(430));
                    if (jmsg.at("cmd") == 0)
                    {
                        goal_msg.cmd = goal_msg.CMD_TAKEOFF;
                        // RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(434));
                        try 
                        {
                            if (jmsg.at("height") > 0)
                            {
                                // RCLCPP_WARN_STREAM(this->get_logger(), std::to_string(439));
                                goal_msg.data.push_back(jmsg.at("height"));
                            }
                            else
                            {
                                // RCLCPP_ERROR_STREAM(this->get_logger(), std::to_string(444));
                                feedback.at("message") = "Ошибка: Заданная высота слишком мала. Заданное значение[метров]: " + std::to_string(float(jmsg.at("height")));
                            }
                        }
                        catch (std::exception &e)
                        {
                            // RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(450));
                            feedback.at("message") = "Ошибка: Поле высоты не было задано.";
                        }
                    }
                    else if (jmsg.at("cmd") == 1) 
                    {
                        // RCLCPP_WARN_STREAM(this->get_logger(), std::to_string(456));
                        goal_msg.cmd = goal_msg.CMD_LAND;
                    }
                    else
                    {
                        // RCLCPP_ERROR_STREAM(this->get_logger(), std::to_string(461));
                        feedback.at("message") = "Ошибка: Тип команды задан неверно.";
                    }
                }
                else
                {
                    // RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(467));
                    feedback.at("message") = "Ошибка: Получен результат вместо команды.";
                }
            }
            catch (std::exception &e)
            {
                // RCLCPP_WARN_STREAM(this->get_logger(), std::to_string(473));
                feedback.at("message") = "Ошибка: сообщение сформировано неверно.";
            }

            if (feedback.at("message") == "")
            {
                // RCLCPP_ERROR_STREAM(this->get_logger(), std::to_string(479));
                send_goal(goal_msg);
            }
            else
            {
                std_msgs::msg::String feedback_msg;
                feedback_msg.data = feedback.dump();
                cmd_string_pub->publish(feedback_msg);
                // RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(484));
            }
            // RCLCPP_WARN_STREAM(this->get_logger(), std::to_string(489));
        }


        void feedback_callback(
            GoalHandleCommand::SharedPtr,
            const std::shared_ptr<const Command::Feedback> feedback)
        {
            feedback->message.back();
        }

        void result_callback(const GoalHandleCommand::WrappedResult & result)
        {
            json feedback = {
                {"type", 1},
                {"cmd", 0},
                {"height", 0},
                {"result", false},
                {"message", ""}
            };

            this->goal_done_ = true;
            std_msgs::msg::String msg;

            if (result.result->result == privyaznik_msgs::action::Command::Result::RES_FAILED)
            {
                    feedback["message"] = "Задача была прервана.";
                    msg.data = feedback.dump();
                    cmd_string_pub->publish(msg);
                    return;
            }
            else if (result.result->result == privyaznik_msgs::action::Command::Result::RES_SUCCESS)
            {
                    feedback["message"] = "Задача выполнена успешно.";
                    feedback["result"] = true;
                    msg.data = feedback.dump();
                    cmd_string_pub->publish(msg);
                    return;
            }
            // switch (result.code) {
            //     case rclcpp_action::ResultCode::SUCCEEDED:
            //         feedback["message"] = "Задача выполнена успешно.";
            //         feedback["result"] = true;
            //         break;
            //     case rclcpp_action::ResultCode::ABORTED:
            //         feedback["message"] = "Задача была прервана.";
            //         msg.data = feedback.dump();
            //         cmd_string_pub->publish(msg);
            //         return;
            //     case rclcpp_action::ResultCode::CANCELED:
            //         feedback["message"] = "Задача была отменена.";
            //         msg.data = feedback.dump();
            //         cmd_string_pub->publish(msg);
            //         return;
            //     default:
            //         feedback["message"] = "Ошибка: неизвестный результат выполнения.";
            //         msg.data = feedback.dump();
            //         cmd_string_pub->publish(msg);
            //         return;
            // }

            // RCLCPP_INFO_STREAM(this->get_logger(), "Result received");
            auto res = result.result->result;
            RCLCPP_INFO_STREAM(this->get_logger(), PRId32 << res);
            feedback["message"] = "Результат получен: " + res;
            msg.data = feedback.dump();
            cmd_string_pub->publish(msg);
        }
};



class MavsdkBridgeNode : public rclcpp::Node
{
    

    public:
        int32_t old_num_satellites;

        MavsdkBridgeNode()
        : Node("minimal_subscriber"), mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}}
        {
            std::string default_username = "firefly";
            std::string username = "firefly";
            
            this->declare_parameter("username", default_username);
            this->get_parameter_or("username", username, default_username);


            cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); 
            sub_options.callback_group = cb_group;
            // sub_options.qos_overriding_options.
            // sub_options.
            sub_prec_landing = this->create_subscription<geometry_msgs::msg::Vector3>("camera/landing_position", 10, std::bind(&MavsdkBridgeNode::prec_land_callback, this, _1), sub_options);
            // sub_commands = this->create_subscription<privyaznik_msgs::msg::Command>("commands", 10, std::bind(&MavsdkBridgeNode::commands_callback, this, _1), sub_options);
            sub_imu_data = this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&MavsdkBridgeNode::imu_data_callback, this, _1), sub_options);
            sub_gps1_data = this->create_subscription<mavros_msgs::msg::GPSRAW>("/mavros/gpsstatus/gps1/raw", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&MavsdkBridgeNode::gps_data1_callback, this, _1), sub_options);
            sub_gps2_data = this->create_subscription<mavros_msgs::msg::GPSRAW>("/mavros/gpsstatus/gps2/raw", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&MavsdkBridgeNode::gps_data2_callback, this, _1), sub_options);

            utm_to_wgs_client = this->create_client<privyaznik_msgs::srv::UtmToWgs>("utm_to_wgs", rmw_qos_profile_default, cb_group);
            wgs_to_utm_client = this->create_client<privyaznik_msgs::srv::WgsToUtm>("wgs_to_utm", rmw_qos_profile_default, cb_group);
            // logic_timer = this->create_wall_timer(50ms, std::bind(&MavsdkBridgeNode::logic_in_timer, this), cb_group);
            // cmd_service = this->create_service <privyaznik_msgs::srv::Command>("commands", std::bind(&MavsdkBridgeNode::commands_callback, this, _1, _2), rmw_qos_profile_default, cb_group);

            cmd_action_service = rclcpp_action::create_server<privyaznik_msgs::action::Command>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "command_action",
            std::bind(&MavsdkBridgeNode::handle_goal, this, _1, _2),
            std::bind(&MavsdkBridgeNode::handle_cancel, this, _1),
            std::bind(&MavsdkBridgeNode::handle_accepted, this, _1));
            // std::bind(&MavsdkBridgeNode::commands_callback, this, _1));

            // cmd_service = this->create_service<privyaznik_msgs::srv::Command>("commands", std::bind(&MavsdkBridgeNode::commands_callback, this, _1, _2), rmw_qos_profile_default, cb_group);
            
            data_timer = this->create_wall_timer(50ms, std::bind(&MavsdkBridgeNode::data_callback, this));
            info_timer = this->create_wall_timer(750ms, std::bind(&MavsdkBridgeNode::info_output, this));
            
            gps1_cb_save = std::make_shared<mavros_msgs::msg::GPSRAW>();
            gps2_cb_save = std::make_shared<mavros_msgs::msg::GPSRAW>();

            ConnectionResult connection_result = mavsdk.add_any_connection(mavlink_addr);

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

            mission = std::make_unique<mavsdk::MissionRaw>(system.value());
            action = std::make_unique<mavsdk::Action>(system.value());
            telemetry = std::make_unique<mavsdk::Telemetry>(system.value());
            mavlink_passthrough = std::make_unique<MavlinkPassthrough>(system.value());

            auto param_handle = Param{*system};

            telemetry = std::make_unique<Telemetry>(system.value());

            // We want to listen to the altitude of the drone at 1 Hz.
            telemetry->set_rate_position_async(1.0, [](Telemetry::Result set_rate_result)
            {
                std::cerr << "Setting position rate info: " << set_rate_result << "\n\n";
            });

            create_json();
            if (username == "firefly")
            {
                socket.bind(zmq_connect_address);
            }
            else 
            {
                std::string default_zmq_connect_address = "tcp://127.0.0.1:8080";
                socket.bind(default_zmq_connect_address);
            }

            telemetry->subscribe_home([this](Telemetry::Position home_in){home_position = home_in;});

            telemetry->subscribe_position([this](Telemetry::Position position) 
            {
                telemetry_data["Altitude"] = position.relative_altitude_m;
                telemetry_data["Latitude"] = position.latitude_deg;
                telemetry_data["Longtitude"] = position.longitude_deg;
                current_position = position;
            });

            telemetry->subscribe_heading([this](Telemetry::Heading hdd)
            {
                telemetry_data["Heading"] = hdd.heading_deg;
                current_heading = hdd.heading_deg;
            });
        }


        void wgs_to_utm(privyaznik_msgs::srv::WgsToUtm::Request request)
        {
            wgs_to_utm_response_future = std::make_shared<rclcpp::Client<privyaznik_msgs::srv::WgsToUtm>::FutureAndRequestId>(this->wgs_to_utm_client->async_send_request(std::make_shared<privyaznik_msgs::srv::WgsToUtm::Request>(request)));
        }



        void utm_to_wgs(privyaznik_msgs::srv::UtmToWgs::Request request)
        {
            utm_to_wgs_response_future = std::make_shared<rclcpp::Client<privyaznik_msgs::srv::UtmToWgs>::FutureAndRequestId>(this->utm_to_wgs_client->async_send_request(std::make_shared<privyaznik_msgs::srv::UtmToWgs::Request>(request)));
        }


        Telemetry::Position get_curr_position()
        {
            return current_position;
        }


    private:
        Mavsdk mavsdk;
        bool is_in_flight;
        std::optional<std::shared_ptr<mavsdk::System>> system;
        std::unique_ptr<Telemetry> telemetry;
        std::unique_ptr<MavlinkPassthrough> mavlink_passthrough;
        std::unique_ptr<mavsdk::MissionRaw> mission;
        std::unique_ptr<mavsdk::Action> action;

        Telemetry::Position current_position;
        Telemetry::Position home_position;
        double current_heading;

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_prec_landing;
        rclcpp::Subscription<privyaznik_msgs::msg::Command>::SharedPtr sub_commands;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_data;
        rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr sub_gps1_data;
        rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr sub_gps2_data;

        rclcpp::Client<privyaznik_msgs::srv::UtmToWgs>::SharedPtr utm_to_wgs_client;
        rclcpp::Client<privyaznik_msgs::srv::WgsToUtm>::SharedPtr wgs_to_utm_client;

        rclcpp::TimerBase::SharedPtr logic_timer;
        rclcpp::TimerBase::SharedPtr _timer;

        rclcpp::CallbackGroup::SharedPtr cb_group;
        rclcpp::SubscriptionOptions sub_options;

        rclcpp::Service<privyaznik_msgs::srv::Command>::SharedPtr cmd_service;
        rclcpp_action::Server<privyaznik_msgs::action::Command>::SharedPtr cmd_action_service;


        std::shared_ptr<rclcpp::Client<privyaznik_msgs::srv::WgsToUtm>::FutureAndRequestId> wgs_to_utm_response_future;
        std::shared_ptr<rclcpp::Client<privyaznik_msgs::srv::UtmToWgs>::FutureAndRequestId> utm_to_wgs_response_future;


        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const privyaznik_msgs::action::Command::Goal> goal)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request with order " << goal->cmd);
            (void)uuid;
        
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }


        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<privyaznik_msgs::action::Command>>goal_handle)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<privyaznik_msgs::action::Command>> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&MavsdkBridgeNode::commands_callback, this, _1), goal_handle}.detach();
        }


        void commands_callback(const std::shared_ptr<GoalHandlePrivyaznik> goal_handle)
        {
            const auto goal = goal_handle->get_goal();
            auto res = std::make_shared<privyaznik_msgs::action::Command::Result>();

            std::vector<float> data = goal->data;

            switch (goal->cmd)
            {
                case privyaznik_msgs::action::Command::Goal::CMD_LAND:
                    res->result = try_land(data);
                    break;
                case privyaznik_msgs::action::Command::Goal::CMD_TAKEOFF:
                    res->result = try_takeoff(data);
                    break;
                case privyaznik_msgs::action::Command::Goal::CMD_MOVE:
                    //res->result = try_move(data);
                    break;
                default:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "mavsdk_bridge: unknown command.");
                    
            }
            if (res->result == res->RES_SUCCESS)
            {
                goal_handle->succeed(res);
            } 
            else if (res->result == res->RES_FAILED)
            {
                goal_handle->abort(res);
            }
        }

        void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr attitude)
        {
            tf2::Quaternion q_container;
            tf2Scalar roll, pitch, yaw;
            q_container.setX(attitude->orientation.x);
            q_container.setY(attitude->orientation.y);
            q_container.setZ(attitude->orientation.z);
            q_container.setW(attitude->orientation.w);

            tf2::Matrix3x3 mtrx(q_container);
            mtrx.getRPY(roll, pitch, yaw);

            telemetry_data["Roll"] = roll;
            telemetry_data["Pitch"] = pitch;
            telemetry_data["Yaw"] = yaw;
        }

        inline void gps_fix_type_info(const mavros_msgs::msg::GPSRAW::SharedPtr gps_fix, int gps_id)
        {
            switch (gps_fix->fix_type)
            {
                case mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_NO_GPS:
                    RCLCPP_WARN_STREAM(this->get_logger(), "NO GPS_" << gps_id << "." << std::endl);
                    break;
                case mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_NO_FIX:
                    RCLCPP_WARN_STREAM(this->get_logger(), "NO FIX GPS_" << gps_id << "." << std::endl);
                    break;
                case mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_2D_FIX:
                    RCLCPP_INFO_STREAM(this->get_logger(), "2D FIX GPS_" << gps_id << "." << std::endl);
                    break;
                case mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_3D_FIX:
                    RCLCPP_INFO_STREAM(this->get_logger(), "3D FIX GPS_" << gps_id << "." << std::endl);
                    break;
            }
        }

        mavros_msgs::msg::GPSRAW::SharedPtr gps1_cb_save;
        mavros_msgs::msg::GPSRAW::SharedPtr gps2_cb_save;

        void gps_data1_callback(mavros_msgs::msg::GPSRAW::SharedPtr gps1)
        {
            // gps_fix_type_info(gps1, 1);
            gps1_cb_save = gps1;
        }

        void gps_data2_callback(mavros_msgs::msg::GPSRAW::SharedPtr gps2)
        {
            // gps_fix_type_info(gps2, 2);
            gps2_cb_save = gps2;
        }

        void prec_land_callback(const geometry_msgs::msg::Vector3::SharedPtr coords) // Проблема такого колбэка в том, что const не позволяет модифицировать переменные за пределами функции
        {
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

            // Function to pack and send the message
            auto send_landing_target_message = [this, &landing_target_msg](MavlinkAddress mavlink_address, uint8_t channel)
            {
                mavlink_message_t message;
                mavlink_msg_landing_target_encode(mavlink_passthrough->get_our_sysid(), channel, &message, &landing_target_msg);
                return message;
            };

            // Send the message using queue_message
            mavsdk::MavlinkPassthrough::Result res = mavlink_passthrough->queue_message(send_landing_target_message);
            error_msg(res);
        }

        /**
         * @brief Выполняет попытку сесть, как только получена команда. Если не получилось, пытается еще.
         * @param data std::vector<float> - не несет полезных значений для этой команды
        */
        uint8_t try_land(std::vector<float> data)
        {    
            if (telemetry->landed_state() == Telemetry::LandedState::OnGround)
            {
                return privyaznik_msgs::action::Command::Result::RES_FAILED;
            }
            mavsdk::Action::Result result = action->return_to_launch();
            if (result != mavsdk::Action::Result::Success) 
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Land failed");
                return privyaznik_msgs::action::Command::Result::RES_FAILED;
            }
            while (telemetry->landed_state() != Telemetry::LandedState::Landing)
            {
                std::this_thread::sleep_for(20ms);
            }
            while (telemetry->landed_state() != Telemetry::LandedState::OnGround)
            {
                std::this_thread::sleep_for(100ms);
                if (telemetry->landed_state() == Telemetry::LandedState::InAir)
                {
                    return privyaznik_msgs::action::Command::Result::RES_FAILED;
                }
            }
            return privyaznik_msgs::action::Command::Result::RES_SUCCESS;
        }



        /**
         * @brief Выполняет попытку подвинуться на заданные смещение и вращение (по рысканию) относительно позиции, в которой получил команду
         * @param data std::vector<float> - {x, y, z, yaw}
        */
        uint8_t try_move(std::vector<float> data)
        {
            float altitude, yaw, start_yaw, res;
            double easting, northing;
            easting = data[0]; 
            northing = data[1]; 
            altitude = data[2];
            yaw = data[3];


            privyaznik_msgs::srv::WgsToUtm::Request req;
            req.abs_altitude = current_position.absolute_altitude_m;
            req.rel_altitude = current_position.relative_altitude_m;
            req.latitude = current_position.latitude_deg;
            req.longitude = current_position.longitude_deg;
            req.rel_altitude = current_position.relative_altitude_m;

            wgs_to_utm(req);

            while (wgs_to_utm_response_future->wait_for(50ms) != std::future_status::ready) RCLCPP_WARN_STREAM(this->get_logger(), "Try move converting WGS to UTM. Waiting...");
            std::shared_ptr<privyaznik_msgs::srv::WgsToUtm_Response> response = wgs_to_utm_response_future->get();

// double local_yaw = global_to_local(current_heading);
// easting = data.at(0) * cos(local_yaw) - data.at(1) * sin(local_yaw) + response->easting;
// northing = data.at(0) * sin(local_yaw) + data.at(1) * cos(local_yaw) + response->northing;
            altitude = data.at(2) + current_position.absolute_altitude_m; 
// yaw = local_to_global(normalize_angle(data.at(3) + local_yaw));
            

            start_yaw = data[3]; // Установка начальной ориентации
            yaw = start_yaw * (180/M_PI) + current_heading; // Перевод в градусы

            yaw = (yaw-int(yaw)) + ( int(yaw) % 360 ); // Нормализация в пределах от -360 до 360

            if (fabs(yaw + current_heading) <= 180 ) // Если модуль итоговой ориентации меньше 180
            {
                res = yaw;
            }
            else
            {
                if (yaw > 0) 
                {
                    res = yaw - 360;
                }
                else
                {
                    res = yaw + 360;
                }
            
            }
            
            RCLCPP_INFO_STREAM(this->get_logger(), "Goal: " << res);
            // res = (res + current_heading) * M_PI / 180 ;
            res = (res + current_heading);

            privyaznik_msgs::srv::UtmToWgs::Request u_req;
            u_req.easting = easting + response->easting;
            u_req.northing = northing + response->northing;
            u_req.zone_letter = response->zone_letter;
            u_req.zone_number = response->zone_number;
            utm_to_wgs(u_req);

            while (utm_to_wgs_response_future->wait_for(50ms) != std::future_status::ready) RCLCPP_WARN_STREAM(this->get_logger(), "Try move converting UTM to WGS. Waiting...");
            std::shared_ptr<privyaznik_msgs::srv::UtmToWgs_Response> goal = utm_to_wgs_response_future->get();

            mavsdk::Action::Result result = action->goto_location(goal->latitude + 0.001, goal->longitude, altitude, res);
            if (result != mavsdk::Action::Result::Success) 
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Move failed");
                return privyaznik_msgs::action::Command::Result::RES_FAILED;
            }
            return privyaznik_msgs::action::Command::Result::RES_SUCCESS;
        }



        /**
         * @brief Выполняет попытку взлета на заданную высоту.
         * @param data std::vector<float> - {height}
        */
        uint8_t try_takeoff(std::vector<float> data)
        {   
            if (telemetry->landed_state() != Telemetry::LandedState::OnGround)
            {   
                // this->action
                return privyaznik_msgs::action::Command::Result::RES_FAILED;
            }

            set_home_position_to_current_position();

            
            float z_coor;
            if (data.size()>0)
            {

                z_coor = data[0]; 
                if (z_coor<=80)
                {

                    while (!telemetry->health_all_ok())
                    {
                        std::cout << "Waiting for system to be ready\n";
                        sleep_for(seconds(1));
                    }
                    
                    std::cout << "System ready\n";
                    std::cout << "Creating and uploading mission\n";

                    // Telemetry::Position home_position;
                    // while (isnan(home_position.absolute_altitude_m) == true) 
                    // {   
                    //     telemetry->subscribe_position([&home_position](Telemetry::Position position) 
                    //     {   

                    //         std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
                    //         std::cout<< "x coordinate "<<position.latitude_deg<<std::endl;
                    //         std::cout<< "y coordinate "<<position.longitude_deg<<std::endl;
                    //         home_position = position;
                    //     });

                    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    // }

                    uint16_t seq = 0;

                    std::vector<mavsdk::MissionRaw::MissionItem> land_raw_items;

                    land_raw_items.push_back(create_mission_item(0, 0, 16, 1, 1, 0, 0, 0, 0, home_position.latitude_deg, home_position.longitude_deg, home_position.absolute_altitude_m, 0));
                    land_raw_items.push_back(create_mission_item(1, 3, 22, 0, 1, 0, 0, 0, 0, 0, 0, z_coor, 0));  
                    
                    std::vector<mavsdk::MissionRaw::MissionItem> mission_items_plan = land_raw_items;

                    std::cout << "Uploading mission...\n";
                    
                    const mavsdk::MissionRaw::Result upload_result = mission->upload_mission(mission_items_plan);

                    if (upload_result != mavsdk::MissionRaw::Result::Success) 
                    {
                        std::cerr << "Mission upload failed: " << upload_result << ", exiting.\n";
                        return privyaznik_msgs::action::Command::Result::RES_FAILED;
                    }

                    std::cout << "Arming...\n";
                    const mavsdk::Action::Result arm_result = action->arm();
                    if (arm_result != mavsdk::Action::Result::Success) 
                    {
                        std::cerr << "Arming failed: " << arm_result << '\n';
                        return privyaznik_msgs::action::Command::Result::RES_FAILED;
                    }
                    std::cout << "Armed.\n";

                    const mavsdk::Action::Result takeoff_result = action->takeoff();
                    if (takeoff_result != mavsdk::Action::Result::Success) 
                    {
                        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
                        return privyaznik_msgs::action::Command::Result::RES_FAILED;
                    }
                    std::atomic<bool>mission_complete = false;
                    // Before starting the mission, we want to be sure to subscribe to the mission progress.
                    auto ms_handle = mission->subscribe_mission_progress([&mission_complete](mavsdk::MissionRaw::MissionProgress mission_progress) 
                    {
                        std::cout << "Mission status update: " << mission_progress.current << " / "
                                << mission_progress.total << '\n';
                        if (mission_progress.current == mission_progress.total)
                        {
                            // We can only set a flag here. If we do more request inside the callback,
                            // we risk blocking the system.

                            mission_complete = true;
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
                            mission->unsubscribe_mission_progress(ms_handle);
                            return privyaznik_msgs::action::Command::Result::RES_FAILED;
                        }
                    }    

                    while (not mission_complete)
                    {
                        std::this_thread::sleep_for(100ms);
                    }

                    mission->unsubscribe_mission_progress(ms_handle);
                    return privyaznik_msgs::action::Command::Result::RES_SUCCESS;
                }
                else return privyaznik_msgs::action::Command::Result::RES_FAILED;

            }
            else return privyaznik_msgs::action::Command::Result::RES_FAILED;

            }

        

        void set_home_position_to_current_position()
        {
            mavlink_command_int_t cmd;
            cmd.command = MAV_CMD_DO_SET_HOME;
            cmd.param1=1;

            // Function to pack and send the message
            auto send_landing_target_message = [this, &cmd](MavlinkAddress mavlink_address, uint8_t channel) {
                mavlink_message_t message;
                mavlink_msg_command_int_encode(mavlink_passthrough->get_our_sysid(), channel, &message, &cmd);
                return message;
            };

            // Send the message using queue_message
            mavsdk::MavlinkPassthrough::Result res = mavlink_passthrough->queue_message(send_landing_target_message);
            switch (res) 
            {
                case mavsdk::MavlinkPassthrough::Result::Success:
                    RCLCPP_INFO_STREAM(this->get_logger(), "Set home message queued successfully." << std::endl);
                    break;
                case mavsdk::MavlinkPassthrough::Result::Unknown:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "An unknown error occurred while sending the message." << std::endl);
                    break;
                case mavsdk::MavlinkPassthrough::Result::ConnectionError:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Error connecting to the autopilot. Check the connection and try again." << std::endl);
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandNoSystem:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Mavlink system not available." << std::endl);
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandBusy:
                    RCLCPP_WARN_STREAM(this->get_logger(), "System is busy. Please try again later." << std::endl);
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandDenied:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Command to set home has been denied." << std::endl);
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandUnsupported:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Sending set home message is not supported by this autopilot." << std::endl);
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandTimeout:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Timeout while sending the set home message. Check connection and retry." << std::endl);
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandTemporarilyRejected:
                    std::cout << "Sending set home message temporarily rejected. Try again later." << std::endl;
                    break;
                case mavsdk::MavlinkPassthrough::Result::CommandFailed:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to send the set home message." << std::endl);
                    break;
                // Add cases for other specific error handling if needed...
                default:
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Unhandled Mavlink passthrough result: " << static_cast<int>(res) << std::endl);
            }
        }

        void data_callback()
        {
            telemetry_data["Time"] = this->get_clock()->now().seconds();

            telemetry_data["GPSsat1"] = gps1_cb_save->satellites_visible;
            telemetry_data["GPSsat2"] = gps2_cb_save->satellites_visible;

            std::string serial_data = telemetry_data.dump();
            socket.send(zmq::str_buffer("Telemetry_data_topic"), zmq::send_flags::sndmore);
            socket.send(zmq::buffer(serial_data));
        }

        void info_output() 
        {
            std::cout << "\nTIME GOES BACK ---> " << std::to_string(this->get_clock()->now().seconds()) << "\n\n"; 
            std::cout << "GPS_1 num of satellites: " << gps1_cb_save->satellites_visible << "\n"; // old_num_satellites
            std::cout << "GPS_2 num of satellites: " << gps2_cb_save->satellites_visible << "\n\n";

            std::cout << "----- JSON -----" << "\n";
            for (auto& [key,value] : telemetry_data.items())
            {
                std::cout << key << " : " << value << "\n";
            }
            std::cout << "----------------" << "\n";

            gps_fix_type_info(gps1_cb_save, 1);
            gps_fix_type_info(gps2_cb_save, 2);
        }

        rclcpp::TimerBase::SharedPtr data_timer;
        rclcpp::TimerBase::SharedPtr info_timer;
};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<MavsdkBridgeNode>());
    // rclcpp::shutdown();

    rclcpp::executors::MultiThreadedExecutor mt_exec;
    auto mavsdk_node = std::make_shared<MavsdkBridgeNode>();
    auto cmd_client = std::make_shared<CommandActionClient>();
    

    mt_exec.add_node(mavsdk_node);
    mt_exec.add_node(cmd_client);
    mt_exec.spin();

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