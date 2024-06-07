#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using nlohmann::json;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class JsonCmdClient : public rclcpp::Node
{
    public:
        JsonCmdClient()
        : Node("json_cmd_client"), count_(0) // инициалзация полей
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("npu/commands", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile()); // создание паблишера
            subscriber = this->create_subscription<std_msgs::msg::String>("npu/copter_feedback", 10, std::bind(&JsonCmdClient::feedback_cb, this, std::placeholders::_1)); // создание паблишера
            timer_ = this->create_wall_timer(500ms, std::bind(&JsonCmdClient::timer_callback, this)); // инициализация таймера. Привязка к таймеру нода, а к ноду колбэка(таймер -> нод -> колбэк) 
        }

    private:  
        rclcpp::TimerBase::SharedPtr timer_; // Инициализация вышеиспоользуемых полей. В классе так можно
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
        size_t count_;

        void timer_callback() // Сама функция колбэка
        {
            json msg =
            {
                {"type", 0},
                {"cmd", 0},
                {"height", 50.0},
                {"result", false},
                {"message", ""}
            };
            auto message = std_msgs::msg::String();
            message.data = msg.dump();
            RCLCPP_INFO_STREAM(this->get_logger(), message.data);
            publisher_->publish(message);

            this->timer_->cancel();
        }


        void feedback_cb(std_msgs::msg::String msg)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), msg.data);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JsonCmdClient>());
    rclcpp::shutdown();
    return 0;
}
