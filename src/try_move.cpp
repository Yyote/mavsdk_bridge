#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "privyaznik_msgs/msg/command.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class CommandPub : public rclcpp::Node
{
    public:
    CommandPub()
    : Node("command_publisher"), count_(0) // инициалзация полей
    {
        publisher_ = this->create_publisher<privyaznik_msgs::msg::Command>("commands", 10); // создание паблишера
        timer_ = this->create_wall_timer(500ms, std::bind(&CommandPub::timer_callback, this)); // инициализация таймера. Привязка к таймеру нода, а к ноду колбэка(таймер -> нод -> колбэк) 
    }

    private:
    void timer_callback() // Сама функция колбэка
    {
        privyaznik_msgs::msg::Command msg;

        msg.cmd = msg.CMD_MOVE;
        msg.data.resize(4);
        msg.data.at(0) = 0;
        msg.data.at(1) = 0;
        msg.data.at(2) = 0;
        msg.data.at(3) = M_PI_2;

        publisher_->publish(msg);
        rclcpp::shutdown();
    }
    rclcpp::TimerBase::SharedPtr timer_; // Инициализация вышеиспоользуемых полей. В классе так можно
    rclcpp::Publisher<privyaznik_msgs::msg::Command>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPub>());
    rclcpp::shutdown();
    return 0;
}
