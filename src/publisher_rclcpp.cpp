#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "Python.h"
#include <pybind11/embed.h> 

using namespace std::chrono_literals;
namespace py = pybind11;

class MinimalPublisher : public rclcpp::Node
{
public:
	MinimalPublisher()
		: Node("minimal_publisher"), count_(0)
	{
		publisher_ = this->create_publisher<std_msgs::msg::String>("Chatter", 10);
		timer_ = this->create_wall_timer(
			500ms, std::bind(&MinimalPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto message = std_msgs::msg::String();
		message.data = "Hello, world! " + std::to_string(count_++);
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		publisher_->publish(message);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;
};

int main(int argc, char *argv[])
{

	py::scoped_interpreter guard{}; // start the interpreter and keep it alive

	// import the python module
	py::module py_sub = py::module::import("python_pkg.subscriber_rclpy");



	// starting the rclcpp node 
	rclcpp::init(argc, argv);
	// running the rclpy publisher node in a separate thread
	rclcpp::executors::SingleThreadedExecutor executor1;
	auto node1 = std::make_shared<MinimalPublisher>();
	executor1.add_node(node1);
	std::thread spinThread([&executor1]()
						   { executor1.spin(); });
	std::cout << "rclcpp thread started" << std::endl;


	// creating an instance of the class
	// running the rclpy subscriber node from python and blocking the main thread
	py::object object = py_sub.attr("MinimalSubscriber")();

	// If node is interrupted, destroy the node and shutdown rclcpp
	std::cout << "Bye Bye, world!" << std::endl;
	rclcpp::shutdown();
	return 0;
}