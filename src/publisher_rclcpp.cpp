#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "Python.h"

using namespace std::chrono_literals;

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

	Py_Initialize();
	PyObject *pName = PyUnicode_FromString("python_pkg.subscriber_rclpy");

	PyObject *pModule = PyImport_Import(pName);
	if (!pModule)
	{
		printf("PyImport_Import script.py failed!\n");
		return 1;
	}
	// getting all attributes of the module
	PyObject *dict = PyModule_GetDict(pModule);

	// getting this subscriber class
	std::string py_class_name = "MinimalSubscriber";
	PyObject *py_class = PyDict_GetItemString(dict, py_class_name.c_str());

	// creating an instance of the class


	// starting the rclcpp node 
	rclcpp::init(argc, argv);
	// running the rclpy publisher node in a separate thread
	rclcpp::executors::SingleThreadedExecutor executor1;
	auto node1 = std::make_shared<MinimalPublisher>();
	executor1.add_node(node1);
	std::thread spinThread([&executor1]()
						   { executor1.spin(); });
	std::cout << "rclcpp thread started" << std::endl;

	// running the rclpy subscriber node from python and blocking the main thread
	PyObject* obj = PyObject_CallObject(py_class, NULL);

	Py_Finalize();
	// If node is interrupted, destroy the node and shutdown rclcpp
	std::cout << "Bye Bye, world!" << std::endl;
	rclcpp::shutdown();
	return 0;
}