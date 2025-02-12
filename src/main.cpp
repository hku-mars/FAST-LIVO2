#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include "fastlivo2/LIVMapper.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Allow undeclared params
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<LIVMapper>(options);

    auto it = std::make_shared<image_transport::ImageTransport>(node);
    node->initializeSubscribersAndPublishers(*it);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok())
    {
        executor.spin_once();
    }
    node->savePCD();

    rclcpp::shutdown();
    return 0;
}