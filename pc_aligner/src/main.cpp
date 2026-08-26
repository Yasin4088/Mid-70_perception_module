#include <rclcpp/rclcpp.hpp>

#include "aligner_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AlignerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
