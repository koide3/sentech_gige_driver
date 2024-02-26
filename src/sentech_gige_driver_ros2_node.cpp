#include <sentech_gige_driver_ros2.hpp>

int main(int argc, char** argv) {
  try {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<sentech_gige_driver::SentechGigeDriverNode>(options);

    rclcpp::spin(node);
    node->stop();

    rclcpp::shutdown();
  } catch (const GenICam::GenericException& e) {
    std::cerr << "exception: " << e.GetDescription() << std::endl;
  }

  return 0;
}