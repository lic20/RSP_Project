#include <pnpm/pnpm.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  std::shared_ptr<rsp::action_client> client = std::make_shared<rsp::action_client>("client");
  geometry_msgs::msg::Pose p;
  p.position.x = 0.2;
  p.position.y = 0.2;
  p.position.z = 0.05;
  // p.orientation.x = 0.3;
  // p.orientation.y = 0.3;
  // p.orientation.z = 0.0;
  // p.orientation.w = 1;
  client->call(p);
  rclcpp::spin(client);
  rclcpp::shutdown();

  return 0;
}
