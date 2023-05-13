#include <pnpm/pnpm.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  std::shared_ptr<rsp::action_client> client = std::make_shared<rsp::action_client>("client");
  geometry_msgs::msg::Pose p;
  p.position.x = -0.2019;
  p.position.y = -0.1508;
  p.position.z = 0.0512;
  p.orientation.x = -0.5329322;
  p.orientation.y = 0.4504065;
  p.orientation.z = 0.5883034;
  p.orientation.w = 0.4086763;
  client->call(p);
  rclcpp::spin(client);
  rclcpp::shutdown();

  return 0;
}
