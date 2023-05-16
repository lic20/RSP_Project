#include <pnpm/pnpm.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);

  std::shared_ptr<rsp::action_client> client = std::make_shared<rsp::action_client>("client");
  geometry_msgs::msg::Pose p;
  p.position.x = 0.25507784;
  p.position.y = 0.0001*2.7471;
  p.position.z = -0.001*8.37688;
  p.orientation.x = -1.4562803e-02;
  p.orientation.y = 1.6421275e-03;
  p.orientation.z = -6.6668338e-01;
  p.orientation.w = 7.4519700e-01;
  client->call(p);
  rclcpp::spin(client);
  rclcpp::shutdown();

  return 0;
}
