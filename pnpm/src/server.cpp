#include <pnpm/pnpm.hpp>

int main( int argc, char** argv ){

  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<rsp::action_server>("pnpm") );
  

  // std::shared_ptr<rsp::action_server> s1 = std::make_shared<rsp::action_server>("pnpm") ;
 

  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(s1);
  // executor.add_node(s2);

  // executor.spin();
  
  //rclcpp::spin(s1);
  //rclcpp::spin(s2);
  
  rclcpp::shutdown();

  return 0;

}
