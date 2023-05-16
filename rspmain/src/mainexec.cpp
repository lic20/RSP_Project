#include <rspmain/rspmain.hpp>

int main( int argc, char** argv ){

    rclcpp::init(argc, argv);

    std::shared_ptr<rsp::rspmain> rsp_main = std::make_shared<rsp::rspmain>("rsp_main");
    rsp_main->call();
    rclcpp::spin(rsp_main);
    rclcpp::shutdown();

  return 0;
}