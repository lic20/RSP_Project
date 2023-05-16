#include<base2tag/base2tag_srv.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::cout<<"base2tag server ready"<<std::endl;
    rclcpp::spin(std::make_shared<base2tag_srv>("base2tag"));

    rclcpp::shutdown();

    return 0;
}