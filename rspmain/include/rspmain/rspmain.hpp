#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rsp_msgs/srv/get_trans.hpp>
#include<rsp_msgs/action/rsp.hpp>
#include<pnpm/pnpm.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rsp {

class rspmain : public rclcpp::Node {
  private:
    // rclcpp_action::Client<rsp_msgs::action::RSP>::SharedPtr pnp_client;
  public:
    rclcpp::Client<rsp_msgs::srv::GetTrans>::SharedPtr camera_client;
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr gripper_client_joints;
    rspmain(const std::string& name );

    void call();
    void camera_client_callback(const rclcpp::Client<rsp_msgs::srv::GetTrans>::SharedFuture future);
    void gripper_client_drop_callback(const rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future);
  };

}