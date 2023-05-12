#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rsp_msgs/action/rsp.hpp>
#include <open_manipulator_msgs/srv/set_kinematics_pose.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <open_manipulator_msgs/msg/kinematics_pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rsp{

  class action_server : public rclcpp::Node {
  private:
    rclcpp_action::Server<rsp_msgs::action::RSP>::SharedPtr server;
  public:
    rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr kins_client;
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr gripper_client;
    action_server( const std::string& name );

    rclcpp_action::GoalResponse
    goal_callback(const rclcpp_action::GoalUUID& id,
		  rsp_msgs::action::RSP::Goal::ConstSharedPtr goal);
    rclcpp_action::CancelResponse
    cancel_callback
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<rsp_msgs::action::RSP>>
     goal_handle );
    void
    accept_goal
    (const std::shared_ptr<rclcpp_action::ServerGoalHandle<rsp_msgs::action::RSP>>
     goal_handle );

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    sensor_msgs::msg::JointState curr_joint;
    int flag;

    void topic_callback(const sensor_msgs::msg::JointState& joints);

    void kins_client_callback(const rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedFuture future);

    void gripper_client_callback(const rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future);

  };

  class action_client : public rclcpp::Node {
  private:
    rclcpp_action::Client<rsp_msgs::action::RSP>::SharedPtr client;
    
  public:
    
    action_client( const std::string& name );

    void call( const geometry_msgs::msg::Pose& command );
    
    void response_callback
    ( rclcpp_action::ClientGoalHandle<rsp_msgs::action::RSP>::SharedPtr handle );
    void feedback_callback
    ( rclcpp_action::ClientGoalHandle<rsp_msgs::action::RSP>::SharedPtr handle,
      const std::shared_ptr<const rsp_msgs::action::RSP::Feedback> feedback );
    void result_callback
    ( const rclcpp_action::ClientGoalHandle<rsp_msgs::action::RSP>::WrappedResult&
      result );
    
  };

}
