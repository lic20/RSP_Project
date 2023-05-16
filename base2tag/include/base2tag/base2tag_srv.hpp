#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <open_manipulator_msgs/msg/kinematics_pose.hpp>
#include <Eigen/Dense>
#include <rsp_msgs/srv/get_trans.hpp>
class base2tag_srv : public rclcpp::Node
{
private:
    rclcpp::Service<rsp_msgs::srv::GetTrans>::SharedPtr getTrans_srv;
    rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr kin_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr tag_pose_subscriber;
    float srv_response[7];
    float kin_msg[7];
    float tag_msg[7];

    Eigen::MatrixXf gripper2cam = Eigen::MatrixXf (4,4);
    Eigen::MatrixXf base2gripper = Eigen::MatrixXf(4,4);
    Eigen::MatrixXf cam2tag = Eigen::MatrixXf(4,4);

public:
    base2tag_srv(const std::string &name);
    void srv_callback(const std::shared_ptr<rsp_msgs::srv::GetTrans::Request> request,
                  std::shared_ptr<rsp_msgs::srv::GetTrans::Response> response);

    void kin_callback(const open_manipulator_msgs::msg::KinematicsPose &msg);
    void tag_callback(const geometry_msgs::msg::Transform &msg);
};