#include<rspmain/rspmain.hpp>
// #include<pnpm/pnpm.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rsp{

  rspmain::rspmain( const std::string& name ): Node( name ){
      
      // pnp_client = std::make_shared<rsp::action_client>("client");

      gripper_client_joints = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path");
      gripper_client_joints->wait_for_service();
      std::cout << "gripper client joints is ready" << std::endl;
    }

  void rspmain::call() {

    open_manipulator_msgs::msg::JointPosition jp;
    jp.joint_name.push_back("joint1");
    jp.joint_name.push_back("joint2");
    jp.joint_name.push_back("joint3");
    jp.joint_name.push_back("joint4");
    jp.joint_name.push_back("gripper");

    // Set to Home Position
    jp.position.push_back(-0.003);
    jp.position.push_back(-1.043);
    jp.position.push_back(0.402);
    jp.position.push_back(1.263);
    jp.position.push_back(0.01);

    jp.max_accelerations_scaling_factor = 0.5;
    jp.max_velocity_scaling_factor = 0.5;

    auto request_gri = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    request_gri->path_time = 5.0;
    request_gri->joint_position = jp;
    std::cout << "A" << std::endl;
    gripper_client_joints->async_send_request(request_gri, std::bind(&rspmain::gripper_client_drop_callback, this, std::placeholders::_1));
    std::cout << "B" << std::endl;


  }
  void rspmain::camera_client_callback(const rclcpp::Client<rsp_msgs::srv::GetTrans>::SharedFuture future) {
    std::cout << "camera client callback: " << std::endl;
    auto response = future.get();
    std::this_thread::sleep_for(1000ms);
    std::cout << response->trans[0] << std::endl;
    if (response->trans[0] == -1 || response->trans[0] == 0) {
      std::cout << "ERROR: CANNOT SEE AR TAG!!" << std::endl;
      return;
    }

    std::shared_ptr<rsp::action_client> pnp_client = std::make_shared<rsp::action_client>("client");
    // std::cout << pnp_client->test << " " << std::endl;
    geometry_msgs::msg::Pose p;
    p.position.x = response->trans[0] ;
    p.position.y = response->trans[1];
    p.position.z = response->trans[2];
    p.orientation.x = response->trans[3];
    p.orientation.y = response->trans[4];
    p.orientation.z = response->trans[5];
    p.orientation.w = response->trans[6];
    pnp_client->call(p);
}

  void rspmain::gripper_client_drop_callback(const rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future) {
    std::cout << "Position Adjusted" << std::endl;
    auto response = future.get();
    std::this_thread::sleep_for(5000ms);
    std::cout << response->is_planned << std::endl;

    camera_client = this->create_client<rsp_msgs::srv::GetTrans>("getTrans_srv");

    std::cout << "main client: wait for service" << std::endl;
    camera_client->wait_for_service();
    std::cout<<"camera client ready" << std::endl;

    auto camera_req = std::make_shared<rsp_msgs::srv::GetTrans::Request>();
    camera_req->pwd = "0415";
    camera_client->async_send_request(camera_req, std::bind(&rspmain::camera_client_callback, this, std::placeholders::_1));
  }
}