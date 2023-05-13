#include <pnpm/pnpm.hpp>
// #include <open_manipulator_x_controller/open_manipulator_x_controller.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rsp{

  action_server::action_server( const std::string& name ):
    Node( name ){

    server = rclcpp_action::create_server<rsp_msgs::action::RSP>
      ( this,
	name, 
	std::bind( &action_server::goal_callback, this, _1, _2 ),
	std::bind( &action_server::cancel_callback, this, _1 ),
	std::bind( &action_server::accept_goal, this, _1 ) );
    
  }
  
  rclcpp_action::GoalResponse
  action_server::goal_callback(const rclcpp_action::GoalUUID&,
			       rsp_msgs::action::RSP::Goal::ConstSharedPtr){

    //   return rclcpp_action::GoalResponse::REJECT;
    flag = 1;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    
  }
  
  rclcpp_action::CancelResponse
  action_server::cancel_callback
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<rsp_msgs::action::RSP>> ){
    std::cout << "cancel" << std::endl;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void
  action_server::accept_goal
  (const std::shared_ptr<rclcpp_action::ServerGoalHandle<rsp_msgs::action::RSP>>
   goal_handle ){
    
    kins_client =  this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("goal_task_space_path");
    

    std::cout << "kins_client and gripper client created" << std::endl;
    kins_client->wait_for_service();
    std::cout << "kins_client is ready" << std::endl;


    auto pose_goal = goal_handle->get_goal()->posandori;
    pose_goal.position.z = pose_goal.position.z + 0.01;

    open_manipulator_msgs::msg::KinematicsPose kp;
    kp.pose = pose_goal;
    kp.max_accelerations_scaling_factor = 0.5;
    kp.max_velocity_scaling_factor = 0.5;
    kp.tolerance = 0.01;

    auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
    request->kinematics_pose = kp;
    request->end_effector_name = "gripper";
    request->path_time = 5.0;
    kins_client->async_send_request(request, std::bind(&action_server::kins_client_callback, this, std::placeholders::_1));
    std::cout << "Kins_Client req sent" << std::endl;


    auto result = std::make_shared<rsp_msgs::action::RSP::Result>();
    result->sucorfail = 1;
    goal_handle->succeed(result);
    
  }
  void action_server::kins_client_callback(const rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedFuture future) {
    std::cout << "kins client callback: " << std::endl;
    auto response = future.get();
    std::this_thread::sleep_for(5000ms);
    std::cout << response->is_planned << std::endl;

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&action_server::topic_callback, this, _1));

    // std::this_thread::sleep_for(1000ms);
    

  }

  void action_server::gripper_client_callback(const rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future) {
    std::cout << "gripper client callback: " << std::endl;
    auto response = future.get();
    std::this_thread::sleep_for(5000ms);
    std::cout << response->is_planned << std::endl;
  }

  void action_server::topic_callback(const sensor_msgs::msg::JointState& joints){
    // std::cout << "Gripper Position:" << std::endl;
    // std::cout << joints.position[4] << std::endl;

    curr_joint = joints;

    // std::cout << "Show current Joints: " << std::endl;

    // std::cout << curr_joint.position[0] << std::endl;
    // std::cout << curr_joint.position[4] << std::endl;

    if (flag) {
      gripper_client = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_tool_control");
      gripper_client->wait_for_service();
      std::cout << "gripper_client is ready" << std::endl;

      open_manipulator_msgs::msg::JointPosition jp;
      jp.joint_name.push_back("joint1");
      jp.joint_name.push_back("joint2");
      jp.joint_name.push_back("joint3");
      jp.joint_name.push_back("joint4");
      jp.joint_name.push_back("gripper");

      
      jp.position.push_back(curr_joint.position[0]);
      jp.position.push_back(curr_joint.position[1]);
      jp.position.push_back(curr_joint.position[2]);
      jp.position.push_back(curr_joint.position[3]);
      jp.position.push_back(-0.01);

      jp.max_accelerations_scaling_factor = 0.5;
      jp.max_velocity_scaling_factor = 0.5;

      auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
      request->path_time = 5.0;
      request->joint_position = jp;

      gripper_client->async_send_request(request, std::bind(&action_server::gripper_client_callback, this, std::placeholders::_1));

      flag = 0;
    }
  }


  action_client::action_client( const std::string& name ) :
    Node(name){
    client =
      rclcpp_action::create_client<rsp_msgs::action::RSP>( this, "pnpm" );
      // std::cout << "action client created" << std::endl;
    client->wait_for_action_server();
    // std::cout << "action client got a server" << std::endl;
  }

  void action_client::call( const geometry_msgs::msg::Pose& command ){

    rsp_msgs::action::RSP::Goal goal;
    goal.posandori = command;

    rclcpp_action::Client<rsp_msgs::action::RSP>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&action_client::response_callback, this, _1);
    options.feedback_callback =
      std::bind(&action_client::feedback_callback, this, _1, _2);
    options.result_callback = 
      std::bind(&action_client::result_callback, this, _1);

    client->async_send_goal( goal, options );
    
  }
  
  void action_client::response_callback
  ( rclcpp_action::ClientGoalHandle<rsp_msgs::action::RSP>::SharedPtr )
  {
    std::cout << "client response" << std::endl;
  }
  void action_client::feedback_callback
  ( rclcpp_action::ClientGoalHandle<rsp_msgs::action::RSP>::SharedPtr ,
    const std::shared_ptr<const rsp_msgs::action::RSP::Feedback> feedback )
  {
    std::cout << "feedback" << std::endl
	      << feedback->message << std::endl;
  }
  void action_client::result_callback
  ( const rclcpp_action::ClientGoalHandle<rsp_msgs::action::RSP>::WrappedResult&
    result )
  {
    std::cout << "result" << std::endl
	      << (int)result.result->sucorfail << std::endl;
  }
  
}
