#include <base2tag/base2tag_srv.hpp>
// #include<std_msgs/msg/float32.hpp>
#include <math.h>
using namespace std::chrono_literals;
base2tag_srv::base2tag_srv(const std::string &name) : Node(name)
{
    getTrans_srv = create_service<rsp_msgs::srv::GetTrans>("getTrans_srv",
                                                           std::bind(&base2tag_srv::srv_callback, this, std::placeholders::_1, std::placeholders::_2));
    kin_pose_subscriber = create_subscription<open_manipulator_msgs::msg::KinematicsPose>("kinematics_pose", 10,
                                                                                          std::bind(&base2tag_srv::kin_callback, this, std::placeholders::_1));
    tag_pose_subscriber = create_subscription<geometry_msgs::msg::Transform>("/aruco/pose", 10,
                                                                             std::bind(&base2tag_srv::tag_callback, this, std::placeholders::_1));
    gripper2cam << 0, 0, 1, -0.0503, -1, 0, 0, 0.0321, 0, -1, 0, 0.0428, 0, 0, 0, 1;
}

void base2tag_srv::srv_callback(const std::shared_ptr<rsp_msgs::srv::GetTrans::Request> request,
                                std::shared_ptr<rsp_msgs::srv::GetTrans::Response> response)
{
    // float a[9] = request->mat1;
    // float b[9] = request->mat2;

    // std_msgs::float32 result[9];

    // for (int i = 0; i < 3; i++)
    // {
    //     for (int j = 0; j < 3; j++)
    //     {
    //         response->mat_result[3 * i + j] = request->mat1[3 * i] * request->mat2[j] + request->mat1[3 * i + 1] * request->mat2[j + 3] + request->mat1[3 * i + 2] * request->mat2[j + 6];
    //     }
    // }

    // // response->mat_result = result;
    // std::cout << "service: done" << std::endl;

    if (request->pwd == "0415")
    {
        if (tag_msg[0] == -1 && tag_msg[1] == -1 && tag_msg[2] == -1 && tag_msg[3] == -1 && tag_msg[4] == -1 && tag_msg[5] == -1 && tag_msg[6] == -1)
        {
            srv_response[0] = -1;
            srv_response[1] = -1;
            srv_response[2] = -1;
            srv_response[3] = -1;
            srv_response[4] = -1;
            srv_response[5] = -1;
            srv_response[6] = -1;
            std::cout << "No tag found"<<std::endl;
        }
        else
        {
            base2gripper(0, 3) = kin_msg[0];
            base2gripper(1, 3) = kin_msg[1];
            base2gripper(2, 3) = kin_msg[2];

            base2gripper(0, 0) = 2 * (kin_msg[6] * kin_msg[6] + kin_msg[3] * kin_msg[3]) - 1;
            base2gripper(0, 1) = 2 * (kin_msg[3] * kin_msg[4] - kin_msg[6] * kin_msg[5]);
            base2gripper(0, 2) = 2 * (kin_msg[3] * kin_msg[5] + kin_msg[6] * kin_msg[4]);
            base2gripper(1, 0) = 2 * (kin_msg[3] * kin_msg[4] + kin_msg[6] * kin_msg[5]);
            base2gripper(1, 1) = 2 * (kin_msg[6] * kin_msg[6] + kin_msg[4] * kin_msg[4]) - 1;
            base2gripper(1, 2) = 2 * (kin_msg[4] * kin_msg[5] - kin_msg[6] * kin_msg[3]);
            base2gripper(2, 0) = 2 * (kin_msg[3] * kin_msg[5] - kin_msg[6] * kin_msg[4]);
            base2gripper(2, 1) = 2 * (kin_msg[4] * kin_msg[5] + kin_msg[6] * kin_msg[3]);
            base2gripper(2, 2) = 2 * (kin_msg[6] * kin_msg[6] + kin_msg[5] * kin_msg[5]) - 1;
            base2gripper(3, 0) = 0;
            base2gripper(3, 1) = 0;
            base2gripper(3, 2) = 0;
            base2gripper(3, 3) = 1;

            cam2tag(0, 3) = tag_msg[0];
            cam2tag(1, 3) = tag_msg[1];
            cam2tag(2, 3) = tag_msg[2];

            cam2tag(0, 0) = 2 * (tag_msg[6] * tag_msg[6] + tag_msg[3] * tag_msg[3]) - 1;
            cam2tag(0, 1) = 2 * (tag_msg[3] * tag_msg[4] - tag_msg[6] * tag_msg[5]);
            cam2tag(0, 2) = 2 * (tag_msg[3] * tag_msg[5] + tag_msg[6] * tag_msg[4]);
            cam2tag(1, 0) = 2 * (tag_msg[3] * tag_msg[4] + tag_msg[6] * tag_msg[5]);
            cam2tag(1, 1) = 2 * (tag_msg[6] * tag_msg[6] + tag_msg[4] * tag_msg[4]) - 1;
            cam2tag(1, 2) = 2 * (tag_msg[4] * tag_msg[5] - tag_msg[6] * tag_msg[3]);
            cam2tag(2, 0) = 2 * (tag_msg[3] * tag_msg[5] - tag_msg[6] * tag_msg[4]);
            cam2tag(2, 1) = 2 * (tag_msg[4] * tag_msg[5] + tag_msg[6] * tag_msg[3]);
            cam2tag(2, 2) = 2 * (tag_msg[6] * tag_msg[6] + tag_msg[5] * tag_msg[5]) - 1;
            cam2tag(3, 0) = 0;
            cam2tag(3, 1) = 0;
            cam2tag(3, 2) = 0;
            cam2tag(3, 3) = 1;

            Eigen::MatrixXf res = base2gripper * gripper2cam * cam2tag;

            response->trans[0] = res(0, 3);
            response->trans[1] = res(1, 3);
            response->trans[2] = res(2, 3);

            response->trans[6] = sqrt(1 + res(0, 0) + res(1, 1) + res(2, 2)) / 2;
            response->trans[3] = (res(2, 1) - res(1, 2)) / (4 * response->trans[6]);
            response->trans[4] = (res(0, 2) - res(2, 0)) / (4 * response->trans[6]);
            response->trans[5] = (res(1, 0) - res(0, 1)) / (4 * response->trans[6]);
            std::cout << "Calculated!!" << std::endl;
        }
    }
    else
    {
        srv_response[0] = -1;
        srv_response[1] = -1;
        srv_response[2] = -1;
        srv_response[3] = -1;
        srv_response[4] = -1;
        srv_response[5] = -1;
        srv_response[6] = -1;
        std::cout << "Wrong password! You are not a valid user!"<<std::endl;
    }
}

void base2tag_srv::kin_callback(const open_manipulator_msgs::msg::KinematicsPose &msg)
{
    kin_msg[0] = msg.pose.position.x;
    kin_msg[1] = msg.pose.position.y;
    kin_msg[2] = msg.pose.position.z;
    kin_msg[3] = msg.pose.orientation.x;
    kin_msg[4] = msg.pose.orientation.y;
    kin_msg[5] = msg.pose.orientation.z;
    kin_msg[6] = msg.pose.orientation.w;
}

void base2tag_srv::tag_callback(const geometry_msgs::msg::Transform &msg)
{
    if (msg.translation.x == -1 && msg.translation.y == -1 && msg.translation.z == -1 && msg.rotation.x == -1 && msg.rotation.y == -1 && msg.rotation.z == -1 && msg.rotation.w == -1)
    {
        // No tag found
        tag_msg[0] = -1;
        tag_msg[1] = -1;
        tag_msg[2] = -1;
        tag_msg[3] = -1;
        tag_msg[4] = -1;
        tag_msg[5] = -1;
        tag_msg[6] = -1;
    }
    else
    {
        tag_msg[0] = msg.translation.x;
        tag_msg[1] = msg.translation.y;
        tag_msg[2] = msg.translation.z;
        tag_msg[3] = msg.rotation.x;
        tag_msg[4] = msg.rotation.y;
        tag_msg[5] = msg.rotation.z;
        tag_msg[6] = msg.rotation.w;
    }
}