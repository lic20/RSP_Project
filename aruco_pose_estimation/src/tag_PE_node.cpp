#include <aruco_pose_estimation/tag_pose_estimation.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    rclcpp::spin( std::make_shared<image_subscriber>("aruco") );

    rclcpp::shutdown();

    return 0;
}