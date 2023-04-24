#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/core/cvstd_wrapper.hpp>
#include<vector>
class image_subscriber : public rclcpp :: Node
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        //cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = std::make_shared<cv::aruco:: DetectorParameters>();
        //cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::Dictionary> dictionary = std::make_shared<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250));
        //cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        std::vector<double> intrinsic = {908.534057617188,0,643.08056640625,0,908.295959472656,356.429168701172,0,0,1};
        cv::Mat cameraMatrix = cv::Mat(intrinsic).reshape(1,3);
        
        std::vector<double> distortion = {0,0,0,0,0};
        cv::Mat distCoeffs = cv::Mat(distortion).reshape(1,1);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        
    public:
        image_subscriber(const std::string& name);
        void callback(const sensor_msgs::msg::Image& msg);
    };