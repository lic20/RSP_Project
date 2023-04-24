#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <aruco_pose_estimation/tag_pose_estimation.hpp>
using namespace cv;

image_subscriber::image_subscriber(const std::string &name) : Node(name)
{
    image_sub = create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 10,
                                                             std::bind(&image_subscriber::callback, this, std::placeholders::_1));
}

void image_subscriber::callback(const sensor_msgs::msg::Image &msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat img = cv_ptr->image;
    Mat imgCopy;

    cv::aruco::detectMarkers(img, dictionary, corners, ids,detectorParams,rejected);
    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(img, corners, ids);
        int nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        // cameraMatrix = cameraMatrix.reshape(1,3);
        // distCoeffs = distCoeffs.reshape(1, 1);
        cv::aruco::estimatePoseSingleMarkers(corners,0.05,cameraMatrix,distCoeffs,rvecs,tvecs);
        for (int i = 0;i < rvecs.size();i++)
		{
			cv::aruco::drawAxis(img, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);
		}

        cv::imshow("Display Image", img);
        //std::cout<<"1"<<std::endl;
    }
    else
    {
        cv::imshow("Display Image", img);
        std::cout<<size(rejected)<<std::endl;
    }
        

    cv::waitKey(1);
}
