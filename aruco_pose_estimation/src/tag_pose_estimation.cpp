#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <aruco_pose_estimation/tag_pose_estimation.hpp>
#include <math.h>
using namespace cv;

image_subscriber::image_subscriber(const std::string &name) : Node(name)
{
    image_sub = create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 10,
                                                             std::bind(&image_subscriber::callback, this, std::placeholders::_1));
    transform_pub = create_publisher<geometry_msgs::msg::Transform>("/aruco/pose",10);
}

void image_subscriber::callback(const sensor_msgs::msg::Image &msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat img = cv_ptr->image;
    //std::cout << img.rows <<std::endl;
    //std::cout << img.cols <<std::endl;
    Mat imgCopy;

    cv::aruco::detectMarkers(img, dictionary, corners, ids,detectorParams,rejected);
    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(img, corners, ids);
        int nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        // cameraMatrix = cameraMatrix.reshape(1,3);
        // distCoeffs = distCoeffs.reshape(1, 1);
        cv::aruco::estimatePoseSingleMarkers(corners,0.024,cameraMatrix,distCoeffs,rvecs,tvecs);
        // for (size_t i = 0; i < 3; i++)
        // {
        //     std::cout<< cameraMatrix <<std::endl;
        // }
        
        for (int i = 0;i < rvecs.size();i++)
		{
			cv::aruco::drawAxis(img, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);
            
            std::cout << "x: "<< rvecs[i][0] << " "  << "y: "<< rvecs[i][1] << " "  << "z: "<< rvecs[i][2]<<std::endl;
            std::cout << "-----------------------------------"<<std::endl;
            cv::Mat r;
            cv::Rodrigues(rvecs[i],r);
            std::cout<< r <<std::endl;
		}
        image_subscriber::publish_pose(rvecs,tvecs);

        cv::imshow("Display Image", img);
        //std::cout<<"1"<<std::endl;
    }
    else
    {
        cv::imshow("Display Image", img);
        //std::cout<<size(rejected)<<std::endl;
    }
        

    cv::waitKey(1);
}

void image_subscriber::publish_pose(const std::vector<cv::Vec3d>& r, const std::vector<cv::Vec3d>& t)
{
    geometry_msgs::msg::Transform msg;

    msg.translation.x = t[0][0];
    msg.translation.y = t[0][1];
    msg.translation.z = t[0][2];
    
    double theta = sqrt(pow(r[0][0],2)+pow(r[0][1],2)+pow(r[0][2],2));
    double norm_x = r[0][0]/theta;
    double norm_y = r[0][1]/theta;
    double norm_z = r[0][2]/theta;

    msg.rotation.x = norm_x*sin(theta/2);
    msg.rotation.y = norm_y*sin(theta/2);
    msg.rotation.z = norm_z*sin(theta/2);
    msg.rotation.w = cos(theta/2);

    transform_pub->publish(msg);
}
