#ifndef DEPTH_MAP_PROCESS_RBF_TARGET_HPP
#define DEPTH_MAP_PROCESS_RBF_TARGET_HPP

// Standard Libraries
#include <string>
#include <utility> // For std::pair
#include <vector>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Dense>

// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>


namespace DepthMapProcess
{

    class RBF_Target
    {
    public:
        // Constructor and Destructor
        RBF_Target(ros::NodeHandle &nh);
        ~RBF_Target();

        // Public Methods
        // std::tuple<float, float, float> Computation_process(const cv::Mat &depth_image, const geometry_msgs::PoseStamped &local_pose);
        // cv::Mat rbf(const cv::Mat& X, const cv::Mat& Y, double sigma);
        int count = 0;
        int sigma = 1;
        int windowSize = 30;

        cv::Mat X, Y;
        cv::Mat standardRBFMatrix; 
    private:

        float cx,cy,fx,fy;
        std::string cam_intr_param;

        Eigen::Matrix3f cam_intrin_inv;

        Eigen::Matrix3f cam2body_R;
        Eigen::Vector3f cam2body_T;
        // Private Methods
        void depthImageCallback(const sensor_msgs::ImageConstPtr &depth_msg);
        void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
        cv::Point FindCenterPoint(cv::Mat map2find);


        // ROS Components
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber depth_sub_;
        ros::Subscriber local_pose_sub_;

        ros::Publisher Target_pub_;

        // Storage for synchronized data
        cv::Mat depth_image_;
        geometry_msgs::PoseStamped local_pose_;
        bool depth_image_received_;
        bool local_pose_received_;
    };

} // namespace my_package

#endif // MY_PACKAGE_DEPTH_MAP_PROCESS_HPP


