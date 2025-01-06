#include <depth_map_process/RBF_Target.hpp>
#include <cmath>

using namespace cv;
using namespace std;

Mat rbf(const Mat &X, const Mat &Y, double sigma)
{
    Mat rbfMatrix;
    exp(-(X.mul(X) + Y.mul(Y)) / (2 * sigma * sigma), rbfMatrix);
    return rbfMatrix;
}

void meshgrid(int start, int end, Mat &X, Mat &Y)
{
    vector<double> range;
    for (int i = start; i < end; ++i)
    {
        range.push_back(i);
    }

    repeat(Mat(range).t(), range.size(), 1, X);
    repeat(Mat(range), 1, range.size(), Y);
}



namespace DepthMapProcess
{

    DepthMapProcess::RBF_Target::RBF_Target(ros::NodeHandle &nh)
        : nh_(nh), it_(nh_), depth_image_received_(false), local_pose_received_(false)
    {   
        nh_.param<std::string>("mpc_aware/cam_intr",cam_intr_param,"/cam_intr");
        istringstream iss(cam_intr_param);
        iss >> cx >> cy >> fx >> fy;
        // cout << cx << cy << fx << fy << endl;
        // cout << cam_intr_param << endl;
        Eigen::Matrix3f cam_intrin;
        cam_intrin << fx,0,cx,0,fy,cy,0,0,1;

        cam_intrin_inv = cam_intrin.inverse();

        cam2body_R << 0,0,1,-1,0,0,0,-1,0;
        cam2body_T << 0.1,0,0;
        // Subscribe to the depth image topic
        depth_sub_ = it_.subscribe("/iris_0/realsense/depth_camera/depth/image_raw", 1, &DepthMapProcess::RBF_Target::depthImageCallback, this);

        // Subscribe to the local pose topic
        local_pose_sub_ = nh_.subscribe("iris_0/mavros/vision_pose/pose", 1, &DepthMapProcess::RBF_Target::localPoseCallback, this);
        
        Target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/depth_map_process/target", 1);
        
        meshgrid(-windowSize, windowSize, X, Y);

        standardRBFMatrix = rbf(X, Y, sigma);
        standardRBFMatrix.convertTo(standardRBFMatrix, CV_32F);
        // cout << standardRBFMatrix.size() << endl;
        ROS_INFO("DepthMapProcess initialized.");
    }

    DepthMapProcess::RBF_Target::~RBF_Target()
    {
        ROS_INFO("DepthMapProcess destroyed.");
    }


    void DepthMapProcess::RBF_Target::depthImageCallback(const sensor_msgs::ImageConstPtr &depth_msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg, "mono8");
            depth_image_ = cv_ptr->image;

            // cv::Mat image_re = depth_image_.reshape(1);
            // double minVal, maxVal;
            // cv::Point minLoc, maxLoc;
            // cv::minMaxLoc(image_re, &minVal, &maxVal, &minLoc, &maxLoc);
            // cout << "Min: " << minVal << endl;
            // cout << "Max: " << maxVal << endl;
            // cout << "Min Loc: " << minLoc.x << ", " << minLoc.y << endl;
            // cout << "Max Loc: " << maxLoc.x << ", " << maxLoc;
            // cout << depth_image_ << endl;
            // disparity_image = disparity_image.reshape(width, height, 4);

            cv::patchNaNs(depth_image_, 20.0);
            depth_image_received_ = true;
            //ROS_INFO("Depth image received.");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void DepthMapProcess::RBF_Target::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        local_pose_ = *pose_msg;
        local_pose_received_ = true;
        Eigen::Quaternionf q(local_pose_.pose.orientation.w, local_pose_.pose.orientation.x,
                             local_pose_.pose.orientation.y, local_pose_.pose.orientation.z);
        Eigen::Matrix3f rotation_uav = q.toRotationMatrix();

        Eigen::Vector3f position_uav;
        position_uav << local_pose_.pose.position.x, local_pose_.pose.position.y, local_pose_.pose.position.z;

        //ROS_INFO("Local pose received.");

        // If both depth image and local pose are received, perform computation
        if (depth_image_received_ && local_pose_received_)
        {
            // method 1
            // Get the x and y coordinates of the target point in the image plane
            // double maxWeightedSum = 0;
            // int maxX = 0, maxY = 0;

            // for (int x0 = windowSize; x0 < depth_image_.cols - windowSize; ++x0)
            // {
            //     for (int y0 = windowSize; y0 < depth_image_.rows - windowSize; ++y0)
            //     {
                    
            //         Mat localDepthPatch = depth_image_(Rect(x0 - windowSize, y0 - windowSize, 2 * windowSize, 2 * windowSize));

            //         double currentWeightedSum = sum(standardRBFMatrix.mul(localDepthPatch))[0];
                    
            //         if (currentWeightedSum > maxWeightedSum)
            //         {
            //             maxWeightedSum = currentWeightedSum;
            //             maxX = x0;
            //             maxY = y0;
            //         }
            //     }
            // }
            // cv::Point maxLoc(maxX, maxY);
        
        /*method 2*/
            
            cv::Point maxLoc = FindCenterPoint(depth_image_);
            //cout << "size of depth image" <<depth_image.size() << endl;
            double target_depth = depth_image_.at<float>(maxLoc.y, maxLoc.x);
            // cout << "width: " << depth_image.cols << ", height: " << depth_image.rows << endl;
            std::cout << "x, y, depth in img respectively: " << maxLoc.x  << ", " << maxLoc.y << "," << target_depth << endl;
            
            Eigen::Vector3f target_in_image;
            target_in_image << float(target_depth * maxLoc.x), float(target_depth * maxLoc.y), float(target_depth);
            
            Eigen::Vector3f target_in_body  = (cam2body_R * (cam_intrin_inv * target_in_image) - cam2body_T) * 0.5;
            Eigen::Vector3f target_in_world = rotation_uav * target_in_body + position_uav;
    
            geometry_msgs::PointStamped Target_point = geometry_msgs::PointStamped();
            std::cout << "Target in world: " << target_in_world.transpose() << endl;
            Target_point.point.x = target_in_world.x(); 
            Target_point.point.y = target_in_world.y(); 
            Target_point.point.z = target_in_world.z() < 1.5 ? 1.5 : target_in_world.z();
            Target_point.point.z = target_in_world.z() > 3 ? 3 : target_in_world.z();
            count ++;
            if (count == 1){
                Target_pub_.publish(Target_point);
                count = 0;
            }
            // Target_pub_.publish(Target_point);

            // Reset flags after processing
            depth_image_received_ = false;
            local_pose_received_ = false;
        }
    }
    cv::Point DepthMapProcess::RBF_Target::FindCenterPoint(cv::Mat map2find)
    {
        cv::Mat depth_image;
        cv::filter2D(map2find, depth_image, CV_32FC1, standardRBFMatrix,cv::Point(-1,-1),(0,0),cv::BORDER_CONSTANT);

        
        double maxVal = 0;
        double minVal = 0;
        minMaxLoc(depth_image, &minVal, &maxVal);

        depth_image = (depth_image - minVal) / (maxVal - minVal);

        Mat dst = Mat::zeros(360, 640, CV_8UC1);
        depth_image.convertTo(dst, CV_8U, 255.0);

        cv::threshold(dst, depth_image, 200, 255, cv::THRESH_BINARY);

        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        dilate(depth_image, depth_image, element);
        erode (depth_image, depth_image, element);

        std::vector<std::vector<Point>> contours;
        findContours(depth_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        int max_count = 0;
        double max_area = 0;
        for(int i = 0; i < contours.size(); ++i)
        {
            double area = contourArea(contours[i]);
            if (area > max_area)
            {
                max_count = i;
                max_area = area;
            }
        }

        Moments mu = moments(contours[max_count]);
        cv::Point maxLoc(int(mu.m10/mu.m00), int(mu.m01/mu.m00));
        return maxLoc;
        
    }

}
 // namespace my_package
int main(int argc, char **argv){
    ros::init(argc, argv, "depth_map_process_node");

    ros::NodeHandle nh;

    DepthMapProcess::RBF_Target rbf_target(nh);

    ros::spin();

}