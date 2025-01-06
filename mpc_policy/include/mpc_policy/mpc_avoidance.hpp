#ifndef MPC_POLICY_MPC_AVOIDANCE_HPP
#define MPC_POLICY_MPC_AVOIDANCE_HPP

#include <eigen3/Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>

class MPC_avoidance
{
    public:
        MPC_avoidance(ros::NodeHandle &nh);
        ~MPC_avoidance();

    private:
        float roll_, pitch_, yaw_;

        bool received_angular_ = false;
        bool received_pose_ = false;



        geometry_msgs::PointStamped target_point_;
        geometry_msgs::TwistStamped current_twist_;
        geometry_msgs::PoseStamped  current_pose_;

        Eigen::Vector3d U_ref_;
        

        Eigen::VectorXd QPSolution_pass = Eigen::VectorXd::Zero(15);
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6,6); // State matrix
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6,3); // Input matrix
        Eigen::VectorXd Q = Eigen::VectorXd::Zero(6); // State cost matrix
        Eigen::Vector3d R; // Input cost matrix
        Eigen::MatrixXd H,G,E;

        int N = 5;  // Prediction horizon

        OsqpEigen::Solver solver;

        void targetPointCallback(const geometry_msgs::PointStamped::ConstPtr &target_msg);

        void currentAngularCallback(const geometry_msgs::TwistStamped::ConstPtr &angular_msg);

        void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

        void MPC_Matrices(const Eigen::VectorXd& x_ref,
                          const Eigen::VectorXd& u_ref,
                          Eigen::MatrixXd& H,
                          Eigen::MatrixXd& G,
                          Eigen::MatrixXd& E);
        ros::NodeHandle nh_;

        ros::Subscriber target_sub_;
        ros::Subscriber current_angular_sub_;
        ros::Subscriber current_pose_sub_;

        ros::Publisher velocity_pub_;
};

#endif