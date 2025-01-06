#include <mpc_policy/mpc_avoidance.hpp>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

MPC_avoidance::MPC_avoidance(ros::NodeHandle &nh):nh_(nh)
{
    //nh_.param("cam_intr",cam_intr);
    
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(N * (B.cols()));
    solver.data()->setNumberOfConstraints(N * (B.cols()));
    
    target_sub_ = nh_.subscribe("/depth_map_process/target", 1, &MPC_avoidance::targetPointCallback, this);
    current_pose_sub_ = nh_.subscribe("iris_0/mavros/vision_pose/pose", 1, &MPC_avoidance::currentPoseCallback, this);
    current_angular_sub_ = nh_.subscribe("iris_0/mavros/local_position/velocity_local", 1, &MPC_avoidance::currentAngularCallback, this);

    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("iris_0/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    // std::cout << "hello" << std::endl;
    
    A << 1,0,0.0476,0,0,0,
         0,1,0,0.0476,0,0,
         0,0,0.9048,0,0,0,
         0,0,0,0.9048,0,0,
         0,0,0,0,1,0.0476,
         0,0,0,0,0,0.9048;
    // A.resize(6,6);
    
    B << 0.0024, 0, 0,
         0, 0.0024, 0,
         0.0952, 0, 0,
         0, 0.0952, 0,
         0, 0, 0.0024,
         0, 0, 0.0952;

    // std::cout << B << std::endl;
    Q << 15,15,3,3,30,30;
    R << 4,4,30;

    U_ref_ << 8,0,0;

    ROS_INFO("MPC controller initialized.");
}

MPC_avoidance::~MPC_avoidance()
{
    ROS_INFO("MPCAvoidance destroyed.");
}


void MPC_avoidance::targetPointCallback(const geometry_msgs::PointStamped::ConstPtr &target_msg)
{
    target_point_ = *target_msg;
    // received_pose_ = true;
    // ROS_INFO("RECEIVED TARGET POINT");
    // tf2::Quaternion tarQuater(target_point_.pose.orientation.x, target_point_.pose.orientation.y,
    //                           target_point_.pose.orientation.z, target_point_.pose.orientation.w);
    // tf2::Matrix3x3 QuaterMat(tarQuater);
    // QuaterMat.getRPY(roll_, pitch_, yaw_);

}

void MPC_avoidance::currentAngularCallback(const geometry_msgs::TwistStamped::ConstPtr &angular_msg)
{
    current_twist_ = *angular_msg;
    received_angular_ = true;
    // ROS_INFO("RECEIVED CURRENT ANGULAR");
}

void MPC_avoidance::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    current_pose_ = *pose_msg;
    received_pose_ = true;
    if (received_angular_ && received_pose_)
    {
        Eigen::Quaternionf q(current_pose_.pose.orientation.w, current_pose_.pose.orientation.x,
                             current_pose_.pose.orientation.y, current_pose_.pose.orientation.z);
        Eigen::Vector3f eulerAngles = q.matrix().eulerAngles(2,1,0); //R-P-Y
        Eigen::VectorXd state_cur = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd state_ref = Eigen::VectorXd::Zero(6);
        state_cur << current_pose_.pose.position.x, current_pose_.pose.position.y, current_twist_.twist.linear.x,
                     current_twist_.twist.linear.y, eulerAngles.z(), current_twist_.twist.angular.z;
        // state_ref << target_point_.point.x, target_point_.point.y,0.5 * (target_point_.point.x - current_pose_.pose.position.x),
        //              (target_point_.point.y - current_pose_.pose.position.y),0,0;
        state_ref << target_point_.point.x, target_point_.point.y,0,0.5*(target_point_.point.y - current_pose_.pose.position.y),0,0;

        // std::cout << "state_ref: " << state_ref.transpose() << std::endl;

        MPC_Matrices(state_ref,U_ref_,H,G,E);

        Eigen::MatrixXd linearMatrix = Eigen::MatrixXd::Identity(N * B.cols(), N * B.cols());

        /*std::cout << "size of H: " << H.rows() << " " << H.cols() << std::endl;
        std::cout << "size of G: " << G.rows() << " " << G.cols() << std::endl;
        std::cout << "size of E: " << E.rows() << " " << E.cols() << std::endl;
        std::cout << "H:\n" << H << std::endl;
        std::cout << "G:\n" << G << std::endl;
        std::cout << "E:\n" << E << std::endl;*/

        solver.data()->setHessianMatrix(Eigen::SparseMatrix<double>(H.sparseView()));
        solver.data()->setLinearConstraintsMatrix(Eigen::SparseMatrix<double>(linearMatrix.sparseView()));
        
        Eigen::VectorXd f = G.transpose() * state_cur - E.transpose();
        
        /*std::cout << "H:\n" << H << std::endl;
        std::cout << "f:\n" << f.transpose() << std::endl;*/

        solver.data()->setGradient(f);

        Eigen::VectorXd upperBound(N * B.cols());
        upperBound.setConstant(OsqpEigen::INFTY);
        solver.data()->setUpperBound(upperBound);

        Eigen::VectorXd lowerBound(N * B.cols());
        lowerBound.setConstant(-OsqpEigen::INFTY);
        solver.data()->setLowerBound(lowerBound);
        
        if (!solver.initSolver()) std::cout << "Failed to init" << std::endl;
        
        solver.solveProblem();

        Eigen::VectorXd QPSolution = solver.getSolution();
        QPSolution = 0.1 * QPSolution_pass + 0.9 * QPSolution;
        QPSolution[0] = abs(QPSolution[0] - QPSolution_pass[0]) < 0.1 ? QPSolution[0] : QPSolution_pass[0] + 0.1 * (signbit(QPSolution[0] - QPSolution_pass[0]) == true ? -1 : 1);
        QPSolution[1] = abs(QPSolution[1] - QPSolution_pass[1]) < 0.1 ? QPSolution[1] : QPSolution_pass[1] + 0.1 * (signbit(QPSolution[1] - QPSolution_pass[1]) == true ? -1 : 1);
        // std::cout << "QPSolution: " << QPSolution.transpose() << std::endl;
        ROS_INFO("Setting velocity x: %.2f  y: %.2f  yaw: %.2f",QPSolution[0], QPSolution[1], QPSolution[2]);
        
        geometry_msgs::Twist velocity_angular;
        velocity_angular.linear.x  = QPSolution[0];
        velocity_angular.linear.y  = QPSolution[1];
        velocity_angular.linear.z  = 0.1 * (target_point_.point.z - current_pose_.pose.position.z);
        velocity_angular.angular.z = - QPSolution[2];
        velocity_pub_.publish(velocity_angular);

        solver.data()->clearHessianMatrix();
        solver.data()->clearLinearConstraintsMatrix();
        solver.clearSolver();
        received_angular_ = false;
        received_pose_ = false;
        QPSolution_pass = QPSolution;
    }

    // ROS_INFO("RECEIVED CURRENT POINT");
}

void MPC_avoidance::MPC_Matrices(const Eigen::VectorXd &x_ref, const Eigen::VectorXd &u_ref,
                                 Eigen::MatrixXd &H,
                                 Eigen::MatrixXd &G,
                                 Eigen::MatrixXd &E)
{
    // Get the sizes of matrices A and B
    int n = A.rows(); // A is n x n matrix
    int p = B.cols(); // B is n x p matrix

    // Initialize M matrix, (N+1)*n x n matrix
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero((N + 1) * n, n);
    M.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);

    // Initialize C matrix, (N+1)*n x N*p matrix
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero((N + 1) * n, N * p);

    // Define an n x n identity matrix
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(n, n);

    for (int i = 0; i < N; ++i)
    {
        // Rows slice for current iteration
        int row_start = (i + 1) * n;
        int row_end = (i + 2) * n;

        // std::cout << C.block(row_start, 0, n, N * p) << std::endl;
        // Update the C matrix
        C.block(row_start, 0, n, p) = tmp * B;

        C.block(row_start, p, n, N * p - p) = C.block(row_start - n, 0, n, N * p - p);

        // Update tmp and M
        tmp = A * tmp;
        M.block(row_start, 0, n, n) = tmp;
    }

    
    // std::cout << C << std::endl;
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero((N + 1) * n, (N + 1) * n);
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(N * p, N * p);


    for (int i = 0; i < N + 1; ++i)
    {
        Q_bar.block(i * n, i * n, n, n) = Q.asDiagonal();
        
    }

    for (int i = 0; i < N; ++i)
    {
        R_bar.block(i * p, i * p, p, p) = R.asDiagonal();
    }
    
    // Create reference vectors X_ref and U_ref
    Eigen::MatrixXd X_ref = x_ref.replicate(N + 1, 1);
    Eigen::MatrixXd U_ref = u_ref.replicate(N, 1);

    // Calculate H, G, E matrices
    H = 2 * (C.transpose() * Q_bar * C + R_bar);
    G = 2 * M.transpose() * Q_bar * C;
    E = 2 * (X_ref.transpose() * Q_bar * C + U_ref.transpose() * R_bar);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "mpc_policy_node");

    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("iris_0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("iris_0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("iris_0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    for(int i = 200; ros::ok() && i > 0; --i){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    // std::cout << "hello" << std::endl;
    MPC_avoidance mpc_avoidance(nh);

    ros::spin();

}