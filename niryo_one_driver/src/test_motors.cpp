#include "niryo_one_driver/test_motors.h"


NiryoOneTestMotor::NiryoOneTestMotor()
{
    getJointsLimits();

    reset_stepper_publisher = nh.advertise<std_msgs::Empty>("/niryo_one/steppers_reset_controller", 1000);
    calibrate_motor_client = nh.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/calibrate_motors");

    traj_client_ = new TrajClient("/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory", true);
    joint_state_subscriber = nh.subscribe("/joint_states",10, &NiryoOneTestMotor::callbackJointSate, this);

    ROS_INFO("Test motors up");
}

void NiryoOneTestMotor::callbackJointSate(const sensor_msgs::JointState& msg)
{
    _current_joint_pose.resize(_n_joints);
    for (size_t i = 0; i < _n_joints; ++i)
    {
        _current_joint_pose[i] = msg.position[i];
    }
}

bool NiryoOneTestMotor::runTest(int nb_loops)
{
    enable_test = true;
    
    ROS_INFO("Reset Controller");
    std_msgs::Empty reset_controller_topic;
    reset_stepper_publisher.publish(reset_controller_topic);
    ros::Duration(0.05).sleep();

    bool state;
    state = playTrajectory(armExtensionTrajectory(pose_start));
    if (!state) {return false;}

    for (int j=0; j<nb_loops; j++)
    {
        for (size_t i = 0; i < _n_joints; ++i)
        {
            if (_joint_has_position_limits[i])
            {
                std::vector<double> pose_lower_limit = pose_start;
                pose_lower_limit[i] = _joint_lower_limits[i];
                state = playTrajectory(armExtensionTrajectory(pose_lower_limit));
                if (!state) {return false;}

                state = playTrajectory(armExtensionTrajectory(pose_start));
                if (!state) {return false;}

                std::vector<double> pose_upper_limit = pose_start;
                pose_upper_limit[i] = _joint_upper_limits[i];
                state = playTrajectory(armExtensionTrajectory(pose_upper_limit));
                if (!state) {return false;}

                state = playTrajectory(armExtensionTrajectory(pose_start));
                if (!state) {return false;}
            }
        }
    }
    enable_test = false;

    return true;
}

void NiryoOneTestMotor::stopTest()
{
    if (enable_test)
    {    
        enable_test = false;
        ROS_INFO("Reset Controller");
        std_msgs::Empty reset_controller_topic;
        reset_stepper_publisher.publish(reset_controller_topic);
    }
    return;
}

bool NiryoOneTestMotor::getJointsLimits()
{
    // Resize vectors
    _joint_names.resize(_n_joints);
    _joint_upper_limits.resize(_n_joints);
    _joint_lower_limits.resize(_n_joints);
    _joint_has_position_limits.resize(_n_joints);

    // Get limits from URDF
    urdf::ModelSharedPtr urdf(new urdf::Model);
    std::string urdf_str;
    if (nh.getParam("robot_description", urdf_str)) // Check for robot_description in proper namespace
    {
        if (!urdf->initString(urdf_str))
        {
            ROS_ERROR_STREAM("Failed to parse URDF contained in 'robot_description' parameter (namespace: " <<
            nh.getNamespace() << ").");
            return false;
        }
    }
    else if (!urdf->initParam("robot_description")) // Check for robot_description in root
    {
        ROS_ERROR_STREAM("Failed to parse URDF contained in 'robot_description' parameter");
        return false;
    }
    for (auto i = 0; i < _n_joints; i++)
    {
        // Joints name
        _joint_names[i]= "joint_" + std::to_string(i+1);

        urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(_joint_names[i]);
        if (urdf_joint)
        {
            if (urdf_joint->type != urdf::Joint::CONTINUOUS)
            {
                _joint_upper_limits[i] = urdf_joint->limits->upper - 0.2;
                _joint_lower_limits[i] = urdf_joint->limits->lower + 0.2;
                _joint_has_position_limits[i] = true;
            }
            else
                _joint_has_position_limits[i] = false;
        }
        else
        {
            return false;
        }
    }
}

void NiryoOneTestMotor::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
    ROS_INFO("Send trajectory");
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);

}

bool NiryoOneTestMotor::playTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
    if (!enable_test)
    {
        return true;
    }
    
    startTrajectory(goal);
    
    std::vector<double> rounded_target;
    rounded_target.resize(_n_joints);

    for (size_t i = 0; i < _n_joints; ++i)
    {
        rounded_target[i] = round(goal.trajectory.points[goal.trajectory.points.size()-1].positions[i]*10)/10;
    }
    // Wait for trajectory completion
    while(!getState().isDone() && ros::ok() && enable_test)
    {
        sleep(0.1);
    }

    sleep(0.3);
    for (size_t i = 0; i < _n_joints; ++i)
    {
        double goal_joint = goal.trajectory.points[goal.trajectory.points.size()-1].positions[i];
        if (enable_test && (( _current_joint_pose[i] < goal_joint-0.1) || (_current_joint_pose[i] > goal_joint+0.1)))
        {
            return false;
        }
        
    }

    return true;
}

control_msgs::FollowJointTrajectoryGoal NiryoOneTestMotor::armExtensionTrajectory(std::vector<double> joint_positions)
{
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    int ind = 0;
    goal.trajectory.joint_names.resize(_n_joints);
    goal.trajectory.points[ind].positions.resize(_n_joints);
    goal.trajectory.points[ind].velocities.resize(_n_joints);

    for (size_t j = 0; j < _n_joints; ++j)
    {
        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names[j] = _joint_names[j];
        // Positions
        goal.trajectory.points[ind].positions[j] = joint_positions[j];
        // Velocities
        goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    return goal;
}

actionlib::SimpleClientGoalState NiryoOneTestMotor::getState()
{
    return traj_client_->getState();
}
