#ifndef NIRYO_TEST_MOTORS_H
#define NIRYO_TEST_MOTORS_H


#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>

#include <urdf/model.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
// #include <moveit/move_group_interface/move_group_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "niryo_one_msgs/SetBool.h"
#include "niryo_one_msgs/SetInt.h"

#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>


typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class NiryoOneTestMotor {

    private:
        ros::NodeHandle nh;
        ros::ServiceClient calibrate_motor_client;
        ros::Subscriber joint_state_subscriber;

        ros::Publisher reset_stepper_publisher;

        TrajClient* traj_client_;

        std::vector<double> pose_start{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        bool enable_test;
        int _n_joints = 6;
        std::vector<std::string>  _joint_names;
        std::vector<double>  _joint_upper_limits;
        std::vector<double>  _joint_lower_limits;
        std::vector<double>  _joint_has_position_limits;

        std::vector<double> _current_joint_pose;


    public:
        NiryoOneTestMotor();

        void callbackJointSate(const sensor_msgs::JointState& msg);
        
        bool getJointsLimits();

        bool runTest(int nb_loops);
        void stopTest();
        void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
        bool playTrajectory(control_msgs::FollowJointTrajectoryGoal goal);

        control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(std::vector<double> joint_positions);
        actionlib::SimpleClientGoalState getState();

};
#endif
