//
// Created by clarence on 18-10-24.
//

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

#define MAX_PLANAR_SPEED 2.0 // m/s
#define MAX_YAW_RATE 0.35 // rad/s, 0.35 at most constrained by the MC itself in simulation
#define MAX_Z_SPEED 1.0 // m/s
#define DEAD_ZONE 0.05
#define PI 3.141592653

double axes_planar_speed = 0.0;
double axes_yaw_rate = 0.0;
double axes_z_speed = 0.0;

Eigen::Vector3d current_pos;
double current_yaw;

void joyCallback(const sensor_msgs::Joy &msg)
{
    if(fabs(msg.axes[4]) > DEAD_ZONE)
    {
        axes_planar_speed = msg.axes[4] * MAX_PLANAR_SPEED;
    }
    else axes_planar_speed = 0.0;

    if(fabs(msg.axes[1]) > DEAD_ZONE)
    {
        axes_z_speed = msg.axes[1] * MAX_Z_SPEED;
    }
    else axes_z_speed = 0.0;

    if(fabs(msg.axes[5]) > DEAD_ZONE && fabs(msg.axes[2]) > DEAD_ZONE)
    {
        axes_yaw_rate = (msg.axes[5] - msg.axes[2]) / 2.0 * MAX_YAW_RATE;
    }
    else axes_yaw_rate = 0.0;
}


void odomCallback(const nav_msgs::Odometry &msg)
{
    current_pos(0) = msg.pose.pose.position.x;
    current_pos(1) = msg.pose.pose.position.y;
    current_pos(2) = msg.pose.pose.position.z;

    Eigen::Quaterniond att;
    att.x() = msg.pose.pose.orientation.x;
    att.y() = msg.pose.pose.orientation.y;
    att.z() = msg.pose.pose.orientation.z;
    att.w() = msg.pose.pose.orientation.w;

    current_yaw = atan2(2*att.y()*att.x() - 2*att.z()*att.w(), -2*att.y()*att.y() - 2*att.w()*att.w() +1 ) + PI;
    //ROS_INFO_STREAM("current_yaw" << current_yaw);
}

void paras_init()
{
    current_yaw = 0.0;
    current_pos(0) = 0.0;
    current_pos(1) = 0.0;
    current_pos(2) = 0.0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_velocity");

    paras_init();

    ros::NodeHandle nh;

    ros::Subscriber joy_sub_ = nh.subscribe("/joy", 10, joyCallback);
    ros::Subscriber odom_sub = nh.subscribe("/firefly/ground_truth/odometry", 2, odomCallback);

    ros::Publisher trajectory_pub =
            nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                    mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    ROS_INFO("Started joy velocity control.");

    // Start gazebo first
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
    while (i <= 10 && !unpaused) {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    if (!unpaused) {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    } else {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(5.0).sleep();

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    // Default desired position and yaw.
    Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
    double desired_yaw = 0.0;

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
            desired_position, desired_yaw, &trajectory_msg);

    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
             nh.getNamespace().c_str(), desired_position.x(),
             desired_position.y(), desired_position.z());
    trajectory_pub.publish(trajectory_msg);

    ros::Duration(2.0).sleep();

    // Control Loop
    int _frequency = 30;
    ros::Rate loop_rate(_frequency);

    double planar_scale = 1.0 / _frequency;
    double z_scale = 1.0 / _frequency;
    double yaw_scale = 1.0 / _frequency;

    double max_error_x = 2.0;
    double max_error_y = 2.0;
    double max_error_z = 2.0;
    double max_error_yaw = 1.0; // rad

    ROS_WARN_STREAM("Joy velocity control ready!");
    while(ros::ok())
    {
        // Generate position setpoint according to reference velocity
        double delt_l = axes_planar_speed * planar_scale;
        double delt_x = delt_l * cos(current_yaw);
        double delt_y = delt_l * sin(current_yaw);

        double delt_z = axes_z_speed * z_scale;
        double delt_yaw = axes_yaw_rate * yaw_scale;

        // Give a limitation according to the current pose
        if(fabs(desired_position.x() - current_pos(0)) < max_error_x)
        {
            // Update
            desired_position.x() += delt_x;
        }

        if(fabs(desired_position.y() - current_pos(1)) < max_error_y)
        {
            desired_position.y() += delt_y;
        }

        if(fabs(desired_position.z() - current_pos(2)) < max_error_z)
        {
            desired_position.z() += delt_z;
        }

        if(fabs(desired_yaw - current_yaw) < 0.5 || fabs(desired_yaw - current_yaw) > 5.8)
        {
            desired_yaw += delt_yaw;
        }

        // Correct yaw to [0, 2*PI]
        if(desired_yaw < 0) desired_yaw += 2*PI;
        else if(desired_yaw > 2*PI) desired_yaw -= 2*PI;


        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                desired_position, desired_yaw, &trajectory_msg);
        trajectory_pub.publish(trajectory_msg);

//        ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f, %f, %f].",
//                 nh.getNamespace().c_str(), axes_yaw_rate, desired_position.x(),
//                 desired_position.y(), desired_position.z(), desired_yaw);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();


    return 0;
}
