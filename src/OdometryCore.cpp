#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <dynamic_reconfigure/server.h>
#include "RoboticsProject/parametersConfig.h"
#include "RoboticsProject/reset.h"
#include "RoboticsProject/reset_general.h"




class Pub_sub_odometry_core{

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;
    ros::ServiceServer resetZeroService;
    ros::ServiceServer resetGeneralService;

    ros::Time lastTime;
    double x,y,th;
    

    int integrationType;
    bool firstIteration;

public:
    Pub_sub_odometry_core() {
        sub = n.subscribe("/cmd_vel", 1, &Pub_sub_odometry_core::computeOdometry, this);
        odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
        firstIteration = true;

        resetZeroService = n.advertiseService("reset_zero" , &Pub_sub_odometry_core::resetZero, this);
        resetGeneralService = n.advertiseService("reset_general" , &Pub_sub_odometry_core::resetGeneral, this);

        x = 0;
        y = 0;
        th = 0;
        integrationType = 0;
    }

    void computeOdometry(const geometry_msgs::TwistStamped::ConstPtr &msg){
        
        ros::Time currentTime = msg->header.stamp;

        if(!firstIteration){
            double vx = msg->twist.linear.x;
            double vy = msg->twist.linear.y;
            double vth = msg->twist.angular.z;
            
            double dt = (currentTime - lastTime).toSec();
        
            if(integrationType){//runge kutta
                x += (vx * cos(th +  vth * dt / 2) - vy * sin(th +  vth * dt / 2)) * dt;
                y += (vx * sin(th +  vth * dt / 2) + vy * cos(th +  vth * dt / 2)) * dt;
                th += vth * dt;
            }else{//euler
                x += (vx * cos(th) - vy * sin(th)) * dt;
                y += (vx * sin(th) + vy * cos(th)) * dt;
                th += vth * dt;
            }
            

            nav_msgs::Odometry odometry;

            geometry_msgs::TransformStamped odometryTransformation;

            tf2::Quaternion q;
            q.setRPY(0, 0, th);


            odometry.header.stamp = currentTime;
            odometry.header.frame_id = "odom";
            odometry.pose.pose.position.x = x;
            odometry.pose.pose.position.y = y;
            odometry.pose.pose.position.z = 0;

            odometry.pose.pose.orientation.x = q.x();
            odometry.pose.pose.orientation.y = q.y();
            odometry.pose.pose.orientation.z = q.z();
            odometry.pose.pose.orientation.w = q.w();

            odometry.child_frame_id = "baseLink";

            odometry.twist.twist.linear.x = vx;
            odometry.twist.twist.linear.y = vy;
            odometry.twist.twist.linear.z = 0;
            
            odometry.twist.twist.angular.z = vth;


            odometryTransformation.header.stamp = currentTime;
            odometryTransformation.header.frame_id = "odom";
            odometryTransformation.child_frame_id = "base_link";
            odometryTransformation.transform.translation.x = x;
            odometryTransformation.transform.translation.y = y;
            odometryTransformation.transform.translation.z = 0;

            odometryTransformation.transform.rotation.x = q.x();
            odometryTransformation.transform.rotation.y = q.y();
            odometryTransformation.transform.rotation.z = q.z();
            odometryTransformation.transform.rotation.w = q.w();

            odom_broadcaster.sendTransform(odometryTransformation);
            odom_pub.publish(odometry);
        }
        lastTime = currentTime;
        firstIteration = false;
    }

    void setIntegration(RoboticsProject::parametersConfig &config){
        integrationType = config.integration;
    }

    bool resetZero(RoboticsProject::reset::Request  &req,
                   RoboticsProject::reset::Response &res)
    {
        firstIteration = true;
        x = 0;
        y = 0;
        th = 0;
        ROS_INFO("x reset to: %f", x);
        ROS_INFO("y reset to: %f", y);
        return true;
    }

    bool resetGeneral(RoboticsProject::reset_general::Request  &req,
                      RoboticsProject::reset_general::Response &res)
    {
        firstIteration = true;
        x = req.x;
        y = req.y;
        th = req.th;
        ROS_INFO("x reset to: %f", x);
        ROS_INFO("y reset to: %f", y);
        ROS_INFO("th reset to: %f", th);
        return true;
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "OdomentryCore");
    Pub_sub_odometry_core pubSubOdometry;

    dynamic_reconfigure::Server<RoboticsProject::parametersConfig> server;
    dynamic_reconfigure::Server<RoboticsProject::parametersConfig>::CallbackType f;

    f = boost::bind(&Pub_sub_odometry_core::setIntegration, &pubSubOdometry, _1);
    server.setCallback(f);

    ros::spin();
    return 0;
}

