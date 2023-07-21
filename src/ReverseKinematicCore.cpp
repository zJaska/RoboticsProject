#include <ros/ros.h>
#include "RoboticsProject/WheelSpeed.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <sstream>

class Pub_sub_Revkinematic{
private:
    ros::NodeHandle n;

    RoboticsProject::WheelSpeed wheelSpeedMsg;

    ros::Subscriber sub;
    ros::Publisher pub;

    double l1;
    double l2;
    double vx, vy, wz;
    double radius, radiusAdj;
    
public:
    Pub_sub_Revkinematic(){
        sub = n.subscribe("/cmd_vel", 1, &Pub_sub_Revkinematic::RigidBodyInvCalc, this);
        pub = n.advertise<RoboticsProject::WheelSpeed>("/wheels_rpm", 1);

        n.getParam("/Width", l1);
        n.getParam("/Length", l2);
        n.getParam("/WheelRad", radius);
        
        
    }

    void RigidBodyInvCalc(const geometry_msgs::TwistStamped::ConstPtr &msg){

        ros::Time currentTime;

        double Krev[4][3] = {{1,-1,-(l1+l2)}, {1,1,(l1+l2)}, {1,1,-(l1+l2)}, {1,-1,(l1+l2)}};

        double coeff = (60)/(radius*2*M_PI);

        vx = msg->twist.linear.x;
        vy = msg->twist.linear.y;
        wz = msg->twist.angular.z;

        //set header
        wheelSpeedMsg.header.stamp = currentTime;
        //wheelSpeedMsg.header.frame_id = "wspeed";

        //set velocity
        wheelSpeedMsg.rpm_fl = (Krev[0][0]*vx + Krev[0][1]*vy + Krev[0][2]*wz)*coeff;
        wheelSpeedMsg.rpm_fr = (Krev[1][0]*vx + Krev[1][1]*vy + Krev[1][2]*wz)*coeff;
        wheelSpeedMsg.rpm_rl = (Krev[2][0]*vx + Krev[2][1]*vy + Krev[2][2]*wz)*coeff;
        wheelSpeedMsg.rpm_rr = (Krev[3][0]*vx + Krev[3][1]*vy + Krev[3][2]*wz)*coeff;

        pub.publish(wheelSpeedMsg);

    }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "RevKinematicCore");

  Pub_sub_Revkinematic pub_sub;

  ros::spin();

  return 0;
}
