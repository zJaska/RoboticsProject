#include <ros/ros.h>
#include "RoboticsProject/WheelSpeed.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <sstream>

class Pub_sub_kinematic{
private:
    ros::NodeHandle n;

    geometry_msgs::TwistStamped kinematicGeometryMessage;

    ros::Subscriber sub;
    ros::Publisher pub;

    double l1;
    double l2;
    double vx, vy, wz;
    double radius, radiusAdj;
    double K[3][4];
    
public:
    Pub_sub_kinematic(){
        sub = n.subscribe("/wheel_speed", 1, &Pub_sub_kinematic::RigidBodyCalc, this);
        pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

        n.getParam("/Width", l1);
        n.getParam("/Length", l2);
        n.getParam("/WheelRad", radius);
        double Kcopy[3][4] = {{1,1,1,1}, {-1,1,1,-1},{-1/(l1+l2), 1/(l1+l2), -1/(l1+l2), 1/(l1+l2)}};

        std::copy(&Kcopy[0][0], &Kcopy[0][0] + 3 * 4, &K[0][0]); 
    }

    void RigidBodyCalc(const RoboticsProject::WheelSpeed &msg){

        radiusAdj = 2*M_PI*radius/(4*60);
        
        vx = (K[0][0]*msg.rpm_fl + K[0][1]*msg.rpm_fr + K[0][2]*msg.rpm_rl + K[0][3] * msg.rpm_rr)*radiusAdj;
        vy = (K[1][0]*msg.rpm_fl + K[1][1]*msg.rpm_fr + K[1][2]*msg.rpm_rl + K[1][3] * msg.rpm_rr)*radiusAdj;
        wz = (K[2][0]*msg.rpm_fl + K[2][1]*msg.rpm_fr + K[2][2]*msg.rpm_rl + K[2][3] * msg.rpm_rr)*radiusAdj;

        kinematicGeometryMessage.header.stamp = msg.header.stamp;

        kinematicGeometryMessage.twist.linear.x = vx;
        kinematicGeometryMessage.twist.linear.y = vy;
        kinematicGeometryMessage.twist.linear.z = 0;

        kinematicGeometryMessage.twist.angular.x = 0;
        kinematicGeometryMessage.twist.angular.y = 0;
        kinematicGeometryMessage.twist.angular.z = wz;

        //publish
        pub.publish(kinematicGeometryMessage);
    }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "KinematicCore");

  Pub_sub_kinematic pub_sub;

  ros::spin();

  return 0;
}
