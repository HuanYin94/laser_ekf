#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "pointmatcher_ros/transform.h"
#include <fstream>

#define DELTA_TIME 0.02

using namespace std;
using namespace Eigen;

class odom
{

public:
    odom(ros::NodeHandle &n);
    ~odom();
    ros::NodeHandle& n;

    ros::Subscriber velocity_angular_sub;

    // do math
    float to_radians(float degrees);
    void process_odom();

    // odometry msg
    long int cnt = 0;
    nav_msgs::Odometry odo_msg;

    ros::Publisher odomPub;

    Matrix4f T_veh_last = Matrix4f::Identity();
    Matrix4f T_veh;
    tf::TransformBroadcaster tfBroadcaster;

    Vector3f veh_sta;
    Matrix3f matrixFromAngle(float x, float y, float z);
    Matrix3f getMotionR(float lastOrient);

};

odom::~odom()
{}

odom::odom(ros::NodeHandle& n):
    n(n)
{
    // init veh
    // ???
    this->veh_sta << 0, 0, 0;

    odomPub = n.advertise<nav_msgs::Odometry>("wheel_odom", 1, true);

    cout<<"------------------------------"<<endl;
//    velocity_angular_sub = n.subscribe("velocity_angular", 1, &odom::gotVelocityAngular, this);

    while(ros::ok())
    {
        process_odom();
        ros::Duration(DELTA_TIME).sleep();
    }

}

void odom::process_odom()
{

    cout<<"-----------------------------------------------------"<<endl;

    float Vx, Vy, angularV;
    Vx = 0;
    Vy = 0;
    angularV = 0;

    float x_new, y_new, theta_new;

    x_new = cos(veh_sta(2))*DELTA_TIME*Vx - sin(veh_sta(2))*DELTA_TIME*Vy + veh_sta(0);
    y_new = sin(veh_sta(2))*DELTA_TIME*Vx + cos(veh_sta(2))*DELTA_TIME*Vy + veh_sta(1);
    theta_new = veh_sta(2) + angularV*DELTA_TIME;

    // re-new
    veh_sta(0) = x_new; veh_sta(1) = y_new; veh_sta(2) = theta_new;

    cout<<"odom state:   "<<veh_sta(0)<<"  "<<veh_sta(1)<<"  "<<veh_sta(2)<<endl;

    AngleAxisf V1(veh_sta(2), Vector3f(0, 0, 1));
    Matrix3f R1 = V1.toRotationMatrix();

    // get the quatern for odom publish
    Matrix3f veh_rot;
    veh_rot << R1(0,0), R1(0,1), R1(0,2),
               R1(1,0), R1(1,1), R1(1,2),
               R1(2,0), R1(2,1), R1(2,2);
    Quaternionf Q1(veh_rot);

    odo_msg.pose.pose.position.x = veh_sta(0);
    odo_msg.pose.pose.position.y = veh_sta(1);
    odo_msg.pose.pose.position.z = 0;

    odo_msg.pose.pose.orientation.x = Q1.x();
    odo_msg.pose.pose.orientation.y = Q1.y();
    odo_msg.pose.pose.orientation.z = Q1.z();
    odo_msg.pose.pose.orientation.w = Q1.w();

    odo_msg.twist.twist.linear.x = Vx;
    odo_msg.twist.twist.linear.y = Vy;
    odo_msg.twist.twist.angular.x = angularV;

    /// set zero for test
//    odo_msg.twist.twist.linear.x = 0;
//    odo_msg.twist.twist.linear.y = 0;
//    odo_msg.twist.twist.angular.x = 0;


    odo_msg.header.stamp = ros::Time::now();
    odo_msg.header.seq = 999;
    odo_msg.header.frame_id = "world";
    odo_msg.child_frame_id = "base_footprint";

    odomPub.publish(odo_msg);

}



Matrix3f odom::matrixFromAngle(float x, float y, float z)
{
    Matrix3f R1;
    R1 = AngleAxisf(x, Vector3f::UnitX())
      * AngleAxisf(y,  Vector3f::UnitY())
      * AngleAxisf(z, Vector3f::UnitZ());

//    R1 << cos(z), sin(z), 0,
//          -sin(z),cos(z), 0,
//          0, 0, 1;

    return R1;
}

float odom::to_radians(float degrees) {
    return degrees * (M_PI / 180.0);
}

Matrix3f odom::getMotionR(float lastOrient)
{
    Matrix3f R;
    R << cos(lastOrient), sin(lastOrient), 0,
          -sin(lastOrient), cos(lastOrient), 0,
          0, 0, 1;
    return R;
}





int main(int argc, char **argv)
{

    // INIT
    /// new odom code in 2019.05.31
    ros::init(argc, argv, "odom_may");
    ros::NodeHandle n;

    odom odom_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
