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

    // poses & quats(wheels)
    geometry_msgs::Vector3Stamped velocity_angular;

    void gotVelocityAngular(const geometry_msgs::Vector3Stamped& msgIn);

    // do math
    float to_radians(float degrees);
    void process_pub_odom();

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
    velocity_angular_sub = n.subscribe("velocity_angular", 1, &odom::gotVelocityAngular, this);

}

void odom::gotVelocityAngular(const geometry_msgs::Vector3Stamped& msgIn)
{
//    cout<<"I heard the wheel angs."<<endl;
    this->velocity_angular = msgIn;
    cout<<msgIn.header.seq<<endl;
    // process&publish the odometry data
    this->process_pub_odom();
}

void odom::process_pub_odom()
{

    cout<<"-----------------------------------------------------"<<endl;

    float Vx, Vy, angularV;
    Vx = velocity_angular.vector.x;
    Vy = velocity_angular.vector.y;
    angularV = velocity_angular.vector.z;

    float x_new, y_new, theta_new;

    x_new = cos(veh_sta(2))*DELTA_TIME*Vx - sin(veh_sta(2))*DELTA_TIME*Vy + veh_sta(0);
    y_new = sin(veh_sta(2))*DELTA_TIME*Vx + cos(veh_sta(2))*DELTA_TIME*Vy + veh_sta(1);
    theta_new = veh_sta(2) + angularV*DELTA_TIME;

    // re-new
    veh_sta(0) = x_new; veh_sta(1) = y_new; veh_sta(2) = theta_new;

//    Vector3f self_motion;

//    self_motion(0) = v_average*DELTA_TIME*cos(veh_sta(2));
//    self_motion(1) = v_average*DELTA_TIME*sin(veh_sta(2));
//    self_motion(2) = a_average*DELTA_TIME;

//    Matrix3f R_theta = this->getMotionR(this->veh_sta(2));
//    Vector3f world_motion = R_theta.inverse() * self_motion;

//    // add
//    veh_sta = veh_sta + world_motion;

    cout<<"odom state:   "<<veh_sta(0)<<"  "<<veh_sta(1)<<"  "<<veh_sta(2)<<endl;

    AngleAxisf V1(veh_sta(2), Vector3f(0, 0, 1));
    Matrix3f R1 = V1.toRotationMatrix();


//    float dx = v_average * DELTA_TIME;
//    float d_theta = a_average*DELTA_TIME;

//    /// from matlab 2D traj file
//    // delta value
//    // delete and ignore

//    AngleAxisf V1(d_theta, Vector3f(0, 0, 1));
//    Matrix3f R1 = V1.toRotationMatrix();

//    Matrix4f T_r;
//    T_r << R1(0,0), R1(0,1), R1(0,2),  dx,
//         R1(1,0), R1(1,1), R1(1,2),  0,
//         R1(2,0), R1(2,1), R1(2,2), 0 ,
//         0, 0, 0, 1;

//    T_veh = T_veh_last*T_r;
//    T_veh_last = T_veh;

//    cout<<"T_veh:   "<<endl;
//    cout<<T_veh<<endl;

//    tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T_veh, "map", "wheel", wheel_angs.header.stamp));

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

    odo_msg.twist.twist.linear.x = velocity_angular.vector.x;
    odo_msg.twist.twist.linear.y = velocity_angular.vector.y;
    odo_msg.twist.twist.angular.x = velocity_angular.vector.z;

    /// set zero for test
//    odo_msg.twist.twist.linear.x = 0;
//    odo_msg.twist.twist.linear.y = 0;
//    odo_msg.twist.twist.angular.x = 0;


    odo_msg.header.stamp = velocity_angular.header.stamp;
    odo_msg.header.seq = velocity_angular.header.seq;
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
