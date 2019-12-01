#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"

#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/transform.h"

#include <fstream>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

class ekf
{
    typedef PointMatcher<float> PM;

public:
    ekf(ros::NodeHandle &n);
    ~ekf();
    ros::NodeHandle& n;

    ros::Subscriber mag_pose_sub;

    void gotMagPose(const geometry_msgs::PointStamped& msgIn);
    void goTbaseOdom(const nav_msgs::Odometry& msgIn);

    nav_msgs::Odometry odo_msg_timing;

    ros::Subscriber laserOdomSub;

    tf::TransformBroadcaster tfBroader;
    tf::TransformListener tfListener;

    // poses & odoms
    geometry_msgs::PointStamped mag_pose;
    Vector3f mag_measured;  // vector of the measurement of the magnetic

    nav_msgs::Odometry laser_odom;

    float mag_dis_th;
    string savePoseFileName;
    string saveMagFileName;

    bool mag_init_flag = true;
    bool loc_init_flag = true;
    bool measure_flag = false;

    PM::TransformationParameters TlastbaseOdom;
    PM::TransformationParameters TbaseOdom;

    PM::TransformationParameters TMapToWorld;
    PM::TransformationParameters TBaseToMap;
    PM::TransformationParameters TBaseToWorld;

    // tools
    float calc2Ddistance(geometry_msgs::PointStamped lastMag, geometry_msgs::PointStamped nowMag);
    Vector3f get2DTransform(PM::TransformationParameters input);
    float to_degrees(float radians);

    //finals
    Vector3f vehicle_pose;
    Vector3f vehicle_pose_predict;

    Matrix3f conv_sigma;
    Matrix3f conv_sigma_predict;

    Matrix3f noise_R;
    Matrix3f noise_Q;
    Matrix3f Jacob_F;
    Matrix3f Jacob_H;
    Matrix3f Gain_K;
    Matrix3f I;

    void sysInit();
    Matrix3f getMotionR(float lastOrient);
    Matrix3f getJacob_F(float lastOrient, float selfMotion_x, float selfMotion_y);

    float angleNorm(float head);

    bool isPublishTF;
    void publishTF(ros::Time pubTime);

    bool isSaveOut;
    ofstream outSavePose;
    ofstream outSaveMag;

    Eigen::Vector3f RT3D2Pose2D(PM::TransformationParameters RT);
    PM::TransformationParameters Pose2DToRT3D(Vector3f input);
    Eigen::Vector3f RT2D2Pose2D(Matrix3f RT);
    Eigen::Matrix3f Pose2D2RT2D(Vector3f pose);

    ros::Publisher vis_pub;
    void publishMarker(Vector3f input, int cnt);
    int marker_cnt;

};

ekf::~ekf()
{}

ekf::ekf(ros::NodeHandle& n):
    n(n),
    mag_dis_th(getParam<float>("mag_dis_th", 5.0)),
    isPublishTF(getParam<bool>("isPublishTF", true)),
    saveMagFileName(getParam<string>("saveMagFileName", ".")),
    savePoseFileName(getParam<string>("savePoseFileName", ".")),
    isSaveOut(getParam<bool>("isSaveOut", false))
{
    // init 000
    this->sysInit();

    marker_cnt = 0;

    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

//    mag_pose_sub = n.subscribe("mag_pose", 1, &ekf::gotMagPose, this);

    laserOdomSub = n.subscribe("laser_odom_2d", 1, &ekf::goTbaseOdom, this);

}

void ekf::gotMagPose(const geometry_msgs::PointStamped& msgIn)
{

//    cout<<"Mag pose:    "<<msgIn.point.x
//       <<"  "<<msgIn.point.y<<"  "<<msgIn.point.z<<endl;

    if(mag_init_flag)
    {
        this->mag_pose = msgIn;

        mag_measured << mag_pose.point.x, mag_pose.point.y, this->angleNorm(mag_pose.point.z);

        mag_init_flag = false;
    }
    else
    {
        // last and current
        float dis = this->calc2Ddistance(this->mag_pose, msgIn);

        if(dis >= mag_dis_th)
        {
            this->mag_pose = msgIn;

            // repeat for vector define
            mag_measured << mag_pose.point.x, mag_pose.point.y, this->angleNorm(mag_pose.point.z);

            // if false, always odom
            measure_flag = true;
//            measure_flag = false;
        }
    }

    // save the gournd truth
    if(isSaveOut)
    {
        // save mag pose
        outSaveMag.open(saveMagFileName, ios::out|ios::ate|ios::app);
        outSaveMag << msgIn.header.stamp <<"    "
                   << msgIn.point.x << "    "
                << msgIn.point.y << "   "
                << msgIn.point.z <<endl;
        outSaveMag.close();
    }

}

void ekf::goTbaseOdom(const nav_msgs::Odometry& odomMsgIn)
{
    // just record the time
    odo_msg_timing = odomMsgIn;

    this->TbaseOdom = PointMatcher_ros::odomMsgToEigenMatrix<float>(odomMsgIn);

    // process here
    cout<<"---------------------------------------------------"<<endl;

    // initial need mag inf
    if(loc_init_flag && !mag_init_flag)
    {
        this->sysInit();
        return;
    }

    /// Predict Motion and Noise
    /// motion, from robot_base to world
    PM::TransformationParameters TOdomRelative = TlastbaseOdom.inverse() * TbaseOdom;
    TlastbaseOdom = TbaseOdom;

    Vector3f self_motion = this->get2DTransform(TOdomRelative);
    Matrix3f R_theta = this->getMotionR(vehicle_pose(2));
    Vector3f world_motion = R_theta.inverse() * self_motion;

    cout<<"self motion: "<<self_motion.transpose()<<endl;
    cout<<"motion:  "<<world_motion.transpose()<<endl;

    /// in World Coordinate
    vehicle_pose_predict = vehicle_pose + world_motion;
    Jacob_F = this->getJacob_F(vehicle_pose(2), self_motion(0), self_motion(1));
    conv_sigma_predict = Jacob_F*conv_sigma*Jacob_F.transpose() + this->noise_R;

//    cout<<"Noise:"<<endl;
//    cout<<conv_sigma_predict<<endl;

    // Gain
    if(measure_flag)
    {
        cout<<"!!!!!!!!........Measurement got........@@@@@@@@"<<endl;

        cout<<"Mag Pose:    "<<mag_measured.transpose()<<endl;

        this->publishMarker(mag_measured, marker_cnt); marker_cnt++;

        // Gain
        Matrix3f K_ = Jacob_H * conv_sigma_predict * Jacob_H.transpose() + noise_Q;

        Gain_K = conv_sigma_predict * Jacob_H.transpose() * K_.inverse();

        vehicle_pose = vehicle_pose_predict + Gain_K * (mag_measured - Jacob_H * vehicle_pose_predict);

        conv_sigma = (I - Gain_K*Jacob_H) * conv_sigma_predict;

        measure_flag = false;

        cout<<"Conv predict:   "<<endl;
        cout<<conv_sigma_predict<<endl;

        cout<<"Gain:    "<<endl;
        cout<<Gain_K<<endl;

        cout<<"Conv update: "<<endl;
        cout<<conv_sigma<<endl;

    }
    else
    {
        vehicle_pose = vehicle_pose_predict;
        conv_sigma = conv_sigma_predict;

        measure_flag = false; // extra
    }

    this->publishTF(odomMsgIn.header.stamp);

    cout<<"vehicle pose:    " <<vehicle_pose.transpose()<<endl;

    float d_yaw = this->to_degrees(world_motion(2));

    if(isSaveOut)
    {
        // save estimated pose
        outSavePose.open(savePoseFileName, ios::out|ios::ate|ios::app);
        outSavePose << odomMsgIn.header.stamp <<"   "
                    << vehicle_pose(0) << " "
                << vehicle_pose(1) << " "
                << vehicle_pose(2) <<endl;
        outSavePose.close();
    }

}

void ekf::sysInit()
{
//    cout<<"Init mag Pose:    "<<mag_pose.point<<endl;

    cout<<"System Initial!"<<endl;

//    vehicle_pose << mag_pose.point.x, mag_pose.point.y, this->angleNorm(mag_pose.point.z);
    vehicle_pose << 0,0,0;

    // vis pub
    this->publishMarker(mag_measured, marker_cnt); marker_cnt++;

    this->TMapToWorld = this->Pose2DToRT3D(mag_measured);

    cout<<TMapToWorld<<endl;

    TlastbaseOdom = TbaseOdom = PM::TransformationParameters::Identity(4, 4);

    conv_sigma << 0*Matrix3f::Identity(); // 3x3
    noise_R = 0.1*Matrix3f::Identity();  // 3x3

    Jacob_H = 1.0*Matrix3f::Identity();   // jacob for measure is constant in this case
    noise_Q = 0.001*Matrix3f::Identity();

    I = Matrix3f::Identity();

    loc_init_flag = false;
}

void ekf::publishTF(ros::Time pubTime)
{
    cout<<"publish!!!"<<endl;

    // T_basetomap
    TBaseToWorld = this->Pose2DToRT3D(vehicle_pose);
    TBaseToMap = TMapToWorld.inverse() * TBaseToWorld;

    tfBroader.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TBaseToMap, "map", "base_footprint", pubTime));
    tfBroader.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TMapToWorld, "world", "map", pubTime));

}

/// ???
float ekf::angleNorm(float head)
{
    // 0 ~ 360 ?
    while(head > 2*M_PI)
        head = head - 2*M_PI;
    while(head < 0)
        head = head + 2*M_PI;

    return head;
}

Matrix3f ekf::getMotionR(float lastOrient)
{
    Matrix3f R;
    R << cos(lastOrient), sin(lastOrient), 0,
          -sin(lastOrient), cos(lastOrient), 0,
          0, 0, 1;
    return R;
}

Matrix3f ekf::getJacob_F(float lastOrient, float selfMotion_x, float selfMotion_y)
{
    Matrix3f F;
    F << 1, 0, -sin(lastOrient)*selfMotion_x-cos(lastOrient)*selfMotion_y,
         0, 1, cos(lastOrient)*selfMotion_x-sin(lastOrient)*selfMotion_y,
         0, 0, 1;
    return F;
}

float ekf::calc2Ddistance(geometry_msgs::PointStamped lastMag, geometry_msgs::PointStamped nowMag)
{
    float delta_x = (lastMag.point.x - nowMag.point.x);
    float delta_y = (lastMag.point.y - nowMag.point.y);

    return std::sqrt(delta_x*delta_x + delta_y*delta_y);
}

Vector3f ekf::get2DTransform(PM::TransformationParameters input)
{
    Vector3f output;

    output(0) = input(0,3);
    output(1) = input(1,3);

    float delta_yaw = atan2(input(1,0), input(0,0));
    cout<<"d_yaw(deg):     "<<this->to_degrees(delta_yaw)<<endl;

    output(2) = delta_yaw;

    return output;
}

float ekf::to_degrees(float radians) {
    return radians / M_PI * 180.0;
}

/// copy from locVeh.cpp

Vector3f ekf::RT3D2Pose2D(PM::TransformationParameters RT)
{
    Vector3f pose(RT(0,3), RT(1,3), std::atan2(RT(1,0), RT(0,0)));
    return pose;
}

ekf::PM::TransformationParameters ekf::Pose2DToRT3D(Vector3f input)
{
    PM::TransformationParameters t = PM::TransformationParameters::Identity(4, 4);
    t(0,3) = input(0); t(1,3) = input(1);
    AngleAxisf V1(input(2), Vector3f(0, 0, 1));
    Matrix3f R1 = V1.toRotationMatrix();
    t.block(0,0,3,3) = R1;
    return t;
}

Vector3f ekf::RT2D2Pose2D(Matrix3f RT)
{
    Vector3f pose(RT(0,2), RT(1,2), std::atan2(RT(1,0), RT(0,0)));
    return pose;
}

Matrix3f ekf::Pose2D2RT2D(Vector3f pose)
{
    Matrix3f RT;
    RT << cos(pose(2)), -sin(pose(2)), pose(0),
          sin(pose(2)), cos(pose(2)),pose(1),
          0, 0, 1;
    return RT;
}

void ekf::publishMarker(Vector3f input, int cnt)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "sphere";
    marker.id = cnt;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = input(0);
    marker.pose.position.y = input(1);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
//        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );
}

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n;

    ekf ekf_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
