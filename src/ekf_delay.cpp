#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include <fstream>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <visualization_msgs/Marker.h>

#define DELTA_TIME 0.02
#define VEH_WID  2.394

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

class loc
{
    typedef PointMatcher<float> PM;

public:
    loc(ros::NodeHandle &n);
    ~loc();
    ros::NodeHandle& n;

    ros::Subscriber wheel_odom_sub;
    ros::Subscriber laser_odom_sub;
    ros::Subscriber mag_pose_sub;

    geometry_msgs::Pose2D veh_sta_udp;
    ros::Publisher veh_sta_pub;

    nav_msgs::Odometry wheel_odom_msg;
    nav_msgs::Odometry laser_odom_msg;
    geometry_msgs::PointStamped mag_pose_msg;


    void gotMagPose(const geometry_msgs::PointStamped& msgIn);
    void gotLaserOdom(const nav_msgs::Odometry& msgIn);
    void gotWheelOdom(const nav_msgs::Odometry& msgIn);

    float mag_dis_th;
    bool initFinsh_flag;
//    bool procFinsh_flag;  // lock the process
//    int process_type;

    Eigen::VectorXf veh_sta;
    Eigen::MatrixXf conv;
    Eigen::MatrixXf jacob_H;
    Eigen::Matrix3f noise_Q_mag;
    Eigen::Matrix3f noise_Q_laser;
    Eigen::MatrixXf gain_K;
    Eigen::MatrixXf A;

    float angleNorm(float head);

    PM::TransformationParameters TlastbaseOdom;
    PM::TransformationParameters TbaseOdom;

    Eigen::Vector3f RT3D2Pose2D(PM::TransformationParameters RT);
    PM::TransformationParameters Pose2DToRT3D(Vector3f input);
    Eigen::Vector3f RT2D2Pose2D(Matrix3f RT);
    Eigen::Matrix3f Pose2D2RT2D(Vector3f pose);

    Eigen::Vector3f mag_measured;
    Eigen::Vector3f mag_measured_last;

    float calc2Ddistance(Vector3f a, Vector3f b);

//    tf::TransformListener tfListener;
    tf::TransformBroadcaster tfBroader;

    void publishTF(ros::Time pubTime);
    string savePoseName;
    string saveMagName;
    ofstream outSavePose;
    ofstream outSaveMag;
    bool isSave;

    PM::TransformationParameters TMapToWorld;
    PM::TransformationParameters TBaseToMap;
    PM::TransformationParameters TBaseToWorld;

    double lastTime, nowTime;

    ros::Publisher vis_pub;
    void publishMarker(Vector3f input, int cnt);
    int marker_cnt;

    float laserodom_dis_th;
    bool is_use_laser;

    float kw;
};

loc::~loc()
{}

loc::loc(ros::NodeHandle& n):
    n(n),
//    kw(getParam<float>("kw", 5.0)),
    mag_dis_th(getParam<float>("mag_dis_th", 5.0)),
    laserodom_dis_th(getParam<float>("laserodom_dis_th", 0.1)),
    savePoseName(getParam<string>("savePoseName", ".")),
    saveMagName(getParam<string>("saveMagName", ".")),
    isSave(getParam<bool>("isSave", false)),
    is_use_laser(getParam<bool>("is_use_laser", false))
{
    // init some values
    initFinsh_flag = false;
//    procFinsh_flag = true;
//    process_type = 0;
    kw = 1;

//    /// TEMP INIT
//    {
        initFinsh_flag = true;
        veh_sta(0) = 0; veh_sta(1) = 0; veh_sta(2) = 0;
//    }

    /// #TODO: to merege the noises
    cout<<"what?"<<endl;

    /// turn
    noise_Q_mag = 0.0000001*Matrix3f::Identity(); noise_Q_mag(2,2) = noise_Q_mag(2,2)*0.25;
    noise_Q_laser = 0.0000001*Matrix3f::Identity(); noise_Q_laser(2,2) = noise_Q_laser(2,2)*0.25;

    /// straight
//    noise_Q_mag = 0.0001*Matrix3f::Identity(); noise_Q_mag(2,2) = noise_Q_mag(2,2)*0.25;
//    noise_Q_laser = 10*Matrix3f::Identity(); noise_Q_laser(2,2) = noise_Q_laser(2,2)*0.25;

    conv = 0.0001*Matrix3f::Identity();
    jacob_H.resize(3, 6); // in two obs
    A.resize(6,3);A.topLeftCorner(3,3)=Matrix3f::Identity();A.bottomLeftCorner(3,3)=Matrix3f::Identity();

    //
    TMapToWorld = PM::TransformationParameters::Identity(4, 4);
    TBaseToMap = PM::TransformationParameters::Identity(4, 4);
    TBaseToWorld = PM::TransformationParameters::Identity(4, 4);

    // pubers
    marker_cnt = 0;
    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );

    veh_sta_pub = n.advertise<geometry_msgs::Pose2D>("veh_sta_udp", 1);

    // motion: wheel
    wheel_odom_sub = n.subscribe("wheel_odom", 1, &loc::gotWheelOdom, this);

    // observation1: laser odom
    if(is_use_laser)
    {
        laser_odom_sub = n.subscribe("laser_odom_2d", 1, &loc::gotLaserOdom, this); // >
    }


    /// TEMP NOTED
    // observation2: mag nails
//    mag_pose_sub = n.subscribe("mag_pose", 1, &loc::gotMagPose, this);

}


void loc::gotWheelOdom(const nav_msgs::Odometry &msgIn)
{

    wheel_odom_msg = msgIn;

    if(!initFinsh_flag)
    {
        cout<<"Waiting for System Initial."<<endl;
        return;
    }
    else
    {
        cout<<"----------------@@@@@-----Odome----------"<<endl;
//        cout<<"Time:    "<<msgIn.header.stamp<<endl;
//        double start = ros::Time::now().toSec();

        wheel_odom_msg = msgIn;

        nowTime = msgIn.header.stamp.toSec();

        float delta_s = wheel_odom_msg.twist.twist.linear.x * (nowTime - lastTime);
        float delta_theta = wheel_odom_msg.twist.twist.angular.x * (nowTime - lastTime);

        lastTime = nowTime;

        Vector3f delta;
        delta << delta_s*cos(veh_sta(2)+delta_theta/2),
                 delta_s*sin(veh_sta(2)+delta_theta/2),
                 delta_theta;

        Matrix2f delta_conv = Matrix2f::Zero(2,2);
        delta_conv(0,0) = 0.5 * kw * std::abs(delta_s);
        delta_conv(1,1) = 2 * 0.5 / VEH_WID / VEH_WID * kw * abs(delta_s);

        Matrix3f Jacob_x = Matrix3f::Identity(3,3);
        Jacob_x(0,2) = -delta_s * sin(veh_sta(2) + 0.5*delta_theta);
        Jacob_x(1,2) = delta_s * cos(veh_sta(2) + 0.5*delta_theta);

        MatrixXf Jacob_u(3,2);
        Jacob_u << cos(veh_sta(2)+0.5*delta_theta), -delta_s*sin(veh_sta(2)+0.5*delta_theta)*0.5,
                   sin(veh_sta(2)+0.5*delta_theta), delta_s*cos(veh_sta(2)+0.5*delta_theta)*0.5,
                   0, 1;

//        cout<<"Delta:"<<endl;
//        cout<<delta<<endl;

        Vector3f new_veh_sta_head = veh_sta.head(3) + delta;
        veh_sta(0) = new_veh_sta_head(0);
        veh_sta(1) = new_veh_sta_head(1);
        veh_sta(2) = new_veh_sta_head(2);
        veh_sta(2) = this->angleNorm(veh_sta(2));

//        cout<<"Jacob_x"<<endl;
//        cout<<Jacob_x<<endl;
//        cout<<"Jacob_u"<<endl;
//        cout<<Jacob_u<<endl;

        if(veh_sta.size() == 6)
        {
            MatrixXf AA(6,6);
            AA.topLeftCorner(3,3) = Jacob_x;
            AA.topRightCorner(3,3) = Matrix3f::Zero(3,3);
            AA.bottomLeftCorner(3,3) = Matrix3f::Zero(3,3);
            AA.bottomRightCorner(3,3) = Matrix3f::Identity(3,3);

            MatrixXf BB(6,2);
            BB.topLeftCorner(3,2) = Jacob_u;
            BB.bottomLeftCorner(3,2) = MatrixXf::Zero(3,2);

//            cout<<"AA:"<<endl;
//            cout<<AA<<endl;
//            cout<<"BB:"<<endl;
//            cout<<BB<<endl;

            conv = AA * conv * AA.transpose() + BB * delta_conv * BB.transpose();
        }
        else // 3
        {
            conv = Jacob_x * conv * Jacob_x.transpose() + Jacob_u * delta_conv * Jacob_u.transpose();
        }

//        cout<<"conv"<<endl;
//        cout<<conv<<endl;

//        double end = ros::Time::now().toSec();
//        cout<<"Time Cost:   "<< end - start <<endl;

    }

    cout<<"veh sta:   "<<veh_sta.transpose()<<endl;
    this->publishTF(msgIn.header.stamp);

}

void loc::gotLaserOdom(const nav_msgs::Odometry &msgIn)
{
    laser_odom_msg = msgIn;

    if(!initFinsh_flag)
    {
        cout<<"Waiting for System Initial."<<endl;
        return;
    }
    else
    {
        cout<<"----------------@@@@@-----Laser----------"<<endl;

        if(veh_sta.size() == 3)  // 3, remark this moment as the last
        {
            TlastbaseOdom = PointMatcher_ros::odomMsgToEigenMatrix<float>(msgIn);

            //dimens from 3 to 6
            veh_sta = A*veh_sta;

            conv = A*conv*A.transpose();
        }
        else // 6
        {
            TbaseOdom = PointMatcher_ros::odomMsgToEigenMatrix<float>(msgIn);
            PM::TransformationParameters TOdomRelative = TlastbaseOdom.inverse() * TbaseOdom;

            Vector3f laserOdomMeasure = this->RT3D2Pose2D(TOdomRelative);

            Vector3f a(0,0,0);

            float laser_odom_dis = calc2Ddistance(laserOdomMeasure,a) ;

            cout<<"Measure: "<<endl;
            cout<<laserOdomMeasure.transpose()<<endl;

            if(laser_odom_dis > laserodom_dis_th)
            {

                Matrix3f lastRT = this->Pose2D2RT2D(veh_sta.tail(3));
                Matrix3f currRT = this->Pose2D2RT2D(veh_sta.head(3));
                Matrix3f laserOdomRT = lastRT.inverse() * currRT;

                Vector3f laserOdomSelf = this->RT2D2Pose2D(laserOdomRT);

                // trick
//                laserOdomSelf(0) = laserOdomSelf(0) + 0.15;

//                Vector3f laserOdomSelf = veh_sta.tail(3) - veh_sta.head(3);

                cout<<"self:    "<<endl;
                cout<<laserOdomSelf.transpose()<<endl;

                // from wy's file
                MatrixXf matrix_S(2,2);
                matrix_S << -lastRT(1,0), lastRT(0,0),
                            -lastRT(0,0), -lastRT(1,0);

                Vector2f vector_O;
                Vector2f lastT(lastRT(0,2), lastRT(1,2));
                Vector2f currT(currRT(0,2), currRT(1,2));
                vector_O = matrix_S * (currT - lastT);

                jacob_H << lastRT(0,0), lastRT(1,0), 0, -lastRT(0,0), -lastRT(1,0), vector_O(0),
                           lastRT(0,1), lastRT(1,1), 0, -lastRT(0,1), -lastRT(1,1), vector_O(1),
                           0 , 0, 1, 0, 0, -1;

                matrix_S.resize(3,3);
//                matrix_S = jacob_H * conv * jacob_H.transpose() +  std::pow(laser_odom_dis*0.01,2)*noise_Q_laser;
                matrix_S = jacob_H * conv * jacob_H.transpose() +  noise_Q_laser;

                gain_K = conv * jacob_H.transpose() * matrix_S.inverse();

    //            cout<<"Gain_K"<<endl;
    //            cout<<std::fixed<<gain_K<<endl;
    //            cout<<"laserOdomMeasure"<<endl;
    //            cout<<std::fixed<<laserOdomMeasure.transpose()<<endl;
    //            cout<<"laserOdomSelf"<<endl;
    //            cout<<std::fixed<<laserOdomSelf.transpose()<<endl;

                veh_sta = veh_sta + gain_K*(laserOdomMeasure - laserOdomSelf);

                veh_sta(2) = this->angleNorm(veh_sta(2));
                veh_sta(5) = this->angleNorm(veh_sta(5));

                MatrixXf I = MatrixXf::Identity(6,6);
                conv = (I - gain_K*jacob_H)*conv;

                //
                TlastbaseOdom = TbaseOdom;
                Vector3f veh_sta_front = veh_sta.head(3); // top 3
                veh_sta = A * veh_sta_front;
                Matrix3f conv_head = conv.topLeftCorner(3,3);
                conv = A * conv_head * A.transpose();
            }
            else
            {
                TlastbaseOdom = TbaseOdom; // !
            }


        }

    }

    cout<<"veh sta:   "<<veh_sta.transpose()<<endl;
    this->publishTF(msgIn.header.stamp);

}

void loc::gotMagPose(const geometry_msgs::PointStamped &msgIn)
{
    mag_measured << msgIn.point.x,
                    msgIn.point.y,
                    this->angleNorm(msgIn.point.z);

    // is Save ?
    if(isSave)
    {
        outSaveMag.open(saveMagName, ios::out|ios::ate|ios::app);
        outSaveMag << msgIn.header.stamp <<"    "
                   << mag_measured(0) << "    "
                << mag_measured(1) << "   "
                << mag_measured(2) <<endl;
        outSaveMag.close();
    }

    if(!initFinsh_flag)
    {
        cout<<"... Initial ..."<<endl;
        cout<<mag_measured.transpose()<<endl;

        veh_sta = mag_measured;
        // const & init
        this->TMapToWorld = this->Pose2DToRT3D(mag_measured);

        mag_measured_last = mag_measured;

        lastTime = msgIn.header.stamp.toSec();

        initFinsh_flag = true;

        this->publishMarker(mag_measured, marker_cnt); marker_cnt++;

        return;
    }
    else
    {

        float dis = this->calc2Ddistance(mag_measured_last, mag_measured);
//        cout<<"!!!!!!!!  DIS:   "<<dis<<endl;
        if(dis < mag_dis_th)
        {
            return;
        }

        cout<<"----------------@@@@@-----Magne----------"<<endl;
        cout<<"----------------!!!!!!!!!!!!!!!----------"<<endl;
        cout<<"----------------???????????????----------"<<endl;

        this->publishMarker(mag_measured, marker_cnt); marker_cnt++;

        double start = ros::Time::now().toSec();

        /// sta = 6 when next mag msg comes in, laser odom mush
//        if(veh_sta.size() == 3)
//            jacob_H = Matrix3f::Identity();
//        else
//        {
            jacob_H.topLeftCorner(3,3) = Matrix3f::Identity(3,3);jacob_H.topRightCorner(3,3) = Matrix3f::Zero(3,3);
//        }

        MatrixXf S = jacob_H*conv*jacob_H.transpose() + noise_Q_mag;

        gain_K = conv*jacob_H.transpose()*S.inverse();

        Vector3f veh_sta_front = veh_sta.head(3); // top 3
        this->veh_sta = veh_sta + gain_K*(mag_measured - veh_sta_front);
        veh_sta(2) = this->angleNorm(veh_sta(2));
        veh_sta(5) = this->angleNorm(veh_sta(5));

        MatrixXf I = MatrixXf::Identity(6,6);
        conv = (I - gain_K*jacob_H)*conv;

        // dis filter
        mag_measured_last = mag_measured;

    }

    cout<<"veh sta:   "<<veh_sta.transpose()<<endl;
    this->publishTF(msgIn.header.stamp);

}

float loc::angleNorm(float head)
{
    // 0 ~ 360 ?
    while(head > 2*M_PI)
        head = head - 2*M_PI;
    while(head < 0)
        head = head + 2*M_PI;

    return head;
}

Vector3f loc::RT3D2Pose2D(PM::TransformationParameters RT)
{
    Vector3f pose(RT(0,3), RT(1,3), std::atan2(RT(1,0), RT(0,0)));
    return pose;
}

loc::PM::TransformationParameters loc::Pose2DToRT3D(Vector3f input)
{
    PM::TransformationParameters t = PM::TransformationParameters::Identity(4, 4);
    t(0,3) = input(0); t(1,3) = input(1);
    AngleAxisf V1(input(2), Vector3f(0, 0, 1));
    Matrix3f R1 = V1.toRotationMatrix();
    t.block(0,0,3,3) = R1;
    return t;
}

Vector3f loc::RT2D2Pose2D(Matrix3f RT)
{
    Vector3f pose(RT(0,2), RT(1,2), std::atan2(RT(1,0), RT(0,0)));
    return pose;
}

Matrix3f loc::Pose2D2RT2D(Vector3f pose)
{
    Matrix3f RT;
    RT << cos(pose(2)), -sin(pose(2)), pose(0),
          sin(pose(2)), cos(pose(2)),pose(1),
          0, 0, 1;
    return RT;
}

float loc::calc2Ddistance(Vector3f a, Vector3f b)
{
    float delta_x = (a(0) - b(0));
    float delta_y = (a(1) - b(1));

    return std::sqrt(delta_x*delta_x + delta_y*delta_y);
}

void loc::publishTF(ros::Time pubTime)
{
    // if listened?
    if(!initFinsh_flag)
        return;

    /// publish status for udp connection
    veh_sta_udp.x = veh_sta(0);
    veh_sta_udp.y = veh_sta(1);
    veh_sta_udp.theta = veh_sta(2);
    veh_sta_pub.publish(veh_sta_udp);

    /// publish tf for visualization in Rviz

    // T_basetomap
    TBaseToWorld = this->Pose2DToRT3D(veh_sta);
    TBaseToMap = TMapToWorld.inverse() * TBaseToWorld;

    tfBroader.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TBaseToMap, "map", "base_footprint", pubTime));
    tfBroader.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TMapToWorld, "world", "map", pubTime));

    // is Save ?
    if(isSave)
    {
        outSavePose.open(savePoseName, ios::out|ios::ate|ios::app);
        outSavePose << pubTime <<"    "
                   << veh_sta(0) << "    "
                << veh_sta(1)<< "   "
                << veh_sta(2) <<endl;
        outSavePose.close();
    }
}

void loc::publishMarker(Vector3f input, int cnt)
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
    ros::init(argc, argv, "ekf_delay");
    ros::NodeHandle n;

    loc loc_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

}
