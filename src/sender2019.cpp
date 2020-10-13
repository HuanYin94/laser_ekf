#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <fstream>
#include <vector>
#include <time.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "tf/transform_listener.h"
#include "geometry_msgs/Pose2D.h"

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/point_cloud.h"
#include "nabo/nabo.h"
#include "pointmatcher_ros/get_params_from_server.h"


#define PORT    8844
#define MAXLINE 1024
#define BYTENUM 30 // 16 + 4*3 + 2
#define SLEEPSEC 50000 //20Hz // 10e6 = 1s

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;

struct header
{
    unsigned short int flag;
    unsigned short int type;
    unsigned short int ID;
    unsigned short int length;
    unsigned int sec;
    unsigned int miliSec;
};

struct udpMsg
{
    struct header Header;
    float pose_x;
    float pose_y;
    float heading;
    unsigned short int status;
};

int sockfd;
char buffer[MAXLINE];
struct sockaddr_in servaddr;

unsigned short int idCnt = 0x0000;

Vector3f RT3D2Pose2D(PM::TransformationParameters RT)
{
    Vector3f pose(RT(0,3), RT(1,3), std::atan2(RT(1,0), RT(0,0)));
    return pose;
}

void senderCallback(PM::TransformationParameters T_base2world)
{

    // 4x4 RT Matrix to pose-2D
    Vector3f pose = RT3D2Pose2D(T_base2world);

    /// udp, sth.
    udpMsg toVeh;

    toVeh.Header.flag = 0xcece;
    toVeh.Header.type = 0x0001;
    toVeh.Header.ID = idCnt;
    toVeh.Header.length = 16;
    toVeh.Header.sec = ros::Time::now().toSec();
    toVeh.Header.miliSec = ros::Time::now().toNSec();

    toVeh.pose_x = pose(0);
    toVeh.pose_y = pose(1);
    toVeh.heading = pose(2);
    toVeh.status = 1;

    memcpy(buffer, &toVeh, sizeof(toVeh));

    sendto(sockfd, (const char *)buffer, BYTENUM,
        MSG_CONFIRM, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));

    cout<<"-----------------------------------------------"<<endl;

    cout<<toVeh.Header.ID <<"  "<<
          toVeh.pose_x<<"  "<<
          toVeh.pose_y<<"  "<<
          toVeh.heading<<endl;

    idCnt = idCnt + 1;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sender");
    ros::NodeHandle n;

    // tf & transform

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
//    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_addr.s_addr = inet_addr("192.168.1.12");


    PM::TransformationParameters T_base2world;
    tf::TransformListener listener;
    ros::Rate rate(20.0);
    while(n.ok())
    {
        try
        {
            T_base2world = PointMatcher_ros::eigenMatrixToDim<float>(
                       PointMatcher_ros::transformListenerToEigenMatrix<float>(
                       listener,
                       "world",
                       "base_footprint",
                       ros::Time(0)
                   ), 4);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("transfrom exception : %s",ex.what());
        }
        senderCallback(T_base2world);

        rate.sleep();

    }


//    ros::Subscriber sub = n.subscribe("veh_sta_udp", 1, senderCallback);

//    ros::spin();

    return 0;
}
