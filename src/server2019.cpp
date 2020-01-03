#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/publisher.h"

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

#define PORT    9999
#define MAXLINE 1024
//#define BYTENUM 60 // 16 for header only
#define BYTENUM 40

using namespace std;

struct header
{
    unsigned short int flag;
    unsigned short int type;
    unsigned short int ID;
    unsigned short int length;
    unsigned int sec;
    unsigned int miliSec;
};

struct msg
{
    struct header Header;
    float mag_pose_x;
    float mag_pose_y;
    float mag_heading;

    float speed_agv_vx;
    float speed_agv_vy;
    float angular_agv_gyro;
};

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "Server");
    ros::NodeHandle n;

    ros::Publisher mag_pose_pub = n.advertise<geometry_msgs::PointStamped>("mag_pose", 1);
    ros::Publisher velocity_angular_pub = n.advertise<geometry_msgs::Vector3Stamped>("velocity_angular", 1);

    // SOCKET
    int sockfd;
    char buffer[MAXLINE];
    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,
            sizeof(servaddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }


    // MSG
    msg dataFromVeh;
    socklen_t len;
    int status;

    // quaternion for just storing
    geometry_msgs::PointStamped mag_pose;
    geometry_msgs::Vector3Stamped velocity_angular_data;

    mag_pose.header.frame_id = "world";
    velocity_angular_data.header.frame_id = "vehicle";

    // loop closing
    while(n.ok())
    {
        status = recvfrom(sockfd, (char *)buffer, MAXLINE,
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);

        memcpy(&dataFromVeh, buffer, sizeof(dataFromVeh));

        // no print out, it's in record cpp
        cout<<"---------------------"<<endl;
        printf("HEADER ID:      %d\n", dataFromVeh.Header.ID);

        ros::Time timeStamp = ros::Time::now();

        // publish the pose message using pointXYZ
        mag_pose.header.stamp = timeStamp;
        mag_pose.point.x = dataFromVeh.mag_pose_x;
        mag_pose.point.y = dataFromVeh.mag_pose_y;
        mag_pose.point.z = dataFromVeh.mag_heading;
        mag_pose_pub.publish(mag_pose);

        velocity_angular_data.header.stamp = timeStamp;
        velocity_angular_data.vector.x = dataFromVeh.speed_agv_vx;
        velocity_angular_data.vector.y = dataFromVeh.speed_agv_vy;
        velocity_angular_data.vector.z = dataFromVeh.angular_agv_gyro;

        velocity_angular_pub.publish(velocity_angular_data);

    }

    return 0;
}
