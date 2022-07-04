#include<ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

#define MAX_SPEED 10.0f

float distance=100.0f;

void callback_laser(sensor_msgs::LaserScan msg)
{
    distance = msg.ranges[msg.ranges.size()/2];
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"r2d2_test");
    ros::NodeHandle n;
    ros::Subscriber r2d2_laser = n.subscribe("/scan",1,callback_laser);
    ros::Publisher r2d2_control = n.advertise<geometry_msgs::Twist>("/r2d2_diff_drive_controller/cmd_vel",50);
    ros::Rate rate(50);
    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = (distance-3>=0)? distance-3: 0;
        r2d2_control.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}