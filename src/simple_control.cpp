#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>

#define VEL_HZ 3
#define ANGL_ERR 0.1
#define DIST_ERR 0.1
#define MAX_ANGL_VEL 2
#define MIN_ANGL_VEL 0.5
#define MAX_LIN_VEL 3
#define MIN_LIN_VEL 0.2
#define ANGL_P 0.5
ros::Subscriber goalSub;
ros::Subscriber poseSub;
ros::Publisher velocityPub;

bool goalReached = true;
geometry_msgs::Pose goalPose;
geometry_msgs::Pose robotPose;

double angleFromQuaterion(geometry_msgs::Quaternion &msg)
{
    return tf::getYaw(msg);
}

double errorAngle(){
    double robotAngle = angleFromQuaterion(robotPose.orientation);
    double dx = goalPose.position.x - robotPose.position.x;
    double dy = goalPose.position.y - robotPose.position.y;
    // double dz = goalPose.position.z - robotPose.position.z;
    double dz = 0.0;
    
    // double goalAngle = std::acos(dx/std::sqrt(dx*dx+dy*dy+dz*dz));
    double goalAngle = std::atan2(dy,dx);
    ROS_INFO("Our goal angle: %lf",goalAngle);
    double diff = goalAngle - robotAngle;
    if (diff>M_PI)
    {
        diff = M_PI*2-diff;
    }
    else if (diff<-M_PI)
    {
        diff = diff + M_PI*2;
    }
    
    return diff;
}

void callback_goal(geometry_msgs::PoseStamped::ConstPtr msg)
{
    goalPose.orientation = msg->pose.orientation;
    goalPose.position = msg->pose.position;
    goalReached = false;
    ROS_INFO("goal x: %lf; y: %lf; angle: %lf",goalPose.position.x,goalPose.position.y, errorAngle());
}

void callback_robot_pose(geometry_msgs::PoseStamped msg)
{
    robotPose.orientation = msg.pose.orientation;
    robotPose.position = msg.pose.position;
    // ROS_INFO("%lf",angleFromQuaterion(robotPose.orientation));
}


void configure(ros::NodeHandle* n, std::string goalTopic, std::string positionTopic, std::string velocityTopic)
{   
    goalSub = n->subscribe(goalTopic,1,callback_goal);
    poseSub = n->subscribe(positionTopic,1,callback_robot_pose);
    velocityPub = n->advertise<geometry_msgs::Twist>(velocityTopic,1);
    ROS_INFO("Simple Control node is configured");
}



double distError(){
    double dx = goalPose.position.x - robotPose.position.x;
    double dy = goalPose.position.y - robotPose.position.y;
    double dz = goalPose.position.z - robotPose.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

geometry_msgs::Twist adjust()
{
    geometry_msgs::Twist vel;
    ROS_INFO("current position x: %lf; y: %lf; angle: %lf",robotPose.position.x, robotPose.position.y,angleFromQuaterion(robotPose.orientation));
    double error_angle = errorAngle();
    double error_dist = distError();
    ROS_INFO("Goal distance %lf; error angle: %lf",error_dist,error_angle);
    if (error_dist<DIST_ERR)
    {
        goalReached = true;
    }
    else if (abs(error_angle)>ANGL_ERR){
        vel.angular.z = error_angle*ANGL_P;
        if (abs(vel.angular.z) > MAX_ANGL_VEL)
        {
            if (vel.angular.z > 0){
                vel.angular.z = MAX_ANGL_VEL;
            }
            else{
                vel.angular.z = - MAX_ANGL_VEL;
            }
        }
        else if (abs(vel.angular.z) < MIN_ANGL_VEL && vel.angular.z <= 0){
            vel.angular.z = - MIN_ANGL_VEL;
        }
        else if (abs(vel.angular.z) < MIN_ANGL_VEL && vel.angular.z > 0){
            vel.angular.z = MIN_ANGL_VEL;
        }
    }
    else if (abs(error_dist)>DIST_ERR)
    {
        vel.linear.x = 1.5;
    }
    ROS_INFO("twist at z axis: %lf",vel.angular.z);
    return vel;
}






int main(int argc, char **argv)
{
    ros::init(argc,argv,"simple_control");
    ros::NodeHandle n;
    ros::Duration(2.9).sleep();
    configure(&n,"/goal","/slam_out_pose", "/r2d2_diff_drive_controller/cmd_vel");
    ros::Rate r(3);
    while (ros::ok())
    {
        if (!goalReached){
            geometry_msgs::Twist msg = adjust();
            if (goalReached){
                ROS_INFO("GOAL IS REACHED HOORAY!!!!");
            }
            velocityPub.publish(msg);
        }
        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
}