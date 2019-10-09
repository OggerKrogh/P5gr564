#include "ros/ros.h"
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose curPos;
double curXpose;
double curYpose;

double pose(int flag);
void onPoseSet(double x, double y, double theta);
typedef boost::array<double, 36> array;
array Covariance = {0.0};

// CALLBACK FUNCTION FOR onPoseSet
void moveBaseCallback(const geometry_msgs::PoseWithCovarianceStamped msg){

curXpose = msg.pose.pose.position.x;
curYpose = msg.pose.pose.position.y;
//ROS_INFO("x: %f, y: %f",curXpose,curYpose);
}

//MOVETOPOINT FUNCTION
void moveToPoint(double x, double y, double rot)
{
  MoveBaseClient ac("move_base",true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Rate loop_rate(0.1);
  move_base_msgs::MoveBaseGoal start;


  start.target_pose.header.frame_id = "map";
  start.target_pose.header.stamp = ros::Time::now();
  start.target_pose.pose.position.x = x;
  start.target_pose.pose.position.y = y;
  start.target_pose.pose.orientation.z = 0.0;
  start.target_pose.pose.orientation.w = 1.0;


  ac.sendGoal(start);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the base moved to position");
}else{
ROS_INFO("The base failed to move");
  }
  ros::spinOnce();
  ROS_INFO("current pos x: %f, y: %f,",curXpose,curYpose);

  loop_rate.sleep();
}



//TELEPORT THE ROBOT ON INTERNAL MAP FUNCTION
void onPoseSet(double x, double y, double rot)
{

    ros::Rate loop_rate(0.5);

    ros::NodeHandle nh1;
    ros::Publisher pub_ = nh1.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);
    loop_rate.sleep();

    std::string fixed_frame = "map";
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();
    // set x,y coord
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0.0;
    //Convert rotation(radians) to Quaternion
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, rot);
        tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
    //Covariance matrix with 0.0
    pose.pose.covariance = Covariance;
    // publish
    ROS_INFO("x: %f, y: %f, z: 0.0 rot: %f",x,y, rot);
   while( x-0.1 > curXpose || curXpose > x+0.1 ||  y-0.1 > curYpose || curYpose > y+0.1 ){

    ROS_INFO("current pos x: %f, y: %f,",curXpose,curYpose);

    pub_.publish(pose);
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();
  }
    //ROS_INFO("I am okay sleep");


}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "simple_navigation_goals");
    ros::start();

    ros::NodeHandle nh2;
//    ros::Subscriber sub_ = nh2.subscribe<move_base_msgs::MoveBaseActionFeedback> ("/move_base/feedback", 1, moveBaseCallback);
    ros::Subscriber sub_ = nh2.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/amcl_pose", 10, moveBaseCallback);


     while(ros::ok()){
        ROS_INFO("1");
        moveToPoint(6.8, 10.45, 1.0);
        ROS_INFO("2");
        moveToPoint(6.0, 10.45, 1.0);
        ROS_INFO("teleport");
        onPoseSet(24.25, 14.6, 0.0);
        ROS_INFO("4");
        moveToPoint(24.3, 12.95, 1.0);
        ROS_INFO("3");
        moveToPoint(24.25, 14.6, 1.0);
        ROS_INFO("teleport");
        onPoseSet(24.25, 14.6, 0.0);
        onPoseSet(6.0, 10.45, 0.0);

        ROS_INFO("1");
        moveToPoint(6.8, 10.45, 1.0);
        //moveToPoint(7.85, 14.3, 1.0); old point 1

      }
  ROS_INFO("???");
   return 0;
 }
