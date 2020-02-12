#include "std_msgs/String.h"
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pub_goal");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1) ;

  ros::Rate loop_rate(1);

  int count = 0;
  double xVar=0.5;

  while (ros::ok())
  {

   geometry_msgs::PoseStamped goal;

   goal.header.frame_id="map";
   goal.pose.position.x=xVar;
   goal.pose.orientation.w=1;
   goal.pose.orientation.z=0;

   ROS_INFO("%lf",goal.pose.position.x);

   pub.publish(goal);

   ros::spinOnce();

   ros::Duration d(5);
   loop_rate.sleep();
   xVar=xVar+0.05;

   }


  return 0;
}