#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>

geometry_msgs::TransformStamped transformStamped;

//function to save the new height
void heightCallBack(const sensor_msgs::Range::ConstPtr& msg)
{
  transformStamped.transform.translation.z = msg->range;
}

//function to save the new pose
void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  transformStamped.header.stamp = msg->header.stamp;
  transformStamped.transform.translation.x = msg->pose.position.x;
  transformStamped.transform.translation.y = msg->pose.position.y;
  transformStamped.transform.rotation = msg->pose.orientation;
}

int main(int argc, char** argv)
{
  //initialize ROS
  ros::init(argc, argv, "laser_scan_matcher_odom");
  ros::NodeHandle n;

  //create the subscribers
  ros::Subscriber pose = n.subscribe("pose_stamped", 5, poseCallBack);
  ros::Subscriber height = n.subscribe("height", 5, heightCallBack);

  tf::TransformBroadcaster br;

  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";

  ros::Rate loop_rate(40);

  while(ros::ok()) 
  { 
    ros::spinOnce();
    br.sendTransform(transformStamped);
    loop_rate.sleep();
  }
    
  return 0;
}
