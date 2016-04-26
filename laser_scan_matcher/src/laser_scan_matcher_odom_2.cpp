#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Range.h>

ros::Publisher odom_pub;

double x_new = 0;
double y_new = 0;
double z_new = 0;
double th_new = 0;

double x_last = 0;
double y_last = 0;
double z_last = 0;
double th_last = 0;

ros::Time time_new;
ros::Time time_last;
ros::Time z_time_new;
ros::Time z_time_last;

geometry_msgs::Quaternion odom_quat;

int count = 0; //counter to skip the first pose estimate

//function to save the new height
void heightCallBack(const sensor_msgs::Range::ConstPtr& msg)
{
  z_new = msg->range;
  z_time_new = msg->header.stamp;
}

//function to save the new pose
void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  time_new = msg->header.stamp;
  x_new = msg->pose.position.x;
  y_new = msg->pose.position.y;
  th_new = tf2::getYaw(msg->pose.orientation);
  odom_quat = msg->pose.orientation;
}

//function to process the data
void processTransform()
{
  //create the transform broadcaster
  tf2_ros::TransformBroadcaster br;

  //create the messages
  nav_msgs::Odometry odom;
  geometry_msgs::TransformStamped transformStamped;

  //initialize the odom step displacement variables
  double x_last = 0.0;
  double y_last = 0.0;
  double z_last = 0.0;
  double th_last = 0.0;

  //calculates the time between messages being received
  float dt = (time_new - time_last).toSec();
  float z_dt = (z_time_new - z_time_last).toSec();

  //compute the change in displacement
  double delta_x = x_new - x_last;
  double delta_y = y_new - y_last;
  double delta_z = z_new - z_last;
  double delta_th = th_new - th_last;
    
  //compute the velocity
  double vx = delta_x / dt;
  double vy = delta_y / dt;
  double vz = delta_z / z_dt;
  double vth = delta_th / dt;

  if(z_dt == 0)
  {
    vz = 0;
  }

  //Populate the odom header
  odom.header.frame_id = "odom";
  odom.header.stamp = time_new;

  //Set the child frame id
  odom.child_frame_id = "base_link";

  //set the position
  odom.pose.pose.position.x = x_new;
  odom.pose.pose.position.y = y_new;
  odom.pose.pose.position.z = z_new;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.twist.twist.linear.x = vx; 
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = vz;
  odom.twist.twist.angular.z = vth;

  //set the covariance
  odom.pose.covariance[0] = 0.2; //change to 1e3 if this is still bad
  odom.pose.covariance[7] = 0.2; //change to 1e3 if this is still bad 
  odom.pose.covariance[14] = 1e100;
  odom.pose.covariance[21] = 1e100;
  odom.pose.covariance[28] = 1e100;
  odom.pose.covariance[35] = 0.2; //change to 1e3 if this is still bad

  //Populate the transform message
  transformStamped.header.stamp = time_new;
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = x_new;
  transformStamped.transform.translation.y = y_new;
  transformStamped.transform.translation.z = z_new;
  transformStamped.transform.rotation = odom_quat;

  if(count != 0 || dt != 0)
  {
    //publish the odom message
    odom_pub.publish(odom);
  
    //Publish the tranform
    br.sendTransform(transformStamped);
  }

  //save previous values
  time_last = time_new;
  z_time_last = z_time_new;
  x_last = x_new;
  y_last = y_new;
  z_last = z_new;
  th_last = th_new;

  count = 1;
}

int main(int argc, char** argv)
{
  //initialize ROS
  ros::init(argc, argv, "laser_scan_matcher_odom");
  ros::NodeHandle n;

  //create the odom publisher
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 60);

  //create the subscribers
  ros::Subscriber pose = n.subscribe("pose_stamped", 60, poseCallBack);
  ros::Subscriber height = n.subscribe("height", 60, heightCallBack);

  time_new = ros::Time::now();
  time_last = ros::Time::now();
  z_time_new = ros::Time::now();
  z_time_last = ros::Time::now();

  ros::Rate loop_rate(100);

  while(ros::ok()) 
  { 
    ros::spinOnce();
    loop_rate.sleep();
  }
    
  return 0;
}
