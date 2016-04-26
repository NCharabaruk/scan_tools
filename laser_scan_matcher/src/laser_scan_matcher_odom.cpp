#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Range.h>

double x_new = 0;
double y_new = 0;
double z_new = 0;
double th_new = 0;
ros::Time time_new;
ros::Time z_time_new;

//function to save the new height
void HeightCallBack(const sensor_msgs::Range::ConstPtr& msg)
{
  z_new = msg->range;
  z_time_new=msg->header.stamp;
}

//function to save the new pose
void PoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  time_new = msg->header.stamp;
  x_new = msg->pose.position.x;
  y_new = msg->pose.position.y;
  th_new = tf::getYaw(msg->pose.orientation);
}

int main(int argc, char** argv)
{
  //initialize ROS
  ros::init(argc, argv, "laser_scan_matcher_odom");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 60);
  ros::Subscriber pose = n.subscribe("pose_stamped", 60, PoseCallBack);
  ros::Subscriber height = n.subscribe("height", 60, HeightCallBack);
  
  //initialize step displacement variables
  double x_last = 0.0;
  double y_last = 0.0;
  double z_last = 0.0;
  double th_last = 0.0;
  
  ros::Time time_last = ros::Time::now();
  ros::Time z_time_last = ros::Time::now();
  
  //counter to skip the first run
  int count = 0;

  //populate the constant parts of the odom message
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  //set the covariance
  odom.pose.covariance[0] = 0.2; //change to 1e3 if this is still bad
  odom.pose.covariance[7] = 0.2; //change to 1e3 if this is still bad 
  odom.pose.covariance[14] = 1e100;
  odom.pose.covariance[21] = 1e100;
  odom.pose.covariance[28] = 1e100;
  odom.pose.covariance[35] = 0.2; //change to 1e3 if this is still bad

  ros::Rate loop_rate(100);

  while(ros::ok()) 
  { 
    //calculates the time between messages being sent
    float dt = (time_new - time_last).toSec();
    float z_dt = (z_time_new - z_time_last).toSec();

    //skips the rest of the loop if for some reason no time has passed between encoder counts
    if(dt == 0 || count == 0)
    {
    }
    else
    {
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

      ROS_INFO_STREAM("z_new: " << z_new << ", z_last: " << z_last << ", delta_z: " << delta_z << ", z_dt: " << z_dt << ", vz: " << vz);

      //create quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_last);

      //set the time
      odom.header.stamp = time_new;

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

      //publish the message
      odom_pub.publish(odom);

      //
    }
    
    //save previous values
    time_last = time_new;
    z_time_last = z_time_new;
    x_last = x_new;
    y_last = y_new;
    z_last = z_new;
    th_last = th_new;
    
    count = 1;
    
    ros::spinOnce();
    loop_rate.sleep();
  }
    
  return 0;
}
