#include <ros/ros.h>
#include <tum_ics_ur10_controller_tutorial/EETarget.h>
#include <ow_core/types.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ee_publisher");

  ros::NodeHandle nh;
  ros::Publisher ee_target_pub = nh.advertise<tum_ics_ur10_controller_tutorial::EETarget>("ee_target", 1);

  ow::CartesianPosition X_goal_w{0.5, 0.2, 0.5, 0., 0.7071, -0.7071, 0.}; // w, x, y, z

  tum_ics_ur10_controller_tutorial::EETarget msg;
  msg.ee_target = X_goal_w;
  msg.duration = 3.0;
  ee_target_pub.publish(msg);
  

  ros::Rate loop_rate(10);

  ros::Time t_start = ros::Time::now();

  u_int pub_cnt = 0;

  while (ros::ok())
  {
    if ((ros::Time::now() - t_start).toSec() > 1 && pub_cnt == 0)
    {
      ROS_INFO_STREAM("Publishing EE target 1");
      msg.ee_target = X_goal_w;
      msg.duration = 0.5;
      ee_target_pub.publish(msg);
      pub_cnt++;
    }
    if ((ros::Time::now() - t_start).toSec() > 1.25 && pub_cnt == 1)
    {
      ROS_INFO_STREAM("Publishing EE target 2");
      X_goal_w.z() = 0.4;
      msg.ee_target = X_goal_w;
      msg.duration = 0.25;
      ee_target_pub.publish(msg);
      pub_cnt++;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}