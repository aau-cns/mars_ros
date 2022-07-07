#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

class PoseConverter
{
private:
  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster br_tf2_;
  ros::Subscriber sub_pose_;

  std::string parent_id_{ "child" };

  double t_newest_meas_{ 0.0 };

private:
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    // only publish in-order poses to avoid this issue:
    // https://github.com/ros/geometry2/issues/467
    if (msg->header.stamp.toSec() > t_newest_meas_)
    {
      geometry_msgs::TransformStamped transformStamped;

      transformStamped.header.stamp = msg->header.stamp; /*
       transformStamped.header.frame_id = parent_id_;
       transformStamped.child_frame_id = msg->header.frame_id;*/
      transformStamped.header.frame_id = msg->header.frame_id;
      transformStamped.child_frame_id = parent_id_;  // msg->header.frame_id;
      transformStamped.transform.translation.x = msg->pose.position.x;
      transformStamped.transform.translation.y = msg->pose.position.y;
      transformStamped.transform.translation.z = msg->pose.position.z;
      transformStamped.transform.rotation.x = msg->pose.orientation.x;
      transformStamped.transform.rotation.y = msg->pose.orientation.y;
      transformStamped.transform.rotation.z = msg->pose.orientation.z;
      transformStamped.transform.rotation.w = msg->pose.orientation.w;

      br_tf2_.sendTransform(transformStamped);
      t_newest_meas_ = msg->header.stamp.toSec();
    }
  }

public:
  PoseConverter(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.param<std::string>("parent_id", parent_id_, parent_id_);

    sub_pose_ = nh_.subscribe("pose_in", 1, &PoseConverter::PoseCallback, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mars_tf2_converter");
  ros::NodeHandle nh("~");

  PoseConverter conv(nh);

  ros::spin();
  return EXIT_SUCCESS;
};
