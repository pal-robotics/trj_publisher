#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_pose_publisher");

  ros::NodeHandle n;
  tf::TransformBroadcaster br;
  ros::Publisher head_reference_publisher = n.advertise<geometry_msgs::PointStamped>("input_point", 1);
  ros::Publisher hand_reference_publisher = n.advertise<geometry_msgs::PoseStamped>("right_hand_ref_pose", 1);

  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);

  const double publishFrequency = 15;
  ros::Rate loopRate(publishFrequency);

  const unsigned int WAVE_PERIOD = 4; // In secs.
  const double counts_per_wave = static_cast<double>(WAVE_PERIOD) * publishFrequency;
  unsigned long long count = 0;

  // Main loop
  while (ros::ok())
  {
    using std::sin;
    using std::cos;

    const double angle  = 2.0 * M_PI * (static_cast<double>(count) / counts_per_wave);
    ++count;

    std::vector<tf::StampedTransform> transform_list;
    tf::Transform transform;

    // Publish right and left hand goal frames to tf
    transform.setOrigin(tf::Vector3(0.30,
                                    0.32 + 0.10 * std::sin(angle),
                                    1.00 + 0.10 * std::sin(0.5 * angle)));
//     transform.setOrigin(tf::Vector3(0.30,
//                                     0.32 + 0.10 * std::sin(angle),
//                                     1.00));

    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    transform_list.push_back(tf::StampedTransform(transform, ros::Time::now(), "base_link", "/l_hand_goal"));

    transform.getOrigin()[1] = -transform.getOrigin()[1];
    transform_list.push_back(tf::StampedTransform(transform, ros::Time::now(), "base_link", "/r_hand_goal"));

    br.sendTransform(transform_list);

    // Header
    std_msgs::Header header;
    header.stamp    = ros::Time::now();
    header.frame_id = "base_link";

    // Position
    geometry_msgs::Point point;
    point.x = transform.getOrigin().x();
    point.y = transform.getOrigin().y();
    point.z = transform.getOrigin().z();

    // Orientation
    geometry_msgs::Quaternion quaternion;
    quaternion.x = transform.getRotation().x();
    quaternion.y = transform.getRotation().y();
    quaternion.z = transform.getRotation().z();
    quaternion.w = transform.getRotation().w();

    // Publish right hand position for head tracking as PointStamped
    geometry_msgs::PointStamped p;
    p.header = header;
    p.point  = point;
    head_reference_publisher.publish(p);

    // Publish right hand pose for arm IK as PoseStamped
    geometry_msgs::PoseStamped T;
    T.header           = header;
    T.pose.position    = point;
    T.pose.orientation = quaternion;
    hand_reference_publisher.publish(T);

    // Publish the infos about the marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ball";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    vis_pub.publish(marker);

    ros::spinOnce();
    loopRate.sleep();
  }

  return EXIT_SUCCESS;
}
