#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <trj_publisher/TrjPublisherConfig.h>

bool side = true;

void callback(trj_publisher::TrjPublisherConfig &config) {
  //ROS_INFO("Reconfigure Request: %s %s",
  //          config.side?"True":"False");

    side = config.side;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_pose_publisher");

  ros::NodeHandle n;
  tf::TransformBroadcaster br;
  ros::Publisher head_reference_publisher = n.advertise<geometry_msgs::PointStamped>("head_ref_point", 1);

  ros::Publisher r_hand_reference_publisher = n.advertise<geometry_msgs::PoseStamped>("right_hand_ref_pose", 1);
  ros::Publisher l_hand_reference_publisher = n.advertise<geometry_msgs::PoseStamped>("left_hand_ref_pose", 1);

  dynamic_reconfigure::Server<trj_publisher::TrjPublisherConfig> server;
  dynamic_reconfigure::Server<trj_publisher::TrjPublisherConfig>::CallbackType f;

  // Vector
  std::vector<ros::Publisher> hand_publisher_list;
  hand_publisher_list.push_back(r_hand_reference_publisher);
  hand_publisher_list.push_back(l_hand_reference_publisher);

  ros::Publisher r_vis_pub = n.advertise<visualization_msgs::Marker>("right_hand_marker",10);
  ros::Publisher l_vis_pub = n.advertise<visualization_msgs::Marker>("left_hand_marker",10);

  // Vector
  std::vector<ros::Publisher> vis_pub_list;
  vis_pub_list.push_back(r_vis_pub);
  vis_pub_list.push_back(l_vis_pub);


  const double publishFrequency = 15;
  ros::Rate loopRate(publishFrequency);

  const unsigned int WAVE_PERIOD = 4; // In secs.
  const double counts_per_wave = static_cast<double>(WAVE_PERIOD) * publishFrequency;
  unsigned long long count = 0;

  f = boost::bind(&callback, _1);
  server.setCallback(f);

  // Main loop
  while (ros::ok())
  {
    using std::sin;
    using std::cos;

    const double angle  = 2.0 * M_PI * (static_cast<double>(count) / counts_per_wave);
    ++count;

    std::vector<tf::StampedTransform> transform_list;
    tf::Transform transform_r , transform_l;

    // Publish right and left hand goal frames to tf

    transform_l.setOrigin(tf::Vector3(0.30,
                                      0.32,
                                      1.25 + 0.1 * std::sin(0.5 * angle)));
    transform_l.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));


    transform_r.setOrigin(tf::Vector3(0.30,
                                      0.32 + 0.08 * std::sin(angle),
                                      1.25 + 0.08 * std::sin(0.5 * angle)));
    transform_r.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    transform_r.getOrigin()[1] = - transform_r.getOrigin()[1];

    transform_list.push_back(tf::StampedTransform(transform_r, ros::Time::now(), "base_link", "/r_hand_goal"));
    transform_list.push_back(tf::StampedTransform(transform_l, ros::Time::now(), "base_link", "/l_hand_goal"));

    br.sendTransform(transform_list);

    // Header
    std_msgs::Header header;
    header.stamp    = ros::Time::now();
    header.frame_id = "base_link";

    tf::Transform transform;

    if(side)
        transform = transform_l;
    else
        transform = transform_r;

    geometry_msgs::Point point;
    point.x = transform.getOrigin().x();
    point.y = transform.getOrigin().y();
    point.z = transform.getOrigin().z();
    geometry_msgs::PointStamped p;
    p.header = header;
    p.point  = point;
    head_reference_publisher.publish(p);

    for(int i = 0; i<2; i++){

        tf::Transform transform = transform_list[i];

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

        // Publish right hand pose for arm IK as PoseStamped
        geometry_msgs::PoseStamped T;
        T.header           = header;
        T.pose.position    = point;
        T.pose.orientation = quaternion;
        hand_publisher_list[i].publish(T);

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
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        vis_pub_list[i].publish(marker);

    }

    ros::spinOnce();
    loopRate.sleep();
  }

  return EXIT_SUCCESS;
}
