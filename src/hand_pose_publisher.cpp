#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <trj_publisher/TrjPublisherConfig.h>

bool side = false;

std::string ref_frame = "floor_link";
int publish_frequency = 15;
int wave_period = 4;

double left_wave_center_x= 0.3;
double left_wave_center_y = 0.32;
double left_wave_center_z = 1.1;
double left_wave_speed = 0.5;
double left_amplitude_x = 0.00;
double left_amplitude_y = 0.05;
double left_amplitude_z = 0.05;

double right_wave_center_x = 0.3;
double right_wave_center_y = 0.32;
double right_wave_center_z = 1.1;
double right_wave_speed = 0.5;
double right_amplitude_x = 0.00;
double right_amplitude_y = 0.00;
double right_amplitude_z = 0.08;


void callback(trj_publisher::TrjPublisherConfig &config) {

    side = config.side;
    publish_frequency = config.publish_frequency;
    wave_period = config.wave_period;
    
    left_wave_center_x = config.left_wave_center_x;
    left_wave_center_y = config.left_wave_center_y;
    left_wave_center_z = config.left_wave_center_z;
    left_wave_speed = config.left_wave_speed;
    left_amplitude_x = config.left_amplitude_x;
    left_amplitude_y = config.left_amplitude_y;
    left_amplitude_z = config.left_amplitude_z;
    
    right_wave_center_x = config.right_wave_center_x;
    right_wave_center_y = config.right_wave_center_y;
    right_wave_center_z = config.right_wave_center_z;
    right_wave_speed = config.right_wave_speed;
    right_amplitude_x = config.right_amplitude_x;
    right_amplitude_y = config.right_amplitude_y;
    right_amplitude_z = config.right_amplitude_z;
    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_pose_publisher");

  ros::NodeHandle n("~");
  tf::TransformBroadcaster br;
  ros::Publisher head_reference_publisher = n.advertise<geometry_msgs::Vector3Stamped>("head_ref_point", 1);
  ros::Publisher r_hand_reference_publisher = n.advertise<geometry_msgs::TransformStamped>("right_hand_ref_pose", 1);
  ros::Publisher l_hand_reference_publisher = n.advertise<geometry_msgs::TransformStamped>("left_hand_ref_pose", 1);

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

  const double publishFrequency = publish_frequency;
  ros::Rate loopRate(publishFrequency);

  const unsigned int WAVE_PERIOD = wave_period; // In secs.
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
    transform_l.setOrigin(tf::Vector3(left_wave_center_x + left_amplitude_x * std::sin(left_wave_speed * angle),
                                      left_wave_center_y + left_amplitude_y * std::sin(left_wave_speed * angle),
                                      left_wave_center_z + left_amplitude_z * std::sin(left_wave_speed * angle)));
    transform_l.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    transform_r.setOrigin(tf::Vector3(right_wave_center_x + right_amplitude_x * std::sin(right_wave_speed * angle),
                                      right_wave_center_y + right_amplitude_y * std::sin(right_wave_speed * angle),
                                      right_wave_center_z + right_amplitude_z * std::sin(right_wave_speed * angle)));
    transform_r.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    transform_list.push_back(tf::StampedTransform(transform_r, ros::Time::now(), ref_frame, "/r_hand_goal"));
    transform_list.push_back(tf::StampedTransform(transform_l, ros::Time::now(), ref_frame, "/l_hand_goal"));

    br.sendTransform(transform_list);

    // Header
    std_msgs::Header header;
    header.stamp    = ros::Time::now();
    header.frame_id = ref_frame;

    tf::Transform transform;

    if(side)
        transform = transform_l;
    else
        transform = transform_r;

    geometry_msgs::Vector3Stamped p;
    p.header = header;
    p.vector.x = transform.getOrigin().x();
    p.vector.y = transform.getOrigin().y();
    p.vector.z = transform.getOrigin().z();
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

        // Publish right hand pose for arm IK as TransformStamped
        geometry_msgs::TransformStamped T;
        T.header = header;
        T.transform.translation.x = transform.getOrigin().x();
        T.transform.translation.y = transform.getOrigin().y();
        T.transform.translation.z = transform.getOrigin().z();

        T.transform.rotation.x = transform.getRotation().x();
        T.transform.rotation.y = transform.getRotation().y();
        T.transform.rotation.z = transform.getRotation().z();
        T.transform.rotation.w = transform.getRotation().w();

        hand_publisher_list[i].publish(T);

        // Publish the infos about the marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = ref_frame;
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
