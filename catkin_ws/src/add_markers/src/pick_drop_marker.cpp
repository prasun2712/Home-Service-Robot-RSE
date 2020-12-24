#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class AddObjectMarker
{
private:
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  uint32_t shape = visualization_msgs::Marker::CUBE;
  double pick_up[2] = {-3.5, 0}, drop_off[2] = {-3.5, 4};
  visualization_msgs::Marker marker;

public:
  AddObjectMarker();
  ~AddObjectMarker();
  void publish_marker();
};

AddObjectMarker::AddObjectMarker()
{
  marker_pub = nh.advertise<visualization_msgs::Marker>("object_visualization", 1);
  marker.lifetime = ros::Duration();
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "object_cube";
  marker.id = 0;
  marker.type = shape;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 1.0;
  marker.pose.orientation.w = 1.0;
}

AddObjectMarker::~AddObjectMarker()
{
}

void AddObjectMarker::publish_marker()
{
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pick_up[0];
  marker.pose.position.y = pick_up[1];
  ROS_INFO("Object appeared at pick-up location.");
  marker_pub.publish(marker);
  sleep(5);
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Object moving to drop-off location.");
  sleep(5);
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = drop_off[0];
  marker.pose.position.y = drop_off[1];
  ROS_INFO("Object appeared at drop-off location.");
  marker_pub.publish(marker);
  sleep(5);
  ROS_INFO("Press Ctrl+c to exit.");
  while (ros::ok())
  {
    ros::spinOnce();
  }
  ROS_INFO("Exiting.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_drop_marker");
  AddObjectMarker obj;
  obj.publish_marker();
  return 0;
}
