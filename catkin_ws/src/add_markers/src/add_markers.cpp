#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class AddObjectMarker
{
private:
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  ros::Subscriber amcl_sub;
  uint32_t shape = visualization_msgs::Marker::CUBE;
  double pick_up[2] = {-3.5, 0}, drop_off[2] = {-3.5, 4};
  double goal_tolerance = 0.1;
  visualization_msgs::Marker marker;

public:
  AddObjectMarker();
  ~AddObjectMarker();
  void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
};

AddObjectMarker::AddObjectMarker()
{
  marker_pub = nh.advertise<visualization_msgs::Marker>("object_visualization", 1);
  amcl_sub = nh.subscribe("/amcl_pose", 1, &AddObjectMarker::amcl_callback, this);
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
}

AddObjectMarker::~AddObjectMarker()
{
}

void AddObjectMarker::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (abs(msg->pose.pose.position.x - pick_up[0]) < goal_tolerance && abs(msg->pose.pose.position.y - pick_up[1]) < goal_tolerance)
  {
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ROS_INFO("Object moving to drop-off location.");
  }
  else if (abs(msg->pose.pose.position.x - drop_off[0]) < goal_tolerance && abs(msg->pose.pose.position.y - drop_off[1]) < goal_tolerance)
  {
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = drop_off[0];
    marker.pose.position.y = drop_off[1];
    ROS_INFO("Object appeared at drop-off location.");
    marker_pub.publish(marker);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_drop_marker");
  AddObjectMarker obj;
  ros::spin();
  return 0;
}
