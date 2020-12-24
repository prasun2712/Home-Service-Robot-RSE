#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  ros::init(argc, argv, "pick_drop_objects");

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -3.5;
  goal.target_pose.pose.orientation.z = 1.0;

  ROS_INFO("Robot: Moving to pickup location.");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Robot: Reached pickup location.");
    sleep(5);
    ROS_INFO("Robot: Picked up the object.");
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.y = 4.0;
    goal.target_pose.pose.orientation.z = 1.0;
    sleep(2);    
    ROS_INFO("Robot: Moving to drop-off location.");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Robot: Reached drop-off location.");
    else
      ROS_INFO("Robot: Failed to reach drop-off location.");
  }
  else
    ROS_INFO("Robot: Failed to reach pickup location.");
  ROS_INFO("Press Ctrl+c to exit.");
  while( ros::ok())
    {
        ros::spinOnce();
    }
  ROS_INFO("Exiting.");
  return 0;
}
