#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <iostream>
#include <sstream>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>

#include <math.h>

ros::Subscriber desire_traj_vertices_sub;
ros::Publisher desired_state_pub;
tf::TransformBroadcaster* br;
ros::Timer heartbeat;

/**
 * Callback function for listening to the desired vertices msgs
 */
void trajCB(const geometry_msgs::PoseArray& traj_msg) {
  // sanity check for traj_msg size
  if (traj_msg.poses.size() == 0) {
    ROS_ERROR_THROTTLE(1, "Empty trajectory vertices msg.");
    return;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 0 |  16.485 - Fall 2020  - Lab 4 coding assignment  (10 pts)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  //  As a simple warm up exercise before we get to the actual 'real deal',
  //  let's just make our quadcopter fly to the first gate in the course.
  //  In this section:
  //   1. Extract the first vertex of the trajectory
  //   2. Set the acceleration and velocities to zero
  //   3. Publish the desired MultiDOFJointTrajectoryPoint
  //   4. Create and publish TF transform of the desired pose
  // ~~~~ begin solution

  geometry_msgs::Pose firstVertex = traj_msg.poses[0];
  geometry_msgs::Point vertexPos = firstVertex.position;
  ROS_INFO_STREAM("VERTEX POS" << vertexPos. x << ' ' << vertexPos. y << ' ' << vertexPos. z << '\n' );
  
  geometry_msgs::Vector3 desiredPos;
  desiredPos.x= vertexPos.x; desiredPos.y= vertexPos.y; desiredPos.z= vertexPos.z;
  geometry_msgs::Quaternion desiredOri = firstVertex.orientation;
  
  Eigen::Quaterniond qd;
  Eigen::fromMsg(desiredOri,qd);
  Eigen::Matrix3d R = qd.toRotationMatrix();
  double roll = std::atan2(R(2,0), R(2,1));
  double pitch = std::acos(R(2,2));
  double yaw = -std::atan2(R(0,2), R(1,2));
  ROS_INFO_STREAM( "Roll, Pitch, Yaw \n" << roll << ' '<< pitch << ' '<< yaw << "\n" );

  trajectory_msgs::MultiDOFJointTrajectoryPoint des_state;

  geometry_msgs::Transform desiredVertex;
  desiredVertex.rotation = desiredOri;
  desiredVertex.translation = desiredPos;

  geometry_msgs::Twist velocity;
  velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
  velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
  geometry_msgs::Twist acceleration;
  acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
  acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;
  
  des_state.transforms.resize(1);
  des_state.velocities.resize(1);
  des_state.accelerations.resize(1);

  des_state.transforms[0]=desiredVertex;
  des_state.velocities[0]=velocity;
  des_state.accelerations[0]=acceleration;

  //publishing the desired MultiDOFJointTrajectoryPoint
  desired_state_pub.publish(des_state);

  //creating and publishing TF transform of the desired pose
  tf::Transform desired_pose(tf::Transform::getIdentity());
  tf::Vector3 origin(desiredPos.x,desiredPos.y,desiredPos.z);

  tf::Quaternion q;
  tf::quaternionMsgToTF(desiredOri , q);
  
  desired_pose.setOrigin(origin);
  desired_pose.setRotation(q);

  ROS_INFO_STREAM( "Trajectory Position"
           << " x:" << desired_pose.getOrigin().x()
           << " y:" << desired_pose.getOrigin().y()
           << " z:" << desired_pose.getOrigin().z() <<'\n');
  
  br->sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));

  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 0
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_traj_planner");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);
  ros::Time start(ros::Time::now());

  // deired traj vertices subscriber
  desire_traj_vertices_sub = n.subscribe("desired_traj_vertices", 10, trajCB);

  // publisher for desired states for controller
  desired_state_pub =
      n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          "desired_state", 1);

  br = new tf::TransformBroadcaster();

  ros::spin();
}
