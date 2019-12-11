#include <trackjoint/error_codes.h>
#include <trackjoint/joint_trajectory.h>
#include <trackjoint/trajectory_generator.h>
#include <chrono>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  const int kNumDof = 1;
  const double kTimestep = 0.01;
  const double kDesiredDuration = 1;
  const double kMaxDuration = 2;
  const std::string kOutputPathBase =
      "/home/" + std::string(getenv("USER")) + "/trackjoint_data/output_joint";

  std::vector<trackjoint::KinematicState> current_joint_states;
  trackjoint::KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  current_joint_states.push_back(joint_state);
//  current_joint_states.push_back(joint_state);
//  current_joint_states.push_back(joint_state);

  std::vector<trackjoint::KinematicState> goal_joint_states;
  joint_state.position = -1.1;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  goal_joint_states.push_back(joint_state);
//  goal_joint_states.push_back(joint_state);
//  // Big position change for the third joint
//  joint_state.position = 4;
//  goal_joint_states.push_back(joint_state);

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 1e4;
  single_joint_limits.jerk_limit = 1e6;
  limits.push_back(single_joint_limits);

    // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, kDesiredDuration,
                                           kMaxDuration, current_joint_states,
                                           goal_joint_states, limits);

  std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);

  trackjoint::ErrorCodeEnum error_code = traj_gen.InputChecking(current_joint_states, goal_joint_states, limits, kTimestep);

  // Input error handling - if an error is found, the trajectory is not generated.
  if (error_code != trackjoint::ErrorCodeEnum::kNoError) {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }

  // Measure runtime
  auto start = std::chrono::system_clock::now();
  error_code = traj_gen.GenerateTrajectories(&output_trajectories);
  auto end = std::chrono::system_clock::now();

  // Trajectory generation error handling
  if (error_code != trackjoint::ErrorCodeEnum::kNoError) {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }

  std::chrono::duration<double> elapsed_seconds = end-start;

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  /**
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("chatter", 1000);

  ros::Rate loop_rate(10);

  std_msgs::Float64 double_msg;

  int count = 0;
  while (ros::ok() && count < 50)
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    double_msg.data = output_trajectories.at(0).positions(count);

    ROS_INFO("%f", double_msg.data);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(double_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

