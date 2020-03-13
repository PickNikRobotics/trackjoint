#include "trackjoint/utilities.h"

namespace trackjoint
{
void PrintJointTrajectory(const std::size_t joint, const std::vector<trackjoint::JointTrajectory>& output_trajectories,
                          const double desired_duration)
{
  std::cout << "==========" << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "==========" << std::endl;
  std::cout << "Joint " << joint << std::endl;
  std::cout << "==========" << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "==========" << std::endl;
  std::cout << "  Num waypts: " << output_trajectories.at(joint).positions.size() << std::endl;
  std::cout << "  Desired duration: " << desired_duration << std::endl;
  std::cout << "Timestep: " << output_trajectories.at(joint).elapsed_times[1]
            << "  Initial position: " << output_trajectories.at(joint).positions[0]
            << "  Initial velocity: " << output_trajectories.at(joint).velocities[0]
            << "  Initial acceleration: " << output_trajectories.at(joint).accelerations[0]
            << "  Initial jerk: " << output_trajectories.at(joint).jerks[0] << std::endl;
  std::cout << "  Final position: "
            << output_trajectories.at(joint).positions[output_trajectories.at(joint).positions.size() - 1]
            << "  Final velocity: "
            << output_trajectories.at(joint).velocities[output_trajectories.at(joint).positions.size() - 1]
            << "  Final acceleration: "
            << output_trajectories.at(joint).accelerations[output_trajectories.at(joint).positions.size() - 1]
            << "  Final jerk: "
            << output_trajectories.at(joint).jerks[output_trajectories.at(joint).positions.size() - 1] << std::endl;
}
}  // namespace trackjoint
