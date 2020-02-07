#include "trackjoint/utilities.h"

namespace trackjoint
{
/**
 * \brief Discrete differentiation of a vector
 *
 * input: first_element: we usually have an initial condition, so supply it
 * directly.
 */
Eigen::VectorXd DiscreteDifferentiation(const Eigen::VectorXd& input_vector, double timestep, double first_element)
{
  // derivative = (difference between adjacent elements) / timestep
  Eigen::VectorXd input_shifted_right(input_vector.size());
  input_shifted_right(0) = 0;
  input_shifted_right.tail(input_shifted_right.size() - 1) = input_vector.head(input_vector.size() - 1);
  Eigen::VectorXd derivative(input_vector.size());
  derivative(0) = first_element;
  derivative.tail(derivative.size() - 1) =
      (input_vector.tail(input_vector.size() - 1) - input_shifted_right.tail(input_shifted_right.size() - 1)) /
      timestep;

  return derivative;
};

void PrintJointTrajectory(const std::size_t joint, const std::vector<trackjoint::JointTrajectory>& output_trajectories)
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
  std::cout << "Elapsed time: " << output_trajectories.at(joint).elapsed_times[0]
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
