#include "trackjoint/utilities.h"

namespace trackjoint
{
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

void PrintJointTrajectory(const std::size_t joint, const std::vector<trackjoint::JointTrajectory>& output_trajectories,
                          const double desired_duration)
{
  std::cout << "==========" << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "==========" << std::endl;
  std::cout << "Joint " << joint << std::endl;
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

void ClipEigenVector(Eigen::VectorXd* vector, size_t new_num_waypoints)
{
  Eigen::VectorXd new_vector = vector->head(new_num_waypoints);
  *vector = new_vector;
}

bool VerifyVectorWithinBounds(double low_limit, double high_limit, Eigen::VectorXd& vector)
{
  if (high_limit < low_limit)
    return false;

  return ((vector.array() >= low_limit).all() && (vector.array() <= high_limit).all());
}
}  // namespace trackjoint
