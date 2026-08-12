#include "fr3_control_sim/robot_model.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

double pose_matrix_distance(const Eigen::Matrix4d &first,
                            const Eigen::Matrix4d &second) {
  return (first.topLeftCorner<3, 3>() - second.topLeftCorner<3, 3>()).norm() +
         (first.topRightCorner<3, 1>() - second.topRightCorner<3, 1>()).norm();
}

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

} // namespace

int main() {
  try {
    fr3_control_sim::RobotModel model(FR3_SIM_DEFAULT_URDF);
    if (model.nq() != 7 || model.nv() != 7) {
      throw std::runtime_error("Reduced FR3 model is not 7-DoF");
    }

    const Eigen::VectorXd q_home = model.home_configuration();
    const Eigen::Matrix4d home_pose = model.forward_kinematics(q_home);
    if (!home_pose.array().isFinite().all()) {
      throw std::runtime_error("FK returned non-finite values");
    }

    Eigen::VectorXd q_target = q_home;
    q_target[0] += 0.25;
    q_target[1] += 0.15;
    q_target[2] -= 0.20;
    q_target[4] += 0.20;
    q_target[6] -= 0.25;
    const Eigen::Matrix4d target_pose = model.forward_kinematics(q_target);

    // Mink's API is differential: one call returns a bounded tangent step,
    // not a final q.  At the target the step should be numerically zero.
    fr3_control_sim::DifferentialIKOptions differential_options;
    differential_options.posture_cost = 0.0;
    const auto zero_step =
        model.differential_ik_step(q_home, home_pose, differential_options);
    require(zero_step.success, "Differential IK zero step was not solved");
    require(zero_step.delta_q.norm() < 1e-8,
            "Differential IK moved at an already solved target");
    require(zero_step.next_task_error < 1e-8,
            "Differential IK zero-step residual is too large");

    const auto one_step =
        model.differential_ik_step(q_home, target_pose, differential_options);
    require(one_step.success, "Differential IK one-step solve failed");
    require(pose_matrix_distance(model.forward_kinematics(one_step.next_q),
                                 target_pose) <
                pose_matrix_distance(home_pose, target_pose),
            "Differential IK one step did not reduce pose error");

    // The velocity box is expressed in tangent displacement units, exactly as
    // in Mink: |delta_q_i| <= vmax_i * dt.
    differential_options.dt = 1e-3;
    differential_options.enforce_velocity_limits = true;
    const auto bounded_step =
        model.differential_ik_step(q_home, target_pose, differential_options);
    const Eigen::VectorXd velocity_limits = model.joint_velocity_limits();
    for (int index = 0; index < model.nv(); ++index) {
      require(std::abs(bounded_step.delta_q[index]) <=
                  velocity_limits[index] * differential_options.dt + 1e-9,
              "Differential IK violated a velocity limit");
    }

    // Position limits are linearized before solving, rather than applied only
    // by a post-solve clamp.
    const auto limits = model.joint_limits();
    Eigen::VectorXd q_at_upper = q_home;
    q_at_upper[0] = limits[0].second - 1e-6;
    Eigen::VectorXd q_upper_target = q_at_upper;
    q_upper_target[0] = limits[0].second;
    fr3_control_sim::DifferentialIKOptions position_options;
    position_options.posture_cost = 0.0;
    position_options.enforce_velocity_limits = false;
    const auto position_step = model.differential_ik_step(
        q_at_upper, model.forward_kinematics(q_upper_target), position_options);
    require(position_step.next_q[0] <= limits[0].second + 1e-9,
            "Differential IK violated an upper position limit");

    fr3_control_sim::IKOptions options;
    options.max_iterations = 1500;
    options.max_retries = 4;
    options.tolerance = 1e-6;
    const auto result = model.inverse_kinematics(target_pose, q_home, options);
    if (!result.success) {
      throw std::runtime_error("IK did not converge, error=" +
                               std::to_string(result.error));
    }

    const Eigen::Matrix4d recovered = model.forward_kinematics(result.q);
    const double position_error =
        (recovered.topRightCorner<3, 1>() - target_pose.topRightCorner<3, 1>())
            .norm();
    if (position_error > 1e-5) {
      throw std::runtime_error("IK/FK position round-trip error is too large");
    }

    // FR3 is redundant: the same TCP pose can be reached with different elbow
    // postures.  The null-space objective should recover the ready/home
    // posture when solving the home target from a deliberately skewed seed.
    Eigen::VectorXd q_skewed(7);
    q_skewed << -27.75, -48.05, 17.44, -134.54, 12.89, 87.89, 29.59;
    q_skewed *= M_PI / 180.0;
    const Eigen::Matrix4d near_home_target =
        fr3_control_sim::pose_from_xyz_rpy(
            Eigen::Vector3d(0.3069, 0.0, 0.4869),
            Eigen::Vector3d(-3.1416, 0.0, 0.0));
    fr3_control_sim::IKOptions posture_options;
    posture_options.max_iterations = 1000;
    posture_options.max_retries = 0;
    posture_options.posture_gain = 0.1;
    const auto posture_result =
        model.inverse_kinematics(near_home_target, q_skewed, posture_options);
    if (!posture_result.success) {
      throw std::runtime_error("Null-space posture IK did not converge");
    }
    const double posture_distance = (posture_result.q - q_home).norm();
    if (posture_distance > 1e-3) {
      throw std::runtime_error(
          "Null-space posture objective did not recover home");
    }

    // The Mink-style wrapper keeps iterating after the frame task reaches its
    // tolerance when a soft PostureTask is enabled.  With no posture task, an
    // already exact target returns the seed immediately; with posture cost it
    // should move the redundant configuration toward the FR3 home posture.
    const Eigen::Matrix4d skewed_target = model.forward_kinematics(q_skewed);
    fr3_control_sim::DifferentialIKOptions no_posture;
    no_posture.max_iterations = 100;
    no_posture.posture_cost = 0.0;
    const auto no_posture_result =
        model.mink_inverse_kinematics(skewed_target, q_skewed, no_posture);
    require(no_posture_result.success,
            "Mink wrapper failed with an already exact target");
    require((no_posture_result.q - q_skewed).norm() < 1e-8,
            "Mink wrapper changed an exact no-posture solution");

    fr3_control_sim::DifferentialIKOptions soft_posture;
    soft_posture.max_iterations = 200;
    soft_posture.tolerance = 1e-5;
    soft_posture.posture_cost = 0.1;
    const auto soft_posture_result =
        model.mink_inverse_kinematics(skewed_target, q_skewed, soft_posture);
    require((soft_posture_result.q - q_home).norm() <
                (no_posture_result.q - q_home).norm(),
            "Mink PostureTask did not reduce home-posture distance");

    const Eigen::MatrixXd trajectory =
        model.minimum_jerk_trajectory(q_home, result.q, 1.0, 0.02);
    if (trajectory.rows() != 51 || trajectory.cols() != 7 ||
        !trajectory.row(0).isApprox(q_home.transpose()) ||
        !trajectory.row(trajectory.rows() - 1).isApprox(result.q.transpose())) {
      throw std::runtime_error("Minimum-jerk trajectory endpoints are invalid");
    }

    options.max_retries = 8;
    for (unsigned int seed = 1000; seed < 1020; ++seed) {
      const Eigen::Matrix4d reachable_pose =
          model.forward_kinematics(model.random_configuration(seed));
      const auto random_result =
          model.inverse_kinematics(reachable_pose, q_home, options);
      if (!random_result.success) {
        throw std::runtime_error(
            "IK random reachable-pose regression failed at seed " +
            std::to_string(seed));
      }
    }

    std::cout << "FR3 C++ smoke test passed\n"
              << "  joints: " << model.nq() << "\n"
              << "  home tcp xyz: "
              << home_pose.topRightCorner<3, 1>().transpose() << "\n"
              << "  IK error: " << result.error << "\n"
              << "  random reachable IK: 20/20\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FR3 C++ smoke test failed: " << error.what() << '\n';
    return 1;
  }
}
