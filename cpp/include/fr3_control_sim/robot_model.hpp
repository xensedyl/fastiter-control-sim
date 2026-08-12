#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace fr3_control_sim {

struct IKOptions {
  int max_iterations = 1000;
  int max_retries = 8;
  double tolerance = 1e-5;
  double damping = 1e-6;
  double step_size = 0.7;
  double max_step_norm = 0.25;
  double posture_gain = 0.1;
  int line_search_steps = 8;
  unsigned int random_seed = 42;
};

struct IKResult {
  Eigen::VectorXd q;
  bool success = false;
  int iterations = 0;
  int attempts = 0;
  double error = 0.0;
  double position_error = 0.0;
  double orientation_error = 0.0;
};

// Options for one Mink-style differential IK step.  Unlike IKOptions (which
// configures the existing batch solver), these values describe a single
// linearized QP solved at the current configuration.  All revolute quantities
// are in radians and all translations are in meters.
struct DifferentialIKOptions {
  double dt = 0.02;
  double damping = 1e-6;
  // These two fields are used by mink_inverse_kinematics(), the convenience
  // wrapper that repeatedly calls differential_ik_step().  They do not alter
  // a single differential step.
  int max_iterations = 200;
  double tolerance = 1e-5;
  int qp_max_active_sets = 4096;
  // KKT/active-set stopping tolerance for the native box-QP solver.
  double qp_tolerance = 1e-9;
  double frame_gain = 1.0;
  Eigen::Vector3d position_cost = Eigen::Vector3d::Ones();
  Eigen::Vector3d orientation_cost = Eigen::Vector3d::Ones();
  double frame_lm_damping = 0.0;

  // A zero cost disables the soft posture task.  An empty target means the
  // FR3 ready/home configuration supplied by RobotModel::home_configuration().
  double posture_cost = 0.0;
  Eigen::VectorXd posture_costs;
  double posture_gain = 1.0;
  double posture_lm_damping = 0.0;
  Eigen::VectorXd posture_target;

  // Mink's ConfigurationLimit is enabled by default.  The first version of
  // this native port supports the common FR3 box constraints (position and
  // velocity); collision/equality constraints are intentionally left for a
  // later QP backend.
  bool enforce_position_limits = true;
  double position_limit_gain = 0.95;
  bool enforce_velocity_limits = false;
  Eigen::VectorXd velocity_limits;
};

struct DifferentialIKResult {
  Eigen::VectorXd delta_q;
  Eigen::VectorXd velocity;
  Eigen::VectorXd next_q;
  bool success = false;
  double objective = 0.0;
  double task_error = 0.0;
  double next_task_error = 0.0;
  double position_error = 0.0;
  double orientation_error = 0.0;
  int active_lower = 0;
  int active_upper = 0;
  std::string status;
};

class RobotModel {
public:
  explicit RobotModel(const std::string &urdf_path,
                      const std::string &end_effector_frame = "fr3_hand_tcp",
                      double finger_position = 0.02);

  int nq() const { return model_.nq; }
  int nv() const { return model_.nv; }
  const std::string &urdf_path() const { return urdf_path_; }
  const std::string &end_effector_frame() const { return end_effector_frame_; }
  double finger_position() const { return finger_position_; }

  std::vector<std::string> joint_names() const;
  std::vector<std::pair<double, double>> joint_limits() const;
  std::vector<std::string> frame_names() const;

  Eigen::VectorXd home_configuration() const;
  Eigen::VectorXd random_configuration(unsigned int seed = 42) const;

  Eigen::Matrix4d forward_kinematics(const Eigen::VectorXd &q,
                                     const std::string &frame_name = "") const;
  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q,
                           const std::string &frame_name = "") const;
  std::map<std::string, Eigen::Matrix4d>
  frame_placements(const Eigen::VectorXd &q) const;

  IKResult inverse_kinematics(const Eigen::Matrix4d &target,
                              const Eigen::VectorXd &q_seed,
                              const IKOptions &options = IKOptions()) const;

  // Solve one Mink-style differential IK/QP step.  The result contains the
  // tangent displacement delta_q, its velocity delta_q / dt, and the
  // Pinocchio-integrated next configuration.  Call this method repeatedly for
  // a closed-loop controller; it deliberately does not replace the existing
  // one-shot inverse_kinematics() API.
  DifferentialIKResult differential_ik_step(
      const Eigen::VectorXd &q, const Eigen::Matrix4d &target,
      const DifferentialIKOptions &options = DifferentialIKOptions(),
      const std::string &frame_name = "") const;

  // Convenience batch wrapper around repeated differential_ik_step() calls.
  // This is the closest equivalent to Mink's examples, while the step API is
  // intended for a real-time/control loop.
  IKResult mink_inverse_kinematics(
      const Eigen::Matrix4d &target, const Eigen::VectorXd &q_seed,
      const DifferentialIKOptions &options = DifferentialIKOptions()) const;

  Eigen::VectorXd integrate_configuration(const Eigen::VectorXd &q,
                                           const Eigen::VectorXd &delta_q) const;

  Eigen::VectorXd joint_velocity_limits() const;

  Eigen::MatrixXd minimum_jerk_trajectory(const Eigen::VectorXd &q_start,
                                          const Eigen::VectorXd &q_goal,
                                          double duration,
                                          double dt = 0.02) const;

private:
  struct ErrorState {
    double norm = 0.0;
    double position_norm = 0.0;
    double orientation_norm = 0.0;
    Eigen::Matrix<double, 6, 1> vector = Eigen::Matrix<double, 6, 1>::Zero();
  };

  std::string urdf_path_;
  std::string end_effector_frame_;
  double finger_position_;
  pinocchio::Model model_;
  mutable pinocchio::Data data_;

  void validate_configuration(const Eigen::VectorXd &q) const;
  pinocchio::FrameIndex resolve_frame(const std::string &frame_name) const;
  Eigen::VectorXd clamp_configuration(const Eigen::VectorXd &q) const;
  ErrorState pose_error(const Eigen::VectorXd &q, const pinocchio::SE3 &target,
                        pinocchio::FrameIndex frame_id) const;
  IKResult inverse_kinematics_once(const pinocchio::SE3 &target,
                                   const Eigen::VectorXd &q_seed,
                                   pinocchio::FrameIndex frame_id,
                                   const IKOptions &options) const;
};

Eigen::Matrix4d pose_from_xyz_rpy(const Eigen::Vector3d &xyz,
                                  const Eigen::Vector3d &rpy);
Eigen::Vector3d rpy_from_pose(const Eigen::Matrix4d &pose);

} // namespace fr3_control_sim
