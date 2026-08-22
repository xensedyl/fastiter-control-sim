#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <map>
#include <limits>
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

// URDF adaptation of the weighted redundant-joint strategy used by
// LeFranX/franka_xhand_teleoperator.  This is intentionally separate from
// IKOptions/inverse_kinematics(): the latter is the project's original
// damped iterative solver, while this API scans one free joint, solves the
// remaining six coordinates, and ranks valid candidates.
struct FrankaWeightedIKOptions {
  int free_joint_index = -1;       // -1: joint_7 when present, otherwise nq-1
  int samples = 121;               // grid points over the free-joint limits
  int max_iterations = 80;        // fixed-free local solve iterations
  double tolerance = 1e-5;
  double damping = 1e-6;
  double step_size = 0.7;
  double max_step_norm = 0.25;
  double weight_manipulability = 1.0;
  double weight_neutral = 1.0;
  double weight_current = 1.0;
  Eigen::VectorXd neutral_q;       // empty: RobotModel::home_configuration()
  Eigen::VectorXd joint_weights;   // empty: all ones
};

struct FrankaWeightedIKResult {
  Eigen::VectorXd q;
  bool success = false;
  double score = -std::numeric_limits<double>::infinity();
  double manipulability = 0.0;
  double neutral_distance = 0.0;
  double current_distance = 0.0;
  double error = std::numeric_limits<double>::infinity();
  double position_error = std::numeric_limits<double>::infinity();
  double orientation_error = std::numeric_limits<double>::infinity();
  int free_joint_index = -1;
  int samples_tested = 0;
  int valid_solutions = 0;
  int iterations = 0;
};

class RobotModel {
public:
  explicit RobotModel(const std::string &urdf_path,
                      const std::string &end_effector_frame = "",
                      double finger_position = 0.02);

  int nq() const { return model_.nq; }
  int nv() const { return model_.nv; }
  const std::string &urdf_path() const { return urdf_path_; }
  const std::string &end_effector_frame() const { return end_effector_frame_; }
  double finger_position() const { return finger_position_; }

  std::vector<std::string> joint_names() const;
  std::vector<std::string> mimic_joint_names() const;
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

  // LeFranX-style weighted redundant-joint IK adapted to this URDF.  The
  // analytic geofik formulas in the original project are hard-coded for the
  // official Franka dimensions and therefore are not used here.  Instead,
  // this method keeps its free-joint scan and weighted candidate ranking while
  // obtaining FK/Jacobians from the loaded URDF (including mimic joints).
  FrankaWeightedIKResult franka_weighted_ik(
      const Eigen::Matrix4d &target, const Eigen::VectorXd &q_seed,
      const FrankaWeightedIKOptions &options = FrankaWeightedIKOptions(),
      const std::string &frame_name = "") const;

  // Descriptive alias matching the source algorithm's project name.
  FrankaWeightedIKResult lefranx_weighted_ik(
      const Eigen::Matrix4d &target, const Eigen::VectorXd &q_seed,
      const FrankaWeightedIKOptions &options = FrankaWeightedIKOptions(),
      const std::string &frame_name = "") const {
    return franka_weighted_ik(target, q_seed, options, frame_name);
  }

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
