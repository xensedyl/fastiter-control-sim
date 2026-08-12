#include "fr3_control_sim/robot_model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace fr3_control_sim {
namespace {

bool is_finite_matrix(const Eigen::Matrix4d &matrix) {
  return matrix.array().isFinite().all();
}

bool has_joint(const pinocchio::Model &model, const std::string &name) {
  return model.existJointName(name);
}

void validate_differential_options(const DifferentialIKOptions &options,
                                   int nq, int nv, bool validate_batch_fields) {
  if (!std::isfinite(options.dt) || options.dt <= 0.0 ||
      !std::isfinite(options.damping) || options.damping < 0.0 ||
      !std::isfinite(options.frame_gain) || options.frame_gain < 0.0 ||
      options.frame_gain > 1.0 ||
      !std::isfinite(options.frame_lm_damping) ||
      options.frame_lm_damping < 0.0 ||
      !std::isfinite(options.posture_cost) || options.posture_cost < 0.0 ||
      !std::isfinite(options.posture_gain) || options.posture_gain < 0.0 ||
      options.posture_gain > 1.0 || options.qp_max_active_sets <= 0 ||
      !std::isfinite(options.qp_tolerance) || options.qp_tolerance <= 0.0 ||
      !std::isfinite(options.position_limit_gain) ||
      options.position_limit_gain <= 0.0 ||
      options.position_limit_gain > 1.0) {
    throw std::invalid_argument("Invalid differential IK options");
  }
  if (validate_batch_fields && options.max_iterations <= 0) {
    throw std::invalid_argument(
        "differential IK max_iterations must be positive");
  }
  if (validate_batch_fields &&
      (!std::isfinite(options.tolerance) || options.tolerance <= 0.0)) {
    throw std::invalid_argument(
        "differential IK tolerance must be finite and positive");
  }
  if (!options.position_cost.array().isFinite().all() ||
      !options.orientation_cost.array().isFinite().all() ||
      (options.position_cost.array() < 0.0).any() ||
      (options.orientation_cost.array() < 0.0).any()) {
    throw std::invalid_argument(
        "differential IK frame costs must be finite and non-negative");
  }
  if (options.posture_costs.size() != 0) {
    if (options.posture_costs.size() != nv ||
        !options.posture_costs.array().isFinite().all() ||
        (options.posture_costs.array() < 0.0).any()) {
      throw std::invalid_argument(
          "posture_costs must be empty or finite non-negative nv values");
    }
  }
  if (options.posture_target.size() != 0) {
    if (options.posture_target.size() != nq ||
        !options.posture_target.array().isFinite().all()) {
      throw std::invalid_argument(
          "posture_target must be empty or a finite nq configuration");
    }
  }
  if (options.velocity_limits.size() != 0 &&
      (options.velocity_limits.size() != nv ||
       !options.velocity_limits.array().isFinite().all() ||
       (options.velocity_limits.array() < 0.0).any())) {
    throw std::invalid_argument(
        "velocity_limits must be empty or finite non-negative nv values");
  }
}

pinocchio::SE3 target_from_matrix(const Eigen::Matrix4d &target_matrix) {
  if (!is_finite_matrix(target_matrix)) {
    throw std::invalid_argument("target contains NaN or infinity");
  }
  const Eigen::RowVector4d homogeneous_row(0.0, 0.0, 0.0, 1.0);
  if (!target_matrix.row(3).isApprox(homogeneous_row, 1e-8)) {
    throw std::invalid_argument("target last row must be [0, 0, 0, 1]");
  }

  const Eigen::Matrix3d rotation = target_matrix.topLeftCorner<3, 3>();
  const double orthogonality_error =
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
  if (orthogonality_error > 1e-5 || rotation.determinant() <= 0.0) {
    throw std::invalid_argument(
        "target rotation must be a proper rotation matrix");
  }
  Eigen::Quaterniond quaternion(rotation);
  if (quaternion.norm() < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("target rotation is singular");
  }
  quaternion.normalize();
  return pinocchio::SE3(quaternion.toRotationMatrix(),
                        target_matrix.topRightCorner<3, 1>());
}

struct BoxQPResult {
  Eigen::VectorXd x;
  bool success = false;
  int active_lower = 0;
  int active_upper = 0;
  std::string status;
};

Eigen::VectorXd solve_symmetric_system(const Eigen::MatrixXd &matrix,
                                       const Eigen::VectorXd &rhs) {
  const auto acceptable = [&](const Eigen::VectorXd &result) {
    if (!result.array().isFinite().all()) {
      return false;
    }
    const double scale = std::max(
        1.0, std::max(rhs.norm(), matrix.norm() * result.norm()));
    return (matrix * result - rhs).norm() <= 1e-8 * scale;
  };

  Eigen::LDLT<Eigen::MatrixXd> ldlt(matrix);
  if (ldlt.info() == Eigen::Success) {
    const Eigen::VectorXd result = ldlt.solve(rhs);
    if (acceptable(result)) {
      return result;
    }
  }

  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition(matrix);
  const Eigen::VectorXd result = decomposition.solve(rhs);
  if (!acceptable(result)) {
    throw std::runtime_error("differential IK QP linear system is singular");
  }
  return result;
}

// Solve the small bound-constrained convex QP used by the FR3 port.  Mink
// represents configuration and velocity limits as linear inequalities.  For
// the reduced FR3 model all tangent coordinates are revolute joint angles, so
// their intersection is a box.  This dense working-set solver avoids adding a
// Python QP dependency and gives the same KKT solution for these bounds.
BoxQPResult solve_box_qp(const Eigen::MatrixXd &input_hessian,
                         const Eigen::VectorXd &linear,
                         const Eigen::VectorXd &lower,
                         const Eigen::VectorXd &upper,
                         int max_iterations, double tolerance) {
  const int dimension = static_cast<int>(linear.size());
  if (input_hessian.rows() != dimension || input_hessian.cols() != dimension ||
      lower.size() != dimension || upper.size() != dimension) {
    throw std::invalid_argument("differential IK QP dimensions do not match");
  }
  for (int index = 0; index < dimension; ++index) {
    if (std::isfinite(lower[index]) && std::isfinite(upper[index]) &&
        lower[index] > upper[index] + tolerance) {
      return {Eigen::VectorXd::Zero(dimension), false, 0, 0,
              "infeasible differential IK bounds"};
    }
  }

  Eigen::MatrixXd hessian =
      0.5 * (input_hessian + input_hessian.transpose());
  if (!hessian.array().isFinite().all()) {
    return {Eigen::VectorXd::Zero(dimension), false, 0, 0,
            "differential IK QP Hessian is non-finite"};
  }
  Eigen::VectorXd unconstrained;
  try {
    unconstrained = solve_symmetric_system(hessian, -linear);
  } catch (const std::exception &) {
    return {Eigen::VectorXd::Zero(dimension), false, 0, 0,
            "differential IK QP Hessian is singular"};
  }

  enum class BoundState { Free, Lower, Upper };
  std::vector<BoundState> state(static_cast<std::size_t>(dimension),
                                BoundState::Free);
  Eigen::VectorXd x = unconstrained;
  for (int index = 0; index < dimension; ++index) {
    if (std::isfinite(lower[index]) && x[index] < lower[index]) {
      x[index] = lower[index];
      state[static_cast<std::size_t>(index)] = BoundState::Lower;
    } else if (std::isfinite(upper[index]) && x[index] > upper[index]) {
      x[index] = upper[index];
      state[static_cast<std::size_t>(index)] = BoundState::Upper;
    }
  }

  const int iteration_limit = std::max(1, max_iterations);
  std::set<std::vector<int>> visited_active_sets;
  for (int iteration = 0; iteration < iteration_limit; ++iteration) {
    std::vector<int> active_set;
    active_set.reserve(static_cast<std::size_t>(dimension));
    for (const auto bound_state : state) {
      active_set.push_back(static_cast<int>(bound_state));
    }
    if (!visited_active_sets.insert(active_set).second) {
      return {x, false, 0, 0,
              "differential IK active-set cycle detected"};
    }

    std::vector<int> free_indices;
    free_indices.reserve(static_cast<std::size_t>(dimension));
    for (int index = 0; index < dimension; ++index) {
      if (state[static_cast<std::size_t>(index)] == BoundState::Free) {
        free_indices.push_back(index);
      }
    }

    Eigen::VectorXd candidate = x;
    if (!free_indices.empty()) {
      Eigen::MatrixXd free_hessian(free_indices.size(), free_indices.size());
      Eigen::VectorXd free_rhs(free_indices.size());
      for (std::size_t row = 0; row < free_indices.size(); ++row) {
        const int row_index = free_indices[row];
        free_rhs[static_cast<Eigen::Index>(row)] = -linear[row_index];
        for (int index = 0; index < dimension; ++index) {
          if (state[static_cast<std::size_t>(index)] != BoundState::Free) {
            free_rhs[static_cast<Eigen::Index>(row)] -=
                hessian(row_index, index) * x[index];
          }
        }
        for (std::size_t column = 0; column < free_indices.size(); ++column) {
          free_hessian(static_cast<Eigen::Index>(row),
                       static_cast<Eigen::Index>(column)) =
              hessian(row_index, free_indices[column]);
        }
      }
      try {
        const Eigen::VectorXd free_solution =
            solve_symmetric_system(free_hessian, free_rhs);
        for (std::size_t index = 0; index < free_indices.size(); ++index) {
          candidate[free_indices[index]] = free_solution[
              static_cast<Eigen::Index>(index)];
        }
      } catch (const std::exception &) {
        return {x, false, 0, 0,
                "differential IK free-set system is singular"};
      }
    }

    // If the free solution crosses a bound, move to the first boundary and
    // make that coordinate active.  This is the standard primal active-set
    // step and preserves feasibility throughout the iteration.
    double alpha = 1.0;
    int hit_index = -1;
    BoundState hit_state = BoundState::Free;
    const Eigen::VectorXd direction = candidate - x;
    for (const int index : free_indices) {
      const double d = direction[index];
      if (d < 0.0 && std::isfinite(lower[index]) &&
          candidate[index] < lower[index]) {
        const double ratio = (lower[index] - x[index]) / d;
        if (ratio >= 0.0 && ratio < alpha) {
          alpha = ratio;
          hit_index = index;
          hit_state = BoundState::Lower;
        }
      } else if (d > 0.0 && std::isfinite(upper[index]) &&
                 candidate[index] > upper[index]) {
        const double ratio = (upper[index] - x[index]) / d;
        if (ratio >= 0.0 && ratio < alpha) {
          alpha = ratio;
          hit_index = index;
          hit_state = BoundState::Upper;
        }
      }
    }
    if (hit_index >= 0) {
      x += alpha * direction;
      x[hit_index] = hit_state == BoundState::Lower ? lower[hit_index]
                                                    : upper[hit_index];
      state[static_cast<std::size_t>(hit_index)] = hit_state;
      continue;
    }
    x = candidate;
    bool feasible = true;
    for (int index = 0; index < dimension; ++index) {
      if (std::isfinite(lower[index]) && x[index] < lower[index]) {
        if (lower[index] - x[index] <= tolerance) {
          x[index] = lower[index];
        } else {
          feasible = false;
        }
      }
      if (std::isfinite(upper[index]) && x[index] > upper[index]) {
        if (x[index] - upper[index] <= tolerance) {
          x[index] = upper[index];
        } else {
          feasible = false;
        }
      }
    }
    if (!feasible) {
      return {x, false, 0, 0,
              "differential IK active-set produced an infeasible point"};
    }

    const Eigen::VectorXd gradient = hessian * x + linear;
    double largest_violation = tolerance;
    int release_index = -1;
    for (int index = 0; index < dimension; ++index) {
      const auto bound_state = state[static_cast<std::size_t>(index)];
      if (bound_state == BoundState::Lower && gradient[index] < -largest_violation) {
        largest_violation = -gradient[index];
        release_index = index;
      } else if (bound_state == BoundState::Upper &&
                 gradient[index] > largest_violation) {
        largest_violation = gradient[index];
        release_index = index;
      }
    }
    if (release_index >= 0) {
      state[static_cast<std::size_t>(release_index)] = BoundState::Free;
      continue;
    }

    int active_lower = 0;
    int active_upper = 0;
    for (const auto bound_state : state) {
      active_lower += bound_state == BoundState::Lower ? 1 : 0;
      active_upper += bound_state == BoundState::Upper ? 1 : 0;
    }
    return {x, true, active_lower, active_upper, "solved"};
  }

  int active_lower = 0;
  int active_upper = 0;
  for (const auto bound_state : state) {
    active_lower += bound_state == BoundState::Lower ? 1 : 0;
    active_upper += bound_state == BoundState::Upper ? 1 : 0;
  }
  return {x, false, active_lower, active_upper,
          "differential IK active-set iteration limit reached"};
}

std::vector<pinocchio::JointIndex>
finger_joint_ids(const pinocchio::Model &model) {
  std::vector<pinocchio::JointIndex> ids;
  for (const char *name : {"fr3_finger_joint1", "fr3_finger_joint2"}) {
    if (has_joint(model, name)) {
      ids.push_back(model.getJointId(name));
    }
  }
  return ids;
}

} // namespace

RobotModel::RobotModel(const std::string &urdf_path,
                       const std::string &end_effector_frame,
                       double finger_position)
    : urdf_path_(
          std::filesystem::absolute(urdf_path).lexically_normal().string()),
      end_effector_frame_(end_effector_frame),
      finger_position_(finger_position), data_(model_) {
  if (!std::filesystem::is_regular_file(urdf_path_)) {
    throw std::invalid_argument("URDF file does not exist: " + urdf_path_);
  }
  if (!std::isfinite(finger_position_) || finger_position_ < 0.0 ||
      finger_position_ > 0.04) {
    throw std::invalid_argument(
        "finger_position must be in [0.0, 0.04] meters");
  }

  pinocchio::Model full_model;
  pinocchio::urdf::buildModel(urdf_path_, full_model);

  const auto locked_joints = finger_joint_ids(full_model);
  if (locked_joints.size() != 2U) {
    throw std::runtime_error(
        "Expected official FR3 hand joints fr3_finger_joint1 and "
        "fr3_finger_joint2 in URDF");
  }

  Eigen::VectorXd reference = pinocchio::neutral(full_model);
  for (const auto joint_id : locked_joints) {
    const int idx_q = full_model.joints[joint_id].idx_q();
    if (idx_q < 0 || full_model.joints[joint_id].nq() != 1) {
      throw std::runtime_error(
          "FR3 finger joint has an unsupported configuration");
    }
    reference[idx_q] = finger_position_;
  }

  model_ = pinocchio::buildReducedModel(full_model, locked_joints, reference);
  data_ = pinocchio::Data(model_);

  if (model_.nq != 7 || model_.nv != 7) {
    std::ostringstream message;
    message << "Expected a 7-DoF FR3 arm after locking the hand, got nq="
            << model_.nq << " nv=" << model_.nv;
    throw std::runtime_error(message.str());
  }
  (void)resolve_frame(end_effector_frame_);
}

std::vector<std::string> RobotModel::joint_names() const {
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(model_.nq));
  for (pinocchio::JointIndex joint_id = 1; joint_id < model_.joints.size();
       ++joint_id) {
    if (model_.joints[joint_id].nq() > 0) {
      names.push_back(model_.names[joint_id]);
    }
  }
  return names;
}

std::vector<std::pair<double, double>> RobotModel::joint_limits() const {
  std::vector<std::pair<double, double>> limits;
  limits.reserve(static_cast<std::size_t>(model_.nq));
  for (int index = 0; index < model_.nq; ++index) {
    limits.emplace_back(model_.lowerPositionLimit[index],
                        model_.upperPositionLimit[index]);
  }
  return limits;
}

std::vector<std::string> RobotModel::frame_names() const {
  std::vector<std::string> names;
  names.reserve(model_.frames.size());
  for (const auto &frame : model_.frames) {
    names.push_back(frame.name);
  }
  return names;
}

Eigen::VectorXd RobotModel::home_configuration() const {
  Eigen::VectorXd q(7);
  q << 0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0, M_PI / 4.0;
  return q;
}

Eigen::VectorXd RobotModel::random_configuration(unsigned int seed) const {
  std::mt19937 generator(seed);
  Eigen::VectorXd q(model_.nq);
  for (int index = 0; index < model_.nq; ++index) {
    double lower = model_.lowerPositionLimit[index];
    double upper = model_.upperPositionLimit[index];
    if (!std::isfinite(lower)) {
      lower = -M_PI;
    }
    if (!std::isfinite(upper)) {
      upper = M_PI;
    }
    std::uniform_real_distribution<double> distribution(lower, upper);
    q[index] = distribution(generator);
  }
  return q;
}

void RobotModel::validate_configuration(const Eigen::VectorXd &q) const {
  if (q.size() != model_.nq) {
    std::ostringstream message;
    message << "Expected q with " << model_.nq << " values, got " << q.size();
    throw std::invalid_argument(message.str());
  }
  if (!q.array().isFinite().all()) {
    throw std::invalid_argument("q contains NaN or infinity");
  }
}

pinocchio::FrameIndex
RobotModel::resolve_frame(const std::string &frame_name) const {
  const std::string &requested =
      frame_name.empty() ? end_effector_frame_ : frame_name;
  for (pinocchio::FrameIndex index = 0; index < model_.frames.size(); ++index) {
    if (model_.frames[index].name == requested) {
      return index;
    }
  }
  throw std::invalid_argument("Unknown frame: " + requested);
}

Eigen::Matrix4d
RobotModel::forward_kinematics(const Eigen::VectorXd &q,
                               const std::string &frame_name) const {
  validate_configuration(q);
  const auto frame_id = resolve_frame(frame_name);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[frame_id].toHomogeneousMatrix();
}

Eigen::MatrixXd RobotModel::jacobian(const Eigen::VectorXd &q,
                                     const std::string &frame_name) const {
  validate_configuration(q);
  const auto frame_id = resolve_frame(frame_name);
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::Data::Matrix6x result(6, model_.nv);
  result.setZero();
  pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL,
                              result);
  return result;
}

std::map<std::string, Eigen::Matrix4d>
RobotModel::frame_placements(const Eigen::VectorXd &q) const {
  validate_configuration(q);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  std::map<std::string, Eigen::Matrix4d> placements;
  for (pinocchio::FrameIndex index = 0; index < model_.frames.size(); ++index) {
    if (model_.frames[index].type == pinocchio::BODY) {
      placements[model_.frames[index].name] =
          data_.oMf[index].toHomogeneousMatrix();
    }
  }
  return placements;
}

RobotModel::ErrorState
RobotModel::pose_error(const Eigen::VectorXd &q, const pinocchio::SE3 &target,
                       pinocchio::FrameIndex frame_id) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3 current_to_target =
      data_.oMf[frame_id].inverse() * target;

  ErrorState state;
  state.vector = pinocchio::log6(current_to_target).toVector();
  state.position_norm = state.vector.head<3>().norm();
  state.orientation_norm = state.vector.tail<3>().norm();
  state.norm = state.vector.norm();
  return state;
}

Eigen::VectorXd RobotModel::joint_velocity_limits() const {
  Eigen::VectorXd limits = model_.velocityLimit;
  if (limits.size() != model_.nv) {
    throw std::runtime_error("Pinocchio velocity limits have an unexpected size");
  }
  return limits;
}

Eigen::VectorXd RobotModel::integrate_configuration(
    const Eigen::VectorXd &q, const Eigen::VectorXd &delta_q) const {
  validate_configuration(q);
  if (delta_q.size() != model_.nv || !delta_q.array().isFinite().all()) {
    throw std::invalid_argument(
        "delta_q must be finite and have the model's nv entries");
  }
  const Eigen::VectorXd next_q = pinocchio::integrate(model_, q, delta_q);
  if (!next_q.array().isFinite().all()) {
    throw std::runtime_error("Pinocchio integration returned a non-finite q");
  }
  return next_q;
}

DifferentialIKResult RobotModel::differential_ik_step(
    const Eigen::VectorXd &q_input, const Eigen::Matrix4d &target_matrix,
    const DifferentialIKOptions &options,
  const std::string &frame_name) const {
  validate_configuration(q_input);
  validate_differential_options(options, model_.nq, model_.nv, false);

  const auto target = target_from_matrix(target_matrix);
  const auto frame_id = resolve_frame(frame_name);
  // The differential API follows Mink's configuration semantics: the caller
  // chooses whether position limits are active.  Do not silently clamp q or
  // the integrated result when that option is disabled.
  Eigen::VectorXd q = q_input;

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3 current_to_target = data_.oMf[frame_id].inverse() * target;
  const Eigen::Matrix<double, 6, 1> frame_error =
      pinocchio::log6(current_to_target).toVector();
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::Data::Matrix6x local_jacobian(6, model_.nv);
  local_jacobian.setZero();
  pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL,
                              local_jacobian);
  const Eigen::MatrixXd frame_jacobian =
      -pinocchio::Jlog6(current_to_target.inverse()) * local_jacobian;

  Eigen::Matrix<double, 6, 1> frame_cost;
  frame_cost.head<3>() = options.position_cost;
  frame_cost.tail<3>() = options.orientation_cost;
  const Eigen::MatrixXd weighted_frame_jacobian =
      frame_cost.asDiagonal() * frame_jacobian;
  const Eigen::Matrix<double, 6, 1> weighted_frame_error =
      frame_cost.cwiseProduct(options.frame_gain * frame_error);

  Eigen::MatrixXd hessian =
      weighted_frame_jacobian.transpose() * weighted_frame_jacobian;
  Eigen::VectorXd linear =
      weighted_frame_jacobian.transpose() * weighted_frame_error;
  double frame_mu =
      options.frame_lm_damping * weighted_frame_error.squaredNorm();
  hessian.diagonal().array() += options.damping + frame_mu;

  Eigen::VectorXd posture_target = options.posture_target;
  if (posture_target.size() == 0) {
    posture_target = home_configuration();
  }
  validate_configuration(posture_target);
  Eigen::VectorXd posture_costs =
      options.posture_costs.size() == 0
          ? Eigen::VectorXd::Constant(model_.nv, options.posture_cost)
          : options.posture_costs;
  if (posture_costs.maxCoeff() > 0.0) {
    // Pinocchio difference(q_target, q) is q - q_target for this fixed
    // revolute model, matching MuJoCo's mj_differentiatePos(target, q).
    const Eigen::VectorXd posture_error =
        pinocchio::difference(model_, posture_target, q);
    const Eigen::MatrixXd weighted_posture_jacobian =
        posture_costs.asDiagonal();
    const Eigen::VectorXd weighted_posture_error =
        posture_costs * (options.posture_gain * posture_error);
    hessian += weighted_posture_jacobian.transpose() *
               weighted_posture_jacobian;
    linear += weighted_posture_jacobian.transpose() * weighted_posture_error;
    const double posture_mu = options.posture_lm_damping *
                              weighted_posture_error.squaredNorm();
    hessian.diagonal().array() += posture_mu;
  }

  if (!hessian.array().isFinite().all() || !linear.array().isFinite().all()) {
    throw std::runtime_error("differential IK QP objective is non-finite");
  }

  Eigen::VectorXd lower = Eigen::VectorXd::Constant(
      model_.nv, -std::numeric_limits<double>::infinity());
  Eigen::VectorXd upper = Eigen::VectorXd::Constant(
      model_.nv, std::numeric_limits<double>::infinity());

  if (options.enforce_position_limits) {
    for (int index = 0; index < model_.nv; ++index) {
      const double q_min = model_.lowerPositionLimit[index];
      const double q_max = model_.upperPositionLimit[index];
      if (std::isfinite(q_min)) {
        lower[index] = std::max(lower[index],
                                options.position_limit_gain * (q_min - q[index]));
      }
      if (std::isfinite(q_max)) {
        upper[index] = std::min(upper[index],
                                options.position_limit_gain * (q_max - q[index]));
      }
    }
  }
  if (options.enforce_velocity_limits) {
    Eigen::VectorXd velocity_limits = options.velocity_limits;
    if (velocity_limits.size() == 0) {
      velocity_limits = joint_velocity_limits();
    }
    if (velocity_limits.size() != model_.nv ||
        !velocity_limits.array().isFinite().all() ||
        (velocity_limits.array() < 0.0).any()) {
      throw std::invalid_argument(
          "velocity_limits must be empty or finite non-negative nv values");
    }
    for (int index = 0; index < model_.nv; ++index) {
      lower[index] = std::max(lower[index], -velocity_limits[index] * options.dt);
      upper[index] = std::min(upper[index], velocity_limits[index] * options.dt);
    }
  }

  const BoxQPResult qp = solve_box_qp(
      hessian, linear, lower, upper, options.qp_max_active_sets,
      options.qp_tolerance);
  DifferentialIKResult result;
  result.delta_q = qp.success ? qp.x : Eigen::VectorXd::Zero(model_.nv);
  result.velocity = result.delta_q / options.dt;
  result.active_lower = qp.active_lower;
  result.active_upper = qp.active_upper;
  result.success = qp.success;
  result.status = qp.status;
  result.task_error = frame_error.norm();
  result.position_error = frame_error.head<3>().norm();
  result.orientation_error = frame_error.tail<3>().norm();
  if (qp.success) {
    result.next_q = integrate_configuration(q, result.delta_q);
    if (options.enforce_position_limits) {
      for (int index = 0; index < model_.nq; ++index) {
        const double lower_limit = model_.lowerPositionLimit[index];
        const double upper_limit = model_.upperPositionLimit[index];
        if ((std::isfinite(lower_limit) &&
             result.next_q[index] < lower_limit - options.qp_tolerance) ||
            (std::isfinite(upper_limit) &&
             result.next_q[index] > upper_limit + options.qp_tolerance)) {
          result.success = false;
          result.status = "integrated configuration violates position limits";
          result.delta_q.setZero();
          result.velocity.setZero();
          result.next_q = q;
          result.next_task_error = result.task_error;
          return result;
        }
      }
    }
    const ErrorState next_state = pose_error(result.next_q, target, frame_id);
    result.next_task_error = next_state.norm;
    result.objective =
        0.5 * result.delta_q.dot(hessian * result.delta_q) +
        linear.dot(result.delta_q);
  } else {
    result.next_q = q;
    result.next_task_error = result.task_error;
  }
  return result;
}

MinkIKResult RobotModel::mink_inverse_kinematics(
    const Eigen::Matrix4d &target_matrix, const Eigen::VectorXd &q_seed,
    const DifferentialIKOptions &options) const {
  validate_configuration(q_seed);
  validate_differential_options(options, model_.nq, model_.nv, true);
  const auto target = target_from_matrix(target_matrix);
  const auto frame_id = resolve_frame("");
  const bool posture_enabled =
      options.posture_gain > 0.0 &&
      (options.posture_cost > 0.0 ||
       (options.posture_costs.size() != 0 &&
        options.posture_costs.maxCoeff() > 0.0));
  Eigen::VectorXd q = q_seed;
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    const ErrorState state = pose_error(q, target, frame_id);
    // Mink keeps all soft tasks active in the same QP.  Therefore a frame
    // task that is already solved must not terminate the wrapper when a
    // PostureTask is enabled: subsequent differential steps are precisely
    // what moves the redundant elbow while balancing the frame residual.
    if (state.norm <= options.tolerance && !posture_enabled) {
      return MinkIKResult{q, true, iteration, state.norm,
                      state.position_norm, state.orientation_norm};
    }
    const DifferentialIKResult step =
        differential_ik_step(q, target_matrix, options);
    if (!step.success) {
      return MinkIKResult{q, false, iteration + 1, state.norm,
                      state.position_norm, state.orientation_norm};
    }
    q = step.next_q;
    if (step.delta_q.norm() <= options.qp_tolerance) {
      const ErrorState next_state = pose_error(q, target, frame_id);
      return MinkIKResult{q,
                      next_state.norm <= options.tolerance,
                      iteration + 1,
                      next_state.norm,
                      next_state.position_norm,
                      next_state.orientation_norm};
    }
  }
  const ErrorState state = pose_error(q, target, frame_id);
  return MinkIKResult{q, state.norm <= options.tolerance,
                  options.max_iterations, state.norm, state.position_norm,
                  state.orientation_norm};
}

Eigen::MatrixXd
RobotModel::minimum_jerk_trajectory(const Eigen::VectorXd &q_start,
                                    const Eigen::VectorXd &q_goal,
                                    double duration, double dt) const {
  validate_configuration(q_start);
  validate_configuration(q_goal);
  if (!std::isfinite(duration) || !std::isfinite(dt) || duration <= 0.0 ||
      dt <= 0.0) {
    throw std::invalid_argument("duration and dt must be finite and positive");
  }

  const int intervals = std::max(1, static_cast<int>(std::ceil(duration / dt)));
  Eigen::MatrixXd trajectory(intervals + 1, model_.nq);
  for (int index = 0; index <= intervals; ++index) {
    const double s = static_cast<double>(index) / intervals;
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double blend = 10.0 * s3 - 15.0 * s3 * s + 6.0 * s3 * s2;
    trajectory.row(index) = (q_start + blend * (q_goal - q_start)).transpose();
  }
  return trajectory;
}

Eigen::Matrix4d pose_from_xyz_rpy(const Eigen::Vector3d &xyz,
                                  const Eigen::Vector3d &rpy) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.topLeftCorner<3, 3>() = pinocchio::rpy::rpyToMatrix(rpy);
  pose.topRightCorner<3, 1>() = xyz;
  return pose;
}

Eigen::Vector3d rpy_from_pose(const Eigen::Matrix4d &pose) {
  if (!is_finite_matrix(pose)) {
    throw std::invalid_argument("pose contains NaN or infinity");
  }
  return pinocchio::rpy::matrixToRpy(pose.topLeftCorner<3, 3>());
}

} // namespace fr3_control_sim
