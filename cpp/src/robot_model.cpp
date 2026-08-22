#include "fr3_control_sim/robot_model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <urdf_parser/urdf_parser.h>

#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>

namespace fr3_control_sim {
namespace {

bool is_finite_matrix(const Eigen::Matrix4d &matrix) {
  return matrix.array().isFinite().all();
}

bool has_joint(const pinocchio::Model &model, const std::string &name) {
  return model.existJointName(name);
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

bool has_non_finger_mimic_joint(const std::string &urdf_path) {
  const auto urdf = urdf::parseURDFFile(urdf_path);
  if (!urdf) {
    throw std::invalid_argument("Could not parse URDF: " + urdf_path);
  }
  for (const auto &entry : urdf->joints_) {
    const auto &joint = entry.second;
    if (joint && joint->mimic && entry.first != "fr3_finger_joint1" &&
        entry.first != "fr3_finger_joint2") {
      return true;
    }
  }
  return false;
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
  if (!std::isfinite(finger_position_)) {
    throw std::invalid_argument("finger_position must be finite");
  }

  pinocchio::Model full_model;
  // Parse non-finger mimic tags. The 0820 export contains two visible wrist
  // joints (`joint_5-2` and `joint_6`) that are mechanically coupled to
  // `joint_5-1`; parsing with mimic=true keeps those branches in the
  // kinematic tree while exposing only the seven independent variables. The
  // official model's finger mimic is intentionally parsed as ordinary joints
  // and then reduced below, because locking its directing joint would leave a
  // dangling mimic index in Pinocchio.
  const bool parse_mimic = has_non_finger_mimic_joint(urdf_path_);
  pinocchio::urdf::buildModel(urdf_path_, full_model, false, parse_mimic);

  const auto finger_joints = finger_joint_ids(full_model);
  if (finger_joints.size() == 1U) {
    throw std::runtime_error(
        "URDF contains only one FR3 finger joint; expected both or neither");
  }

  std::vector<pinocchio::JointIndex> independent_finger_joints;
  for (const auto joint_id : finger_joints) {
    if (full_model.joints[joint_id].nq() > 0) {
      independent_finger_joints.push_back(joint_id);
    }
  }

  if (!finger_joints.empty() && independent_finger_joints.empty()) {
    throw std::runtime_error("FR3 finger joints have no independent configuration");
  }

  if (!independent_finger_joints.empty()) {
    if (finger_position_ < 0.0 || finger_position_ > 0.04) {
      throw std::invalid_argument(
          "finger_position must be in [0.0, 0.04] meters for the FR3 hand");
    }
    Eigen::VectorXd reference = pinocchio::neutral(full_model);
    for (const auto joint_id : independent_finger_joints) {
      const int idx_q = full_model.joints[joint_id].idx_q();
      if (idx_q < 0 || full_model.joints[joint_id].nq() != 1) {
        throw std::runtime_error(
            "FR3 finger joint has an unsupported configuration");
      }
      reference[idx_q] = finger_position_;
    }
    model_ = pinocchio::buildReducedModel(full_model, independent_finger_joints,
                                          reference);
  } else {
    model_ = std::move(full_model);
  }

  data_ = pinocchio::Data(model_);

  if (model_.nq != 7 || model_.nv != 7) {
    std::ostringstream message;
    message << "Expected a 7-DoF serial arm after optionally locking the hand, "
               "got nq="
            << model_.nq << " nv=" << model_.nv;
    throw std::runtime_error(message.str());
  }

  if (end_effector_frame_.empty()) {
    for (const char *candidate : {"fr3_hand_tcp", "link_7"}) {
      for (const auto &frame : model_.frames) {
        if (frame.name == candidate) {
          end_effector_frame_ = candidate;
          break;
        }
      }
      if (!end_effector_frame_.empty()) {
        break;
      }
    }
    if (end_effector_frame_.empty()) {
      for (auto frame = model_.frames.rbegin(); frame != model_.frames.rend();
           ++frame) {
        if (frame->type == pinocchio::BODY) {
          end_effector_frame_ = frame->name;
          break;
        }
      }
    }
    if (end_effector_frame_.empty()) {
      throw std::runtime_error("Could not infer an end-effector frame from URDF");
    }
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

std::vector<std::string> RobotModel::mimic_joint_names() const {
  std::vector<std::string> names;
  names.reserve(model_.mimicking_joints.size());
  for (const auto joint_id : model_.mimicking_joints) {
    if (joint_id < model_.names.size()) {
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
  const bool is_official_fr3 =
      model_.existJointName("fr3_joint1") &&
      model_.existJointName("fr3_joint2") &&
      model_.existJointName("fr3_joint3") &&
      model_.existJointName("fr3_joint4") &&
      model_.existJointName("fr3_joint5") &&
      model_.existJointName("fr3_joint6") &&
      model_.existJointName("fr3_joint7");
  const bool is_cad_fr3 =
      model_.existJointName("joint_1") &&
      model_.existJointName("joint_2") && model_.existJointName("joint_3") &&
      model_.existJointName("joint_4") && model_.existJointName("joint_5") &&
      model_.existJointName("joint_6") && model_.existJointName("joint_7");
  const bool has_mimic_wrist = !model_.mimicking_joints.empty();

  Eigen::VectorXd q = pinocchio::neutral(model_);
  if (is_official_fr3) {
    q << 0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0,
        M_PI / 2.0, M_PI / 4.0;
    return clamp_configuration(q);
  }
  if (is_cad_fr3) {
    // The original CAD export has an independent joint_6.  The 0820 export
    // instead has joint_5-1 as the independent coordinate and both
    // joint_5-2 and joint_6 as mimic branches, whose valid range is only
    // +/-0.8 rad. Keep that wrist at its neutral coupled position rather
    // than silently clamping the old +/-90 degree ready-pose value.
    q << 0.0, -M_PI / 4.0, 0.0, 3.0 * M_PI / 4.0, 0.0,
        (has_mimic_wrist ? 0.0 : -M_PI / 2.0), -M_PI / 4.0;
    return clamp_configuration(q);
  }

  // For an arbitrary 7-axis URDF, keep the initial pose away from joint
  // limits. This is a better IK seed than Pinocchio's all-zero neutral pose
  // when zero lies directly on a limit (as it does for the CAD-exported
  // joint_4 and joint_6).
  for (int index = 0; index < model_.nq; ++index) {
    const double lower = model_.lowerPositionLimit[index];
    const double upper = model_.upperPositionLimit[index];
    if (std::isfinite(lower) && std::isfinite(upper)) {
      q[index] = 0.5 * (lower + upper);
    }
  }
  return clamp_configuration(q);
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

Eigen::VectorXd
RobotModel::clamp_configuration(const Eigen::VectorXd &q) const {
  validate_configuration(q);
  Eigen::VectorXd clamped = q;
  for (int index = 0; index < model_.nq; ++index) {
    const double lower = model_.lowerPositionLimit[index];
    const double upper = model_.upperPositionLimit[index];
    if (std::isfinite(lower)) {
      clamped[index] = std::max(clamped[index], lower);
    }
    if (std::isfinite(upper)) {
      clamped[index] = std::min(clamped[index], upper);
    }
  }
  return clamped;
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

IKResult RobotModel::inverse_kinematics_once(const pinocchio::SE3 &target,
                                             const Eigen::VectorXd &q_seed,
                                             pinocchio::FrameIndex frame_id,
                                             const IKOptions &options) const {
  Eigen::VectorXd q = clamp_configuration(q_seed);
  const Eigen::VectorXd home = home_configuration();
  ErrorState state = pose_error(q, target, frame_id);
  int stalled_iterations = 0;
  constexpr double posture_tolerance = 1e-4;

  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    const Eigen::VectorXd posture_error = pinocchio::difference(model_, q, home);
    if (state.norm <= options.tolerance &&
        (options.posture_gain <= 0.0 ||
         posture_error.norm() <= posture_tolerance)) {
      return IKResult{q,
                      true,
                      iteration,
                      1,
                      state.norm,
                      state.position_norm,
                      state.orientation_norm};
    }

    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::Data::Matrix6x jacobian_matrix(6, model_.nv);
    jacobian_matrix.setZero();
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL,
                                jacobian_matrix);

    // Differentiate log6(current^-1 * target) exactly, following the
    // Pinocchio closed-loop IK formulation.  This is noticeably more robust
    // than using the raw LOCAL Jacobian for targets far from the seed.
    const pinocchio::SE3 current_to_target =
        data_.oMf[frame_id].inverse() * target;
    const Eigen::MatrixXd task_jacobian =
        -pinocchio::Jlog6(current_to_target.inverse()) * jacobian_matrix;

    Eigen::Matrix<double, 6, 6> normal =
        task_jacobian * task_jacobian.transpose();
    const double adaptive_damping =
        options.damping * std::max(1.0, 10.0 * state.norm);
    normal.diagonal().array() += adaptive_damping;
    const Eigen::Matrix<double, 6, 6> normal_inverse =
        normal.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
    const Eigen::MatrixXd damped_pseudoinverse =
        task_jacobian.transpose() * normal_inverse;
    Eigen::VectorXd delta =
        -options.step_size * damped_pseudoinverse * state.vector;

    // FR3 has seven arm joints for a six-dimensional end-effector task.  Use
    // the remaining null-space motion to prefer the standard ready/home
    // posture without changing the first-order Cartesian task motion.
    if (options.posture_gain > 0.0 && state.norm <= options.tolerance) {
      Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition(
          task_jacobian);
      decomposition.setThreshold(1e-8);
      const Eigen::MatrixXd exact_pseudoinverse = decomposition.pseudoInverse();
      const Eigen::MatrixXd nullspace_projector =
          Eigen::MatrixXd::Identity(model_.nv, model_.nv) -
          exact_pseudoinverse * task_jacobian;
      delta += options.posture_gain * nullspace_projector * posture_error;
    }

    const double delta_norm = delta.norm();
    if (options.max_step_norm > 0.0 && delta_norm > options.max_step_norm) {
      delta *= options.max_step_norm / delta_norm;
    }

    bool improved = false;
    double alpha = 1.0;
    for (int search = 0; search < options.line_search_steps; ++search) {
      Eigen::VectorXd candidate =
          pinocchio::integrate(model_, q, alpha * delta);
      candidate = clamp_configuration(candidate);
      const ErrorState candidate_state =
          pose_error(candidate, target, frame_id);
      const double candidate_posture_error =
          pinocchio::difference(model_, candidate, home).norm();
      const bool task_improved = candidate_state.norm + 1e-12 < state.norm;
      const bool posture_improved_at_solution =
          options.posture_gain > 0.0 && state.norm <= options.tolerance &&
          candidate_state.norm <= options.tolerance &&
          candidate_posture_error + 1e-10 < posture_error.norm();
      if (task_improved || posture_improved_at_solution) {
        q = candidate;
        state = candidate_state;
        improved = true;
        break;
      }
      alpha *= 0.5;
    }

    if (improved) {
      stalled_iterations = 0;
    } else {
      ++stalled_iterations;
      if (stalled_iterations >= 10) {
        return IKResult{q,
                        state.norm <= options.tolerance,
                        iteration + 1,
                        1,
                        state.norm,
                        state.position_norm,
                        state.orientation_norm};
      }
    }
  }

  return IKResult{q,
                  state.norm <= options.tolerance,
                  options.max_iterations,
                  1,
                  state.norm,
                  state.position_norm,
                  state.orientation_norm};
}

IKResult RobotModel::inverse_kinematics(const Eigen::Matrix4d &target_matrix,
                                        const Eigen::VectorXd &q_seed,
                                        const IKOptions &options) const {
  validate_configuration(q_seed);
  if (!is_finite_matrix(target_matrix)) {
    throw std::invalid_argument("target contains NaN or infinity");
  }
  const Eigen::RowVector4d homogeneous_row(0.0, 0.0, 0.0, 1.0);
  if (!target_matrix.row(3).isApprox(homogeneous_row, 1e-8)) {
    throw std::invalid_argument("target last row must be [0, 0, 0, 1]");
  }
  if (options.max_iterations <= 0 || options.max_retries < 0 ||
      options.tolerance <= 0.0 || options.damping < 0.0 ||
      options.step_size <= 0.0 || options.max_step_norm < 0.0 ||
      !std::isfinite(options.posture_gain) || options.posture_gain < 0.0 ||
      options.line_search_steps <= 0) {
    throw std::invalid_argument("Invalid IK options");
  }

  Eigen::Matrix3d rotation = target_matrix.topLeftCorner<3, 3>();
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
  const pinocchio::SE3 target(quaternion.toRotationMatrix(),
                              target_matrix.topRightCorner<3, 1>());
  const auto frame_id = resolve_frame(end_effector_frame_);

  IKResult best = inverse_kinematics_once(target, q_seed, frame_id, options);
  best.attempts = 1;
  if (best.success) {
    return best;
  }

  for (int retry = 0; retry < options.max_retries; ++retry) {
    const Eigen::VectorXd retry_seed = random_configuration(
        options.random_seed + static_cast<unsigned int>(retry));
    IKResult candidate =
        inverse_kinematics_once(target, retry_seed, frame_id, options);
    candidate.attempts = retry + 2;
    if (candidate.error < best.error) {
      best = candidate;
    }
    if (candidate.success) {
      return candidate;
    }
  }
  best.attempts = options.max_retries + 1;
  return best;
}

FrankaWeightedIKResult RobotModel::franka_weighted_ik(
    const Eigen::Matrix4d &target_matrix, const Eigen::VectorXd &q_seed,
    const FrankaWeightedIKOptions &options,
    const std::string &frame_name) const {
  validate_configuration(q_seed);
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
  if (options.samples < 2 || options.max_iterations <= 0 ||
      !std::isfinite(options.tolerance) || options.tolerance <= 0.0 ||
      !std::isfinite(options.damping) || options.damping < 0.0 ||
      !std::isfinite(options.step_size) || options.step_size <= 0.0 ||
      !std::isfinite(options.max_step_norm) || options.max_step_norm < 0.0) {
    throw std::invalid_argument("Invalid FrankaWeightedIKOptions");
  }
  for (const double weight : {options.weight_manipulability,
                              options.weight_neutral,
                              options.weight_current}) {
    if (!std::isfinite(weight) || weight < 0.0) {
      throw std::invalid_argument(
          "Franka weighted IK weights must be finite and non-negative");
    }
  }
  if ((options.neutral_q.size() != 0 &&
       options.neutral_q.size() != model_.nq) ||
      (options.joint_weights.size() != 0 &&
       options.joint_weights.size() != model_.nq)) {
    throw std::invalid_argument(
        "neutral_q and joint_weights must be empty or have nq values");
  }
  if (options.neutral_q.size() != 0 &&
      !options.neutral_q.array().isFinite().all()) {
    throw std::invalid_argument("neutral_q contains NaN or infinity");
  }
  if (options.joint_weights.size() != 0) {
    if (!options.joint_weights.array().isFinite().all() ||
        (options.joint_weights.array() < 0.0).any()) {
      throw std::invalid_argument(
          "joint_weights must be finite and non-negative");
    }
  }

  const auto frame_id = resolve_frame(frame_name);
  const pinocchio::SE3 target(
      rotation, target_matrix.topRightCorner<3, 1>());
  const Eigen::VectorXd q_current = clamp_configuration(q_seed);
  const Eigen::VectorXd neutral =
      options.neutral_q.size() == 0 ? home_configuration()
                                    : clamp_configuration(options.neutral_q);
  const Eigen::VectorXd joint_weights =
      options.joint_weights.size() == 0
          ? Eigen::VectorXd::Ones(model_.nq)
          : options.joint_weights;

  // The original LeFranX solver uses q7 as its one-dimensional free
  // variable.  For a generic URDF adapter, prefer a joint named joint_7 and
  // otherwise fall back to the final independent coordinate.
  int free_index = options.free_joint_index;
  if (free_index < 0) {
    const auto names = joint_names();
    for (int index = 0; index < static_cast<int>(names.size()); ++index) {
      if (names[static_cast<std::size_t>(index)] == "joint_7" ||
          names[static_cast<std::size_t>(index)] == "fr3_joint7") {
        free_index = index;
        break;
      }
    }
    if (free_index < 0) {
      free_index = model_.nq - 1;
    }
  }
  if (free_index < 0 || free_index >= model_.nq) {
    throw std::invalid_argument("free_joint_index is outside q dimensions");
  }

  auto finite_limit = [&](double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
  };
  double free_lower = finite_limit(model_.lowerPositionLimit[free_index],
                                   -M_PI);
  double free_upper = finite_limit(model_.upperPositionLimit[free_index], M_PI);
  if (!(free_upper > free_lower)) {
    throw std::invalid_argument("free joint has invalid position limits");
  }

  FrankaWeightedIKResult result;
  result.q = q_current;
  result.free_joint_index = free_index;
  const ErrorState initial_state = pose_error(q_current, target, frame_id);
  result.error = initial_state.norm;
  result.position_error = initial_state.position_norm;
  result.orientation_error = initial_state.orientation_norm;

  auto normalized_distance = [&](const Eigen::VectorXd &a,
                                 const Eigen::VectorXd &b,
                                 bool weighted) {
    const Eigen::VectorXd difference = pinocchio::difference(model_, a, b);
    double squared = 0.0;
    for (int index = 0; index < model_.nq; ++index) {
      double lower = model_.lowerPositionLimit[index];
      double upper = model_.upperPositionLimit[index];
      double range = upper - lower;
      if (!std::isfinite(range) || range <= 1e-9) {
        range = 2.0 * M_PI;
      }
      const double weight = weighted ? joint_weights[index] : 1.0;
      squared += weight * std::pow(difference[index] / range, 2.0);
    }
    return std::sqrt(std::max(0.0, squared));
  };

  auto score_candidate = [&](const Eigen::VectorXd &candidate,
                             double &manipulability,
                             double &neutral_distance,
                             double &current_distance) {
    const Eigen::MatrixXd candidate_jacobian = jacobian(candidate, frame_name);
    const Eigen::MatrixXd jjt =
        candidate_jacobian * candidate_jacobian.transpose();
    const double determinant = jjt.determinant();
    manipulability = determinant > 0.0 ? std::sqrt(determinant) : 0.0;
    neutral_distance = normalized_distance(candidate, neutral, false);
    current_distance = normalized_distance(candidate, q_current, true);
    return options.weight_manipulability * manipulability -
           options.weight_neutral * neutral_distance -
           options.weight_current * current_distance;
  };

  auto solve_fixed_free = [&](const Eigen::VectorXd &seed, double free_value,
                              Eigen::VectorXd &solved, ErrorState &solved_state,
                              int &iterations) {
    Eigen::VectorXd q = clamp_configuration(seed);
    q[free_index] = std::clamp(free_value, free_lower, free_upper);
    ErrorState state = pose_error(q, target, frame_id);
    iterations = 0;
    int stalled = 0;
    for (; iterations < options.max_iterations; ++iterations) {
      if (state.norm <= options.tolerance) {
        solved = q;
        solved_state = state;
        return true;
      }

      pinocchio::computeJointJacobians(model_, data_, q);
      pinocchio::updateFramePlacements(model_, data_);
      pinocchio::Data::Matrix6x local_jacobian(6, model_.nv);
      local_jacobian.setZero();
      pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL,
                                  local_jacobian);
      const pinocchio::SE3 current_to_target =
          data_.oMf[frame_id].inverse() * target;
      const Eigen::MatrixXd task_jacobian =
          -pinocchio::Jlog6(current_to_target.inverse()) * local_jacobian;

      std::vector<int> free_columns;
      free_columns.reserve(static_cast<std::size_t>(model_.nv - 1));
      for (int index = 0; index < model_.nv; ++index) {
        if (index != free_index) {
          free_columns.push_back(index);
        }
      }
      Eigen::MatrixXd reduced_jacobian(6, model_.nv - 1);
      for (int column = 0; column < reduced_jacobian.cols(); ++column) {
        reduced_jacobian.col(column) =
            task_jacobian.col(free_columns[static_cast<std::size_t>(column)]);
      }
      Eigen::MatrixXd normal =
          reduced_jacobian * reduced_jacobian.transpose();
      normal.diagonal().array() += options.damping;
      Eigen::VectorXd reduced_delta =
          -options.step_size * reduced_jacobian.transpose() *
          normal.ldlt().solve(state.vector);
      if (!reduced_delta.array().isFinite().all()) {
        break;
      }

      Eigen::VectorXd delta = Eigen::VectorXd::Zero(model_.nv);
      for (int column = 0; column < reduced_delta.size(); ++column) {
        delta[free_columns[static_cast<std::size_t>(column)]] =
            reduced_delta[column];
      }
      const double delta_norm = delta.norm();
      if (options.max_step_norm > 0.0 && delta_norm > options.max_step_norm) {
        delta *= options.max_step_norm / delta_norm;
      }

      bool improved = false;
      double alpha = 1.0;
      for (int line = 0; line < 10; ++line) {
        Eigen::VectorXd candidate =
            pinocchio::integrate(model_, q, alpha * delta);
        candidate = clamp_configuration(candidate);
        candidate[free_index] = std::clamp(free_value, free_lower, free_upper);
        const ErrorState candidate_state =
            pose_error(candidate, target, frame_id);
        if (candidate_state.norm + 1e-12 < state.norm) {
          q = candidate;
          state = candidate_state;
          improved = true;
          break;
        }
        alpha *= 0.5;
      }
      if (!improved) {
        ++stalled;
        if (stalled >= 3) {
          break;
        }
      } else {
        stalled = 0;
      }
    }
    solved = q;
    solved_state = state;
    return state.norm <= options.tolerance;
  };

  std::vector<double> free_values;
  free_values.reserve(static_cast<std::size_t>(options.samples) + 1U);
  for (int sample = 0; sample < options.samples; ++sample) {
    const double fraction = static_cast<double>(sample) /
                            static_cast<double>(options.samples - 1);
    free_values.push_back(free_lower + fraction * (free_upper - free_lower));
  }
  free_values.push_back(std::clamp(q_current[free_index], free_lower, free_upper));
  std::sort(free_values.begin(), free_values.end());
  free_values.erase(
      std::unique(free_values.begin(), free_values.end(),
                  [](double a, double b) { return std::abs(a - b) < 1e-12; }),
      free_values.end());

  const Eigen::VectorXd home = home_configuration();
  for (const double free_value : free_values) {
    ++result.samples_tested;
    std::vector<Eigen::VectorXd> seeds;
    seeds.push_back(q_current);
    if ((home - q_current).norm() > 1e-9) {
      seeds.push_back(home);
    }
    for (const Eigen::VectorXd &seed : seeds) {
      Eigen::VectorXd candidate;
      ErrorState candidate_state;
      int candidate_iterations = 0;
      if (!solve_fixed_free(seed, free_value, candidate, candidate_state,
                            candidate_iterations)) {
        continue;
      }
      ++result.valid_solutions;
      double manipulability = 0.0;
      double neutral_distance = 0.0;
      double current_distance = 0.0;
      const double score = score_candidate(
          candidate, manipulability, neutral_distance, current_distance);
      if (!std::isfinite(score)) {
        continue;
      }
      if (!result.success || score > result.score) {
        result.success = true;
        result.q = candidate;
        result.score = score;
        result.manipulability = manipulability;
        result.neutral_distance = neutral_distance;
        result.current_distance = current_distance;
        result.error = candidate_state.norm;
        result.position_error = candidate_state.position_norm;
        result.orientation_error = candidate_state.orientation_norm;
        result.iterations = candidate_iterations;
      }
    }
  }
  return result;
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
