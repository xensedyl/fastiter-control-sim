#include "fr3_control_sim/robot_model.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using fr3_control_sim::DifferentialIKOptions;
using fr3_control_sim::DifferentialIKResult;
using fr3_control_sim::MinkIKResult;
using fr3_control_sim::RobotModel;

PYBIND11_MODULE(_fr3_sim, module) {
  module.doc() =
      "FR3 FK and Mink-style differential IK implemented in C++ with Pinocchio";

  py::class_<DifferentialIKOptions>(module, "DifferentialIKOptions")
      .def(py::init<>())
      .def_readwrite("dt", &DifferentialIKOptions::dt)
      .def_readwrite("damping", &DifferentialIKOptions::damping)
      .def_readwrite("qp_max_active_sets",
                     &DifferentialIKOptions::qp_max_active_sets)
      .def_readwrite("qp_tolerance", &DifferentialIKOptions::qp_tolerance)
      .def_readwrite("frame_gain", &DifferentialIKOptions::frame_gain)
      .def_readwrite("position_cost", &DifferentialIKOptions::position_cost)
      .def_readwrite("orientation_cost",
                     &DifferentialIKOptions::orientation_cost)
      .def_readwrite("frame_lm_damping",
                     &DifferentialIKOptions::frame_lm_damping)
      .def_readwrite("posture_cost", &DifferentialIKOptions::posture_cost)
      .def_readwrite("posture_costs", &DifferentialIKOptions::posture_costs)
      .def_readwrite("posture_gain", &DifferentialIKOptions::posture_gain)
      .def_readwrite("posture_lm_damping",
                     &DifferentialIKOptions::posture_lm_damping)
      .def_readwrite("posture_target",
                     &DifferentialIKOptions::posture_target)
      .def_readwrite("enforce_position_limits",
                     &DifferentialIKOptions::enforce_position_limits)
      .def_readwrite("position_limit_gain",
                     &DifferentialIKOptions::position_limit_gain)
      .def_readwrite("enforce_velocity_limits",
                     &DifferentialIKOptions::enforce_velocity_limits)
      .def_readwrite("velocity_limits",
                     &DifferentialIKOptions::velocity_limits)
      .def_readwrite("max_iterations", &DifferentialIKOptions::max_iterations)
      .def_readwrite("tolerance", &DifferentialIKOptions::tolerance)
      .def("__repr__", [](const DifferentialIKOptions &options) {
        return "DifferentialIKOptions(dt=" + std::to_string(options.dt) +
               ", damping=" + std::to_string(options.damping) +
               ", frame_gain=" + std::to_string(options.frame_gain) +
               ", posture_cost=" + std::to_string(options.posture_cost) +
               ")";
      });

  py::class_<DifferentialIKResult>(module, "DifferentialIKResult")
      .def_readonly("delta_q", &DifferentialIKResult::delta_q)
      .def_readonly("velocity", &DifferentialIKResult::velocity)
      .def_readonly("next_q", &DifferentialIKResult::next_q)
      .def_readonly("success", &DifferentialIKResult::success)
      .def_readonly("objective", &DifferentialIKResult::objective)
      .def_readonly("task_error", &DifferentialIKResult::task_error)
      .def_readonly("next_task_error", &DifferentialIKResult::next_task_error)
      .def_readonly("position_error", &DifferentialIKResult::position_error)
      .def_readonly("orientation_error",
                    &DifferentialIKResult::orientation_error)
      .def_readonly("active_lower", &DifferentialIKResult::active_lower)
      .def_readonly("active_upper", &DifferentialIKResult::active_upper)
      .def_readonly("status", &DifferentialIKResult::status)
      .def("__repr__", [](const DifferentialIKResult &result) {
        return "DifferentialIKResult(success=" +
               std::string(result.success ? "True" : "False") +
               ", task_error=" + std::to_string(result.task_error) +
               ", next_task_error=" +
               std::to_string(result.next_task_error) + ", status='" +
               result.status + "')";
      });

  py::class_<MinkIKResult>(module, "MinkIKResult")
      .def_readonly("q", &MinkIKResult::q)
      .def_readonly("success", &MinkIKResult::success)
      .def_readonly("iterations", &MinkIKResult::iterations)
      .def_readonly("error", &MinkIKResult::error)
      .def_readonly("position_error", &MinkIKResult::position_error)
      .def_readonly("orientation_error", &MinkIKResult::orientation_error)
      .def("__repr__", [](const MinkIKResult &result) {
        return "MinkIKResult(success=" +
               std::string(result.success ? "True" : "False") +
               ", error=" + std::to_string(result.error) +
               ", iterations=" + std::to_string(result.iterations) + ")";
      });

  py::class_<RobotModel>(module, "RobotModel")
      .def(py::init<const std::string &, const std::string &, double>(),
           py::arg("urdf_path"), py::arg("end_effector_frame") = "fr3_hand_tcp",
           py::arg("finger_position") = 0.02)
      .def_property_readonly("nq", &RobotModel::nq)
      .def_property_readonly("nv", &RobotModel::nv)
      .def_property_readonly("urdf_path", &RobotModel::urdf_path)
      .def_property_readonly("end_effector_frame",
                             &RobotModel::end_effector_frame)
      .def_property_readonly("finger_position", &RobotModel::finger_position)
      .def_property_readonly("joint_names", &RobotModel::joint_names)
      .def_property_readonly("joint_limits", &RobotModel::joint_limits)
      .def_property_readonly("frame_names", &RobotModel::frame_names)
      .def("home_configuration", &RobotModel::home_configuration)
      .def("random_configuration", &RobotModel::random_configuration,
           py::arg("seed") = 42)
      .def("forward_kinematics", &RobotModel::forward_kinematics, py::arg("q"),
           py::arg("frame_name") = "")
      .def("jacobian", &RobotModel::jacobian, py::arg("q"),
           py::arg("frame_name") = "")
      .def("frame_placements", &RobotModel::frame_placements, py::arg("q"))
      .def("differential_ik_step", &RobotModel::differential_ik_step,
           py::arg("q"), py::arg("target"),
           py::arg("options") = DifferentialIKOptions(),
           py::arg("frame_name") = "")
      .def("mink_inverse_kinematics", &RobotModel::mink_inverse_kinematics,
           py::arg("target"), py::arg("q_seed"),
           py::arg("options") = DifferentialIKOptions())
      .def("integrate_configuration", &RobotModel::integrate_configuration,
           py::arg("q"), py::arg("delta_q"))
      .def("joint_velocity_limits", &RobotModel::joint_velocity_limits)
      .def("minimum_jerk_trajectory", &RobotModel::minimum_jerk_trajectory,
           py::arg("q_start"), py::arg("q_goal"), py::arg("duration"),
           py::arg("dt") = 0.02);

  module.def("pose_from_xyz_rpy", &fr3_control_sim::pose_from_xyz_rpy,
             py::arg("xyz"), py::arg("rpy"));
  module.def("rpy_from_pose", &fr3_control_sim::rpy_from_pose, py::arg("pose"));
}
