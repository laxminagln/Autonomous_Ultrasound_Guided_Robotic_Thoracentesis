#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>

namespace panda_lung_scan_noetic {

class PandaUltrasoundForceScanController : public controller_interface::MultiInterfaceController<
                                              franka_hw::FrankaModelInterface,
                                              franka_hw::FrankaStateInterface,
                                              hardware_interface::EffortJointInterface> {
 public:
  PandaUltrasoundForceScanController() = default;
  ~PandaUltrasoundForceScanController() override = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override {
    node_handle_ = node_handle;

    if (!node_handle_.getParam("arm_id", arm_id_)) {
      ROS_ERROR("PandaUltrasoundForceScanController: Could not read parameter arm_id");
      return false;
    }

    if (!node_handle_.getParam("joint_names", joint_names_) || joint_names_.size() != 7) {
      ROS_ERROR("PandaUltrasoundForceScanController: Invalid or no joint_names parameters provided");
      return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();

    if (model_interface == nullptr || state_interface == nullptr || effort_joint_interface == nullptr) {
      ROS_ERROR("PandaUltrasoundForceScanController: Failed to get required interfaces from RobotHW");
      return false;
    }

    try {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id_ + "_model"));
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id_ + "_robot"));

      for (size_t i = 0; i < 7; ++i) {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names_[i]));
      }
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("PandaUltrasoundForceScanController: Exception getting interfaces: " << e.what());
      return false;
    }

    // -------- Parameters --------
    node_handle_.param("scan_axis", scan_axis_, 1);
    node_handle_.param("lateral_axis", lateral_axis_, 0);
    node_handle_.param("contact_axis", contact_axis_, 2);

    node_handle_.param("scan_length", scan_length_, 0.145);
    node_handle_.param("scan_width", scan_width_, 0.105);
    node_handle_.param("num_passes", num_passes_, 4);
    node_handle_.param("scan_speed", scan_speed_, 0.003);
    node_handle_.param("pass_coverage_ratio", pass_coverage_ratio_, 0.75);

    node_handle_.param("curve_half_width", curve_half_width_, 0.0525);
    node_handle_.param("curve_drop", curve_drop_, 0.010);
    node_handle_.param("contact_offset", contact_offset_, 0.0);
    node_handle_.param("lateral_offset", lateral_offset_, 0.0);

    node_handle_.param("settle_time", settle_time_, 1.0);
    node_handle_.param("contact_acquire_time", contact_acquire_time_, 1.0);

    node_handle_.param("desired_contact_force", desired_contact_force_, 3.0);
    node_handle_.param("force_tolerance", force_tolerance_, 0.5);
    node_handle_.param("force_kp", force_kp_, 0.0008);
    node_handle_.param("force_ki", force_ki_, 0.0002);
    node_handle_.param("max_force_correction", max_force_correction_, 0.008);
    node_handle_.param("contact_direction_sign", contact_direction_sign_, -1.0);
    node_handle_.param("force_filter_alpha", force_filter_alpha_, 0.9);

    node_handle_.param("connector_points", connector_points_, 12);
    node_handle_.param("scan_resolution", scan_resolution_, 0.003);

    node_handle_.param("lift_off_distance", lift_off_distance_, 0.005);
    node_handle_.param("return_speed", return_speed_, 0.02);

    node_handle_.param("nullspace_stiffness", nullspace_stiffness_, 20.0);

    std::vector<double> stiffness;
    std::vector<double> damping;

    if (!node_handle_.getParam("cartesian_stiffness", stiffness) || stiffness.size() != 6) {
      stiffness = {150.0, 250.0, 40.0, 15.0, 15.0, 15.0};
    }
    if (!node_handle_.getParam("cartesian_damping", damping) || damping.size() != 6) {
      damping = {24.5, 31.6, 12.6, 7.7, 7.7, 7.7};
    }

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
    for (int i = 0; i < 6; ++i) {
      cartesian_stiffness_(i, i) = stiffness[i];
      cartesian_damping_(i, i) = damping[i];
    }

    connector_points_ = std::max(2, connector_points_);
    scan_resolution_ = std::max(0.001, scan_resolution_);
    pass_coverage_ratio_ = std::max(0.0, std::min(1.0, pass_coverage_ratio_));
    num_passes_ = std::max(1, num_passes_);

    ROS_INFO("PandaUltrasoundForceScanController initialized.");
    return true;
  }

  void starting(const ros::Time& time) override {
    franka::RobotState robot_state = state_handle_->getRobotState();

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    initial_position_ = transform.translation();
    initial_orientation_ = Eigen::Quaterniond(transform.linear());
    initial_orientation_.normalize();
    desired_orientation_ = initial_orientation_;

    q_d_nullspace_ = q;

    if (desired_contact_force_ <= 1e-6 || contact_acquire_time_ <= 1e-6) {
      phase_ = Phase::SCAN;
      ROS_INFO("Starting directly in SCAN phase.");
    } else {
      phase_ = Phase::CONTACT;
      ROS_INFO("Starting in CONTACT phase.");
    }

    phase_start_time_ = time;
    measured_force_filtered_ = 0.0;
    force_integral_ = 0.0;
    scan_progress_ = 0.0;
    return_progress_ = 0.0;

    buildScanPath();
    buildReturnPath();

    ROS_INFO_STREAM("Force scan controller starting.");
    ROS_INFO_STREAM("Initial position: " << initial_position_.transpose());
    ROS_INFO_STREAM("Scan path total length: " << scan_path_total_length_);
    ROS_INFO_STREAM("Return path total length: " << return_path_total_length_);
  }

  void update(const ros::Time& time, const ros::Duration& period) override {
    franka::RobotState robot_state = state_handle_->getRobotState();

    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    orientation.normalize();

    const double dt = std::max(1e-6, period.toSec());

    // Measured external wrench estimate in base frame, projected to contact axis
    double raw_contact_force = -contact_direction_sign_ * robot_state.O_F_ext_hat_K[contact_axis_];
    measured_force_filtered_ =
        force_filter_alpha_ * measured_force_filtered_ + (1.0 - force_filter_alpha_) * raw_contact_force;

    const bool force_control_enabled =
        (desired_contact_force_ > 1e-6) && (max_force_correction_ > 1e-9);

    Eigen::Vector3d nominal_position = initial_position_;
    bool use_force_control = false;

    switch (phase_) {
      case Phase::CONTACT: {
        nominal_position = initial_position_;
        nominal_position(contact_axis_) += contact_offset_;
        use_force_control = force_control_enabled;

        if ((time - phase_start_time_).toSec() >= contact_acquire_time_) {
          phase_ = Phase::SCAN;
          phase_start_time_ = time;
          ROS_INFO("Force contact acquired phase complete. Starting scan.");
        }
        break;
      }

      case Phase::SCAN: {
        use_force_control = force_control_enabled;
        scan_progress_ = std::min(scan_progress_ + scan_speed_ * dt, scan_path_total_length_);
        nominal_position = samplePath(scan_path_points_, scan_path_cumulative_lengths_, scan_progress_);

        if (scan_progress_ >= scan_path_total_length_ - 1e-6) {
          phase_ = Phase::RETURN;
          phase_start_time_ = time;
          return_progress_ = 0.0;
          ROS_INFO("Scan finished. Returning to start pose.");
        }
        break;
      }

      case Phase::RETURN: {
        use_force_control = false;
        return_progress_ = std::min(return_progress_ + return_speed_ * dt, return_path_total_length_);
        nominal_position = samplePath(return_path_points_, return_path_cumulative_lengths_, return_progress_);

        if (return_progress_ >= return_path_total_length_ - 1e-6) {
          phase_ = Phase::HOLD;
          phase_start_time_ = time;
          ROS_INFO("Returned to start pose. Holding.");
        }
        break;
      }

      case Phase::HOLD:
      default: {
        use_force_control = false;
        nominal_position = initial_position_;
        break;
      }
    }

    Eigen::Vector3d desired_position = nominal_position;

    if (use_force_control) {
      double force_error = desired_contact_force_ - measured_force_filtered_;
      force_integral_ += force_error * dt;
      force_integral_ = std::max(-10.0, std::min(10.0, force_integral_));

      double u = force_kp_ * force_error + force_ki_ * force_integral_;
      u = std::max(-max_force_correction_, std::min(max_force_correction_, u));

      desired_position(contact_axis_) += contact_direction_sign_ * u;
    } else {
      force_integral_ = 0.0;
    }

    Eigen::Matrix<double, 6, 1> error;
    error.setZero();
    error.head<3>() = position - desired_position;

    if (desired_orientation_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }

    Eigen::Quaterniond error_quaternion(orientation.inverse() * desired_orientation_);
    error.tail<3>() << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail<3>() = -transform.rotation() * error.tail<3>();

    Eigen::Matrix<double, 7, 1> tau_task =
        jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));

    Eigen::Matrix<double, 7, 1> tau_nullspace;
    tau_nullspace.setZero();

    Eigen::Matrix<double, 7, 1> tau_d = tau_task + tau_nullspace + coriolis;
    std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d, tau_J_d);

    ROS_INFO_THROTTLE(
        1.0,
        "phase=%d scan_progress=%.4f return_progress=%.4f "
        "pos=[%.4f %.4f %.4f] des=[%.4f %.4f %.4f] fz=%.3f use_force=%d",
        static_cast<int>(phase_),
        scan_progress_,
        return_progress_,
        position(0), position(1), position(2),
        desired_position(0), desired_position(1), desired_position(2),
        measured_force_filtered_,
        static_cast<int>(use_force_control));

    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d_saturated[i]);
    }
  }

 private:
  enum class Phase { CONTACT, SCAN, RETURN, HOLD };

  ros::NodeHandle node_handle_;

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  std::string arm_id_;
  std::vector<std::string> joint_names_;

  int scan_axis_{1};
  int lateral_axis_{0};
  int contact_axis_{2};

  double scan_length_{0.145};
  double scan_width_{0.105};
  int num_passes_{4};
  double scan_speed_{0.003};
  double pass_coverage_ratio_{0.75};

  double curve_half_width_{0.0525};
  double curve_drop_{0.010};
  double contact_offset_{0.0};
  double lateral_offset_{0.0};

  double settle_time_{1.0};
  double contact_acquire_time_{1.0};

  double desired_contact_force_{3.0};
  double force_tolerance_{0.5};
  double force_kp_{0.0008};
  double force_ki_{0.0002};
  double max_force_correction_{0.008};
  double contact_direction_sign_{-1.0};
  double force_filter_alpha_{0.9};

  int connector_points_{12};
  double scan_resolution_{0.003};

  double lift_off_distance_{0.005};
  double return_speed_{0.02};

  double nullspace_stiffness_{20.0};

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;

  Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond initial_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond desired_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 7, 1> q_d_nullspace_{Eigen::Matrix<double, 7, 1>::Zero()};

  std::vector<Eigen::Vector3d> scan_path_points_;
  std::vector<double> scan_path_cumulative_lengths_;
  double scan_path_total_length_{0.0};

  std::vector<Eigen::Vector3d> return_path_points_;
  std::vector<double> return_path_cumulative_lengths_;
  double return_path_total_length_{0.0};

  double measured_force_filtered_{0.0};
  double force_integral_{0.0};

  double scan_progress_{0.0};
  double return_progress_{0.0};

  Phase phase_{Phase::CONTACT};
  ros::Time phase_start_time_;

  void appendInterpolatedPointSegment(const Eigen::Vector3d& from,
                                      const Eigen::Vector3d& to,
                                      int num_points,
                                      std::vector<Eigen::Vector3d>& out,
                                      bool skip_first = true) {
    int points = std::max(1, num_points);
    for (int i = 0; i <= points; ++i) {
      if (skip_first && i == 0) {
        continue;
      }
      double alpha = static_cast<double>(i) / static_cast<double>(points);
      out.push_back(from + alpha * (to - from));
    }
  }

  void buildCumulativeLengths(const std::vector<Eigen::Vector3d>& pts,
                              std::vector<double>& cum,
                              double& total_length) {
    cum.clear();
    if (pts.empty()) {
      total_length = 0.0;
      return;
    }

    cum.reserve(pts.size());
    cum.push_back(0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
      double ds = (pts[i] - pts[i - 1]).norm();
      cum.push_back(cum.back() + ds);
    }
    total_length = cum.back();
  }

  Eigen::Vector3d samplePath(const std::vector<Eigen::Vector3d>& pts,
                             const std::vector<double>& cum,
                             double s) const {
    if (pts.empty()) {
      return initial_position_;
    }
    if (pts.size() == 1) {
      return pts.front();
    }
    if (s <= 0.0) {
      return pts.front();
    }
    if (s >= cum.back()) {
      return pts.back();
    }

    auto upper = std::upper_bound(cum.begin(), cum.end(), s);
    size_t idx = std::distance(cum.begin(), upper);
    idx = std::max<size_t>(1, idx);

    double s0 = cum[idx - 1];
    double s1 = cum[idx];
    double alpha = (s - s0) / std::max(1e-9, s1 - s0);

    return pts[idx - 1] + alpha * (pts[idx] - pts[idx - 1]);
  }

  void buildScanPath() {
    scan_path_points_.clear();

    Eigen::Vector3d current = initial_position_;
    scan_path_points_.push_back(current);

    std::vector<double> pass_centers = buildPassCenters(num_passes_, curve_half_width_, pass_coverage_ratio_);
    const double half_scan_length = 0.5 * scan_length_;
    const int line_points = std::max(20, static_cast<int>(std::ceil(scan_length_ / scan_resolution_)));

    for (int pass = 0; pass < num_passes_; ++pass) {
      bool forward = (pass % 2 == 0);

      double x_local = pass_centers[pass];
      double x_value = lateral_offset_ + x_local;
      double z_value = initial_position_(contact_axis_) + contact_offset_ +
                       curveZOffset(x_local, curve_half_width_, curve_drop_);

      double y_start_local = forward ? -half_scan_length : half_scan_length;
      double y_end_local = forward ? half_scan_length : -half_scan_length;

      Eigen::Vector3d pass_start = current;
      Eigen::Vector3d pass_end = current;

      pass_start(lateral_axis_) = initial_position_(lateral_axis_) + x_value;
      pass_start(scan_axis_) = initial_position_(scan_axis_) + y_start_local;
      pass_start(contact_axis_) = z_value;

      pass_end(lateral_axis_) = initial_position_(lateral_axis_) + x_value;
      pass_end(scan_axis_) = initial_position_(scan_axis_) + y_end_local;
      pass_end(contact_axis_) = z_value;

      appendInterpolatedPointSegment(current, pass_start, connector_points_, scan_path_points_, true);
      appendInterpolatedPointSegment(pass_start, pass_end, line_points, scan_path_points_, true);

      current = pass_end;
    }

    buildCumulativeLengths(scan_path_points_, scan_path_cumulative_lengths_, scan_path_total_length_);
  }

  void buildReturnPath() {
    return_path_points_.clear();

    if (scan_path_points_.empty()) {
      return_path_points_.push_back(initial_position_);
      buildCumulativeLengths(return_path_points_, return_path_cumulative_lengths_, return_path_total_length_);
      return;
    }

    Eigen::Vector3d scan_end = scan_path_points_.back();
    Eigen::Vector3d lifted_end = scan_end;
    Eigen::Vector3d lifted_home = initial_position_;
    Eigen::Vector3d home = initial_position_;

    // Lift away from the surface opposite to the push direction
    lifted_end(contact_axis_) += -contact_direction_sign_ * lift_off_distance_;
    lifted_home(contact_axis_) += -contact_direction_sign_ * lift_off_distance_;

    return_path_points_.push_back(scan_end);
    appendInterpolatedPointSegment(scan_end, lifted_end,
                                   std::max(2, static_cast<int>(std::ceil(lift_off_distance_ / scan_resolution_))),
                                   return_path_points_, true);
    appendInterpolatedPointSegment(lifted_end, lifted_home, connector_points_, return_path_points_, true);
    appendInterpolatedPointSegment(lifted_home, home,
                                   std::max(2, static_cast<int>(std::ceil(lift_off_distance_ / scan_resolution_))),
                                   return_path_points_, true);

    buildCumulativeLengths(return_path_points_, return_path_cumulative_lengths_, return_path_total_length_);
  }

  static std::array<double, 7> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d) {
    std::array<double, 7> tau_d_saturated{};
    const double delta_tau_max = 1.0;

    for (size_t i = 0; i < 7; i++) {
      double difference = tau_d_calculated(i) - tau_J_d(i);
      tau_d_saturated[i] = tau_J_d(i) + std::max(std::min(difference, delta_tau_max), -delta_tau_max);
    }
    return tau_d_saturated;
  }

  static double curveZOffset(double x_local, double curve_half_width, double curve_drop) {
    if (curve_half_width <= 1e-9) {
      return 0.0;
    }
    double ratio = x_local / curve_half_width;
    return -curve_drop * ratio * ratio;
  }

  static std::vector<double> buildPassCenters(int num_passes, double curve_half_width, double coverage_ratio) {
    std::vector<double> centers;
    int passes = std::max(1, num_passes);
    double ratio = std::max(0.0, std::min(1.0, coverage_ratio));
    double usable_half_width = ratio * curve_half_width;

    if (passes == 1) {
      centers.push_back(0.0);
      return centers;
    }

    double start = -usable_half_width;
    double end = usable_half_width;
    double step = (end - start) / static_cast<double>(passes - 1);

    for (int i = 0; i < passes; ++i) {
      centers.push_back(start + static_cast<double>(i) * step);
    }
    return centers;
  }
};

}  // namespace panda_lung_scan_noetic

PLUGINLIB_EXPORT_CLASS(
    panda_lung_scan_noetic::PandaUltrasoundForceScanController,
    controller_interface::ControllerBase)