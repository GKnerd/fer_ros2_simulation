#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// C interface to the CasADi QP solver (compiled with old ABI in casadi_qp_wrapper.so)
#include "fer_fl_controller/casadi_qp_wrapper.h"

#include "fer_fl_controller/trajectory_generator.hpp"

// CASADI_INSTALL_DIR is injected by CMake to locate CasADi plugins at runtime.
#ifndef CASADI_INSTALL_DIR
#define CASADI_INSTALL_DIR ""
#endif

class MPCCoupledQPNode : public rclcpp::Node
{
public:
  MPCCoupledQPNode() : Node("mpc_coupled_qp_node")
  {
    joint_names_ = {
      "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
      "fer_joint5", "fer_joint6", "fer_joint7"
    };
waypoints_ = {
  { 0.8,  -0.5,    0.5, -2.8,    0.5,  1.2,    0.3   },  // A
  { 0.0,  -1.0,    0.0, -1.5,    0.0,  2.5,    1.5   },  // B
  {-0.8,  -0.5,   -0.5, -2.8,   -0.5,  1.2,    0.3   },  // C
  { 0.0,  -1.3,    0.8, -2.0,   -0.5,  1.8,    0.6   },  // D
  { 0.4,  -0.7,   -0.8, -2.2,    0.8,  1.6,    1.0   },  // E
  { 0.0,  -0.7854, 0.0, -2.3562, 0.0,  1.5708, 0.7854},  // home
};



    current_waypoint_ = 0;
    q_des_ = waypoints_[current_waypoint_];
    dq_des_  = std::vector<double>(7, 0.0);
    ddq_des_ = std::vector<double>(7, 0.0);

    tau_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

    // Reflected motor inertia (motor_inertia * gear_ratio^2) from dynamics.yaml.
    armature_ = {0.605721456, 0.605721456,
                 0.462474144, 0.462474144,
                 0.205544064, 0.205544064, 0.205544064};

    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_forward_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MPCCoupledQPNode::jointCallback, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description", qos,
      std::bind(&MPCCoupledQPNode::robotDescriptionCallback, this, std::placeholders::_1));

    this->declare_parameter<double>("trajectory_duration", 5.0);
    traj_duration_ = this->get_parameter("trajectory_duration").as_double();

    this->declare_parameter<std::string>("mpc_mode", "qp_coupled");
    mpc_mode_ = this->get_parameter("mpc_mode").as_string();
    if (mpc_mode_ != "brute_decoupled" &&
        mpc_mode_ != "qp_decoupled"    &&
        mpc_mode_ != "qp_coupled") {
      RCLCPP_WARN(this->get_logger(),
        "Unknown mpc_mode '%s', falling back to qp_coupled.", mpc_mode_.c_str());
      mpc_mode_ = "qp_coupled";
    }

    // Build the CasADi QP solver once through the ABI-isolated C wrapper.
    qp_handle_ = casadi_qp_coupled_create(CASADI_INSTALL_DIR);
    if (!qp_handle_) {
      RCLCPP_FATAL(this->get_logger(),
        "Failed to build CasADi QP solver. Check that casadi_qp_wrapper.so can load "
        "libcasadi.so and its OSQP plugin from '%s'.", CASADI_INSTALL_DIR);
      throw std::runtime_error("casadi_qp_coupled_create failed");
    }
    RCLCPP_INFO(this->get_logger(), "CasADi coupled QP solver ready (N=15, dt=0.01 s).");

    log_file_.open("mpc_coupled_qp_log.csv");
    log_file_ << std::setprecision(17);

    log_file_ << "time";
    for (int i = 0; i < 7; ++i) log_file_ << ",q" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",dq" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",qdes" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",error" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",v" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",mv" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",c" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",g" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",tau_friction" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",tau_raw" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",tau_sat" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",best_cost" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",qref" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",dqref" << i + 1;
    log_file_ << ",current_waypoint,solve_time_ms\n";

    RCLCPP_INFO(this->get_logger(),
      "MPC coupled QP node started (mpc_mode: %s, trajectory_duration: %.1f s).",
      mpc_mode_.c_str(), traj_duration_);
  }

  ~MPCCoupledQPNode()
  {
    casadi_qp_coupled_destroy(qp_handle_);
    qp_handle_ = nullptr;
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

private:
  // ── Brute-force single-joint shooting MPC (kept as fallback / brute_decoupled mode) ──
  struct MPCResult { double v; double cost; };

  MPCResult computeMPCAcceleration(
    double q, double dq,
    double q_ref, double dq_ref, double ddq_ref,
    double m_diag, double tau_limit,
    double q_weight = 100.0,
    double v_prev   = 0.0)
  {
    const double dt                 = 0.01;
    const int    N                  = 15;
    const double dq_weight          = 1.0;
    const double u_weight           = 0.01;
    const double terminal_q_weight  = 50000.0;
    const double terminal_dq_weight = 300.0;

    const double E_q  = q_ref  - q;
    const double E_dq = dq_ref - dq;

    const double u_abs_weight = 0.02;  // prevents zero-joint overshoot
    double num = static_cast<double>(N) * u_weight * v_prev;
    double den = static_cast<double>(N) * (u_weight + u_abs_weight);

    for (int k = 0; k < N; ++k) {
      const double tau  = (k + 1) * dt;
      const double tau2 = tau * tau;
      const double a_k  = E_q + E_dq * tau + 0.5 * ddq_ref * tau2;
      const double b_k  = E_dq + ddq_ref * tau;
      const double wq   = (k == N - 1) ? (q_weight + terminal_q_weight)  : q_weight;
      const double wdq  = (k == N - 1) ? (dq_weight + terminal_dq_weight) : dq_weight;

      num += wq  * 0.5  * tau2 * a_k + wdq * tau * b_k;
      den += wq  * 0.25 * tau2 * tau2 + wdq * tau2;
    }

    const double v_max = tau_limit / std::max(m_diag, 1e-6);
    const double v_opt = std::clamp(num / den, -v_max, v_max);

    double cost   = 0.0;
    double q_sim  = q;
    double dq_sim = dq;
    for (int k = 0; k < N; ++k) {
      q_sim  += dq_sim * dt + 0.5 * v_opt * dt * dt;
      dq_sim += v_opt * dt;

      const double tau  = (k + 1) * dt;
      const double q_k  = q_ref  + dq_ref * tau + 0.5 * ddq_ref * tau * tau;
      const double dq_k = dq_ref + ddq_ref * tau;
      const double e    = q_k  - q_sim;
      const double de   = dq_k - dq_sim;
      const double wq   = (k == N - 1) ? (q_weight + terminal_q_weight)  : q_weight;
      const double wdq  = (k == N - 1) ? (dq_weight + terminal_dq_weight) : dq_weight;

      cost += wq  * e  * e
            + wdq * de * de
            + u_weight * (v_opt - v_prev) * (v_opt - v_prev);
    }

    return {v_opt, cost};
  }

  // Identified friction model for Franka Panda (Cognetti et al. 2019).
  std::vector<double> computeFrictionTorque(const std::vector<double> & dq)
  {
    const std::vector<double> fi1 = {0.54615, 0.87224, 0.64068, 1.2794, 0.83904, 0.30301, 0.56489};
    const std::vector<double> fi2 = {5.1181, 9.0657, 10.136, 5.5903, 8.3469, 17.133, 10.336};
    const std::vector<double> fi3 = {0.039533, 0.025882, -0.04607, 0.036194, 0.026226, -0.021047, 0.0035526};
    std::vector<double> tau_friction(7, 0.0);
    for (size_t i = 0; i < 7; ++i) {
      const double tau_f_const = fi1[i] / (1.0 + std::exp(-fi2[i] * fi3[i]));
      tau_friction[i] = fi1[i] / (1.0 + std::exp(-fi2[i] * (dq[i] + fi3[i]))) - tau_f_const;
    }
    return tau_friction;
  }

  // Friction feedforward using reference velocity — matches the MuJoCo damping model
  // (mu_viscous=16 Nm·s/rad, frictionloss=0.2 Nm from dynamics.yaml).
  std::vector<double> computeFrictionFeedforward(const std::vector<double> & dq_ref)
  {
    const std::vector<double> mu_viscous = {16.0, 16.0, 16.0, 16.0, 16.0, 16.0, 16.0};
    const std::vector<double> friction   = {0.2,  0.2,  0.2,  0.2,  0.2,  0.2,  0.2};
    std::vector<double> tau_ff(7, 0.0);
    for (size_t i = 0; i < 7; ++i) {
      const double sgn = (dq_ref[i] > 1e-6) ? 1.0 : (dq_ref[i] < -1e-6) ? -1.0 : 0.0;
      tau_ff[i] = mu_viscous[i] * dq_ref[i] + friction[i] * sgn;
    }
    return tau_ff;
  }

  // ── Robot description callback ─────────────────────────────────────────────
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    if (model_ready_) {
      return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(msg->data, tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse /robot_description");
      return;
    }

    if (!tree.getChain("base", "fer_hand_tcp", chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain from base to fer_hand_tcp");
      return;
    }

    if (chain_.getNrOfJoints() != 7) {
      RCLCPP_ERROR(
        this->get_logger(),
        "KDL chain has %u joints, expected 7. Check base/tip link.",
        chain_.getNrOfJoints());
      return;
    }

    dyn_solver_ = std::make_unique<KDL::ChainDynParam>(
      chain_, KDL::Vector(0.0, 0.0, -9.81));

    q_kdl_.resize(7);
    dq_kdl_.resize(7);
    coriolis_kdl_.resize(7);
    gravity_kdl_.resize(7);
    mass_kdl_.resize(7);

    model_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "KDL model initialized successfully.");
  }

  // ── Joint state callback ───────────────────────────────────────────────────
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    if (!model_ready_) {
      return;
    }

    std::unordered_map<std::string, size_t> name_to_index;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      name_to_index[msg->name[i]] = i;
    }

    std::vector<double> q(7, 0.0);
    std::vector<double> dq(7, 0.0);
    std::vector<double> error(7, 0.0);
    std::vector<double> v(7, 0.0);
    std::vector<double> mv_vec(7, 0.0);
    std::vector<double> c_vec(7, 0.0);
    std::vector<double> g_vec(7, 0.0);
    std::vector<double> tau_friction(7, 0.0);
    std::vector<double> tau_raw(7, 0.0);
    std::vector<double> tau_sat(7, 0.0);
    std::vector<double> best_cost(7, 0.0);
    double solve_time_ms = 0.0;
    double qp_cost_value = 0.0;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (name_to_index.find(joint_names_[i]) == name_to_index.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Joint %s not found in /joint_states yet.", joint_names_[i].c_str());
        return;
      }

      size_t idx = name_to_index[joint_names_[i]];
      if (idx >= msg->position.size() || idx >= msg->velocity.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Incomplete joint state data.");
        return;
      }

      q[i]       = msg->position[idx];
      dq[i]      = msg->velocity[idx];
      q_kdl_(i)  = q[i];
      dq_kdl_(i) = dq[i];
    }

    if (dyn_solver_->JntToMass(q_kdl_, mass_kdl_) != 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "JntToMass failed.");
      return;
    }
    if (dyn_solver_->JntToCoriolis(q_kdl_, dq_kdl_, coriolis_kdl_) != 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "JntToCoriolis failed.");
      return;
    }
    if (dyn_solver_->JntToGravity(q_kdl_, gravity_kdl_) != 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "JntToGravity failed.");
      return;
    }

    double t_now = this->now().seconds();
    const double dt_loop = (prev_time_ < 0.0) ? 0.0 : (t_now - prev_time_);
    prev_time_ = t_now;

    if (start_time_ < 0.0) {
      start_time_ = t_now;
      traj_gen_.init(q, q_des_, traj_duration_);
    }
    double t = t_now - start_time_;
    auto ref = traj_gen_.eval(t);

    if (t >= traj_duration_ && current_waypoint_ + 1 < waypoints_.size()) {
      current_waypoint_++;
      q_des_ = waypoints_[current_waypoint_];
      traj_gen_.init(q, q_des_, traj_duration_);
      start_time_ = t_now;
      std::fill(error_integral_.begin(), error_integral_.end(), 0.0);
      RCLCPP_INFO(this->get_logger(),
        "Switching to waypoint %zu", current_waypoint_);
    }

  
    for (size_t i = 0; i < 7; ++i) {
      error[i] = ref.q[i] - q[i];
      error_integral_[i] += error[i] * dt_loop;
      error_integral_[i]  = std::clamp(error_integral_[i], -1.0, 1.0);
    }

    // Compute friction once — needed by both the QP (as a parameter) and the
    // feedback-linearisation torque conversion below.
    // 80% feedforward: leaves residual viscous damping (zeta~2) to prevent
    // oscillation; the integral term covers the remaining 20% within ~2-3 s.
    tau_friction = computeFrictionFeedforward(ref.dq);

    if (mpc_mode_ == "qp_coupled") {
      // ── Pack parameter vector for the CasADi QP ───────────────────────────
      double p_vec[105] = {};
      for (int i = 0; i < 7; ++i) {
        p_vec[i]      = q[i];
        p_vec[7  + i] = dq[i];
        p_vec[14 + i] = v_prev_[i];
        p_vec[21 + i] = ref.q[i];
        p_vec[28 + i] = ref.dq[i];
        p_vec[35 + i] = ref.ddq[i];
      }
      // Mass matrix (KDL + armature on diagonal) in column-major order
      for (int col = 0; col < 7; ++col) {
        for (int row = 0; row < 7; ++row) {
          double M_ij = mass_kdl_(row, col);
          if (row == col) M_ij += armature_[row];
          p_vec[42 + col * 7 + row] = M_ij;
        }
      }
      for (int i = 0; i < 7; ++i) {
        p_vec[91 + i] = coriolis_kdl_(i);
        p_vec[98 + i] = tau_friction[i];
      }

      // Shift QP bounds by the integral contribution so the QP plans within the
      // remaining torque budget: -lim ≤ M·v+C+fric+ki·∫e ≤ lim
      // rearranges to: (-lim - ki·∫e) ≤ M·v+C+fric ≤ (lim - ki·∫e)
      double tau_min[7], tau_max[7];
      for (int i = 0; i < 7; ++i) {
        const double tau_int = ki_[i] * error_integral_[i];
        tau_min[i] = -tau_limits_[i] - tau_int;
        tau_max[i] =  tau_limits_[i] - tau_int;
      }

      double v_out[7]    = {};
      int rc = casadi_qp_coupled_solve(
        qp_handle_,
        p_vec, tau_min, tau_max, v_prev_.data(),
        v_out, &solve_time_ms, &qp_cost_value);

      if (rc == 0) {
        for (int i = 0; i < 7; ++i) {
          v[i]        = v_out[i];
          v_prev_[i]  = v_out[i];
          best_cost[i] = qp_cost_value;
        }
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "QP infeasible / failed at q=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]; "
          "v=0 this cycle.",
          q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
        std::fill(v.begin(),      v.end(),      0.0);
        std::fill(v_prev_.begin(), v_prev_.end(), 0.0);
      }

    } else if (mpc_mode_ == "brute_decoupled") {
      const std::array<double, 7> q_weights = {100, 100, 100, 100, 100, 100, 100};
      for (size_t i = 0; i < 7; ++i) {
        auto result = computeMPCAcceleration(
          q[i], dq[i], ref.q[i], ref.dq[i], ref.ddq[i],
          mass_kdl_(i, i) + armature_[i], tau_limits_[i], q_weights[i], v_prev_[i]);
        v[i]         = result.v;
        best_cost[i] = result.cost;
        v_prev_[i]   = result.v;
      }
    } else {
      // qp_decoupled: not yet implemented
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "mpc_mode '%s' is not yet implemented; falling back to brute_decoupled.",
        mpc_mode_.c_str());
      const std::array<double, 7> q_weights = {100, 100, 100, 100, 100, 100, 100};
      for (size_t i = 0; i < 7; ++i) {
        auto result = computeMPCAcceleration(
          q[i], dq[i], ref.q[i], ref.dq[i], ref.ddq[i],
          mass_kdl_(i, i) + armature_[i], tau_limits_[i], q_weights[i], v_prev_[i]);
        v[i]         = result.v;
        best_cost[i] = result.cost;
        v_prev_[i]   = result.v;
      }
    }

    // ── Feedback-linearisation torque conversion (identical to original) ──────
    for (size_t i = 0; i < 7; ++i) {
      double mv = 0.0;
      for (size_t j = 0; j < 7; ++j) {
        double M_ij = mass_kdl_(i, j);
        if (i == j) M_ij += armature_[i];
        mv += M_ij * v[j];
      }

      mv_vec[i] = mv;
      c_vec[i]  = coriolis_kdl_(i);
      g_vec[i]  = gravity_kdl_(i);

      // gravcomp=1 in MJCF already cancels gravity — do not add g_vec.
      tau_raw[i] = mv_vec[i] + c_vec[i] + tau_friction[i]
                 + ki_[i] * error_integral_[i];
      tau_sat[i] = std::clamp(tau_raw[i], -tau_limits_[i], tau_limits_[i]);
    }

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = tau_sat;
    effort_pub_->publish(cmd);

    if (log_file_.is_open()) {
      log_file_ << t;
      for (double val : q)            log_file_ << "," << val;
      for (double val : dq)           log_file_ << "," << val;
      for (double val : q_des_)       log_file_ << "," << val;
      for (double val : error)        log_file_ << "," << val;
      for (double val : v)            log_file_ << "," << val;
      for (double val : mv_vec)       log_file_ << "," << val;
      for (double val : c_vec)        log_file_ << "," << val;
      for (double val : g_vec)        log_file_ << "," << val;
      for (double val : tau_friction) log_file_ << "," << val;
      for (double val : tau_raw)      log_file_ << "," << val;
      for (double val : tau_sat)      log_file_ << "," << val;
      for (double val : best_cost)    log_file_ << "," << val;
      for (double val : ref.q)        log_file_ << "," << val;
      for (double val : ref.dq)       log_file_ << "," << val;
      log_file_ << "," << current_waypoint_ << "," << solve_time_ms << "\n";
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "MPC QP: raw=[%.2f %.2f %.2f], sat=[%.2f %.2f %.2f], solve=%.3f ms",
      tau_raw[0], tau_raw[1], tau_raw[2],
      tau_sat[0], tau_sat[1], tau_sat[2], solve_time_ms);
  }

  // ── Member variables ───────────────────────────────────────────────────────

  std::mutex  mutex_;
  std::string mpc_mode_;

  bool   model_ready_{false};
  double start_time_{-1.0};
  double prev_time_{-1.0};
  double traj_duration_{3.0};

  QuinticTrajectoryGenerator traj_gen_;

  CasadiQPHandle qp_handle_{nullptr};

  std::vector<double> v_prev_ = std::vector<double>(7, 0.0);
  std::vector<double> error_integral_ = std::vector<double>(7, 0.0);
  // Option C: small integral gain with QP-aware bounds shifting. Overcomes gravity model
  // mismatch and static friction residuals without MPC–integral fighting.
  std::vector<double> ki_{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

  std::vector<std::vector<double>> waypoints_;
  size_t current_waypoint_{0};

  std::vector<std::string> joint_names_;
  std::vector<double>      q_des_;
  std::vector<double>      dq_des_;
  std::vector<double>      ddq_des_;
  std::vector<double>      tau_limits_;
  std::vector<double>      armature_;

  KDL::Chain                            chain_;
  std::unique_ptr<KDL::ChainDynParam>   dyn_solver_;
  KDL::JntArray                         q_kdl_;
  KDL::JntArray                         dq_kdl_;
  KDL::JntArray                         coriolis_kdl_;
  KDL::JntArray                         gravity_kdl_;
  KDL::JntSpaceInertiaMatrix            mass_kdl_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         robot_description_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;

  std::ofstream log_file_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCCoupledQPNode>());
  rclcpp::shutdown();
  return 0;
}
