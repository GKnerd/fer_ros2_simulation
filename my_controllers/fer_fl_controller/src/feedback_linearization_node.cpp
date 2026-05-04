// #include <algorithm>
// #include <fstream>
// #include <iomanip>
// #include <memory>
// #include <string>
// #include <unordered_map>
// #include <vector>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"

// class FeedbackLinearizationNode : public rclcpp::Node
// {
// public:
//   FeedbackLinearizationNode() : Node("feedback_linearization_node")
//   {
//     joint_names_ = {
//       "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
//       "fer_joint5", "fer_joint6", "fer_joint7"
//     };

//     // Safe desired posture in radians, close to the observed initial FER posture
//     q_des_ = {
//       0.0,       // joint1
//       -0.7854,   // joint2
//       0.0,       // joint3
//       -2.3562,   // joint4
//       0.0,       // joint5
//       1.5708,    // joint6
//       0.7854     // joint7
//     };

//     dq_des_ = std::vector<double>(7, 0.0);

//     // Initial test gains
//     kp_ = {30.0, 30.0, 25.0, 20.0, 8.0, 6.0, 4.0};
//     kd_ = { 8.0,  8.0,  6.0,  5.0, 3.0, 2.0, 1.5};

//     // Torque limits from FER joint_limits.yaml
//     tau_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

//     effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
//       "/effort_controller/commands", 10);

//     joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
//       "/joint_states", 10,
//       std::bind(&FeedbackLinearizationNode::jointCallback, this, std::placeholders::_1));

//     log_file_.open("fl_test_log.csv");
//     log_file_ << std::setprecision(17);

//     log_file_ << "time";
//     for (int i = 0; i < 7; ++i) {
//       log_file_ << ",q" << i + 1;
//     }
//     for (int i = 0; i < 7; ++i) {
//       log_file_ << ",dq" << i + 1;
//     }
//     for (int i = 0; i < 7; ++i) {
//       log_file_ << ",tau" << i + 1;
//     }
//     for (int i = 0; i < 7; ++i) {
//       log_file_ << ",qdes" << i + 1;
//     }
//     log_file_ << "\n";

//     RCLCPP_INFO(this->get_logger(), "Improved PD torque test controller started.");
//   }

//   ~FeedbackLinearizationNode()
//   {
//     if (log_file_.is_open()) {
//       log_file_.close();
//     }
//   }

// private:
//   void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
//   {
//     std::unordered_map<std::string, size_t> name_to_index;
//     for (size_t i = 0; i < msg->name.size(); ++i) {
//       name_to_index[msg->name[i]] = i;
//     }

//     std::vector<double> q(7, 0.0);
//     std::vector<double> dq(7, 0.0);
//     std::vector<double> tau(7, 0.0);

//     // Extract only the 7 arm joints in the correct order
//     for (size_t i = 0; i < joint_names_.size(); ++i) {
//       if (name_to_index.find(joint_names_[i]) == name_to_index.end()) {
//         RCLCPP_WARN_THROTTLE(
//           this->get_logger(), *this->get_clock(), 2000,
//           "Joint %s not found in /joint_states yet.", joint_names_[i].c_str());
//         return;
//       }

//       size_t idx = name_to_index[joint_names_[i]];

//       if (idx >= msg->position.size() || idx >= msg->velocity.size()) {
//         RCLCPP_WARN_THROTTLE(
//           this->get_logger(), *this->get_clock(), 2000,
//           "Joint state message does not contain full position/velocity data.");
//         return;
//       }

//       q[i] = msg->position[idx];
//       dq[i] = msg->velocity[idx];
//     }

//     // PD torque control with saturation
//     for (size_t i = 0; i < 7; ++i) {
//       double error = q_des_[i] - q[i];
//       double derror = dq_des_[i] - dq[i];

//       tau[i] = kp_[i] * error + kd_[i] * derror;
//       tau[i] = std::clamp(tau[i], -tau_limits_[i], tau_limits_[i]);
//     }

//     // Publish torque command
//     std_msgs::msg::Float64MultiArray cmd;
//     cmd.data = tau;
//     effort_pub_->publish(cmd);

//     // Save results for plotting using relative time
//     double t_now = this->now().seconds();
//     if (start_time_ < 0.0) {
//       start_time_ = t_now;
//     }
//     double t = t_now - start_time_;

//     if (log_file_.is_open()) {
//       log_file_ << t;
//       for (double val : q) {
//         log_file_ << "," << val;
//       }
//       for (double val : dq) {
//         log_file_ << "," << val;
//       }
//       for (double val : tau) {
//         log_file_ << "," << val;
//       }
//       for (double val : q_des_) {
//         log_file_ << "," << val;
//       }
//       log_file_ << "\n";
//     }

//     // Small debug print every 2 seconds
//     RCLCPP_INFO_THROTTLE(
//       this->get_logger(), *this->get_clock(), 2000,
//       "Publishing torque command. Example: tau1=%.3f tau2=%.3f tau3=%.3f",
//       tau[0], tau[1], tau[2]);
//   }

//   std::vector<std::string> joint_names_;
//   std::vector<double> q_des_;
//   std::vector<double> dq_des_;
//   std::vector<double> kp_;
//   std::vector<double> kd_;
//   std::vector<double> tau_limits_;

//   double start_time_ = -1.0;

//   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;

//   std::ofstream log_file_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<FeedbackLinearizationNode>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <algorithm>
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

class FeedbackLinearizationNode : public rclcpp::Node
{
public:
  FeedbackLinearizationNode() : Node("feedback_linearization_node")
  {
    joint_names_ = {
      "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
      "fer_joint5", "fer_joint6", "fer_joint7"
    };

    q_des_   = {0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854};
    dq_des_  = std::vector<double>(7, 0.0);
    ddq_des_ = std::vector<double>(7, 0.0);

    kp_ = {30.0, 30.0, 25.0, 15.0, 8.0, 6.0, 4.0};
    kd_ = { 8.0,  8.0,  6.0,  8.0, 3.0, 2.0, 1.5};

    tau_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FeedbackLinearizationNode::jointCallback, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description", qos,
      std::bind(&FeedbackLinearizationNode::robotDescriptionCallback, this, std::placeholders::_1));

    log_file_.open("fl_full_log.csv");
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
    for (int i = 0; i < 7; ++i) log_file_ << ",tau_raw" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",tau_sat" << i + 1;

    log_file_ << "\n";

    RCLCPP_INFO(this->get_logger(), "Full feedback linearization diagnostic node started.");
  }

  ~FeedbackLinearizationNode()
  {
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

private:
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

    if (!tree.getChain("base", "fer_link8", chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain from base to fer_link8");
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
    std::vector<double> tau_raw(7, 0.0);
    std::vector<double> tau_sat(7, 0.0);

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

      q[i] = msg->position[idx];
      dq[i] = msg->velocity[idx];

      q_kdl_(i) = q[i];
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

    for (size_t i = 0; i < 7; ++i) {
      error[i] = q_des_[i] - q[i];
      double de = dq_des_[i] - dq[i];

      v[i] = ddq_des_[i] + kd_[i] * de + kp_[i] * error[i];
    }

    for (size_t i = 0; i < 7; ++i) {
      double mv = 0.0;

      for (size_t j = 0; j < 7; ++j) {
        mv += mass_kdl_(i, j) * v[j];
      }

      mv_vec[i] = mv;
      c_vec[i] = coriolis_kdl_(i);
      g_vec[i] = gravity_kdl_(i);

      tau_raw[i] = mv_vec[i] + c_vec[i] + g_vec[i];
      tau_sat[i] = std::clamp(tau_raw[i], -tau_limits_[i], tau_limits_[i]);
    }

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = tau_sat;
    effort_pub_->publish(cmd);

    double t_now = this->now().seconds();
    if (start_time_ < 0.0) {
      start_time_ = t_now;
    }
    double t = t_now - start_time_;

    if (log_file_.is_open()) {
      log_file_ << t;

      for (double val : q) log_file_ << "," << val;
      for (double val : dq) log_file_ << "," << val;
      for (double val : q_des_) log_file_ << "," << val;
      for (double val : error) log_file_ << "," << val;

      for (double val : v) log_file_ << "," << val;
      for (double val : mv_vec) log_file_ << "," << val;
      for (double val : c_vec) log_file_ << "," << val;
      for (double val : g_vec) log_file_ << "," << val;
      for (double val : tau_raw) log_file_ << "," << val;
      for (double val : tau_sat) log_file_ << "," << val;

      log_file_ << "\n";
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Full FL diagnostic: raw=[%.2f %.2f %.2f], sat=[%.2f %.2f %.2f]",
      tau_raw[0], tau_raw[1], tau_raw[2],
      tau_sat[0], tau_sat[1], tau_sat[2]);
  }

  std::mutex mutex_;

  bool model_ready_{false};
  double start_time_{-1.0};

  std::vector<std::string> joint_names_;
  std::vector<double> q_des_;
  std::vector<double> dq_des_;
  std::vector<double> ddq_des_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> tau_limits_;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
  KDL::JntArray q_kdl_;
  KDL::JntArray dq_kdl_;
  KDL::JntArray coriolis_kdl_;
  KDL::JntArray gravity_kdl_;
  KDL::JntSpaceInertiaMatrix mass_kdl_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;

  std::ofstream log_file_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeedbackLinearizationNode>());
  rclcpp::shutdown();
  return 0;
}