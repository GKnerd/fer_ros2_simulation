#include <cmath>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

class FkIkWaypointTester : public rclcpp::Node
{
public:
  FkIkWaypointTester() : Node("fk_ik_waypoint_tester")
  {
    // 6 joint-space configurations used as MPC waypoints.
    // All are within Franka FER joint limits.
    // FK is computed for each so the output can be pasted directly
    // into waypoint_sequencer.py for MoveIt validation.
    named_waypoints_ = {
      {"home", {0.0,  -0.7854, 0.0, -2.3562,  0.0,  1.5708, 0.7854}},
      {"A",    {0.3,  -0.9,    0.0, -1.9,      0.0,  1.8,    0.5   }},
      {"B",    {-0.3, -0.8,    0.2, -2.0,      0.0,  1.6,    0.8   }},
      {"C",    {0.5,  -1.0,   -0.2, -1.5,      0.1,  2.0,    0.3   }},
      {"D",    {-0.5, -1.2,    0.3, -2.2,     -0.1,  1.4,    1.0   }},
      {"E",    {0.0,  -0.6,    0.0, -2.5,      0.0,  1.7,    0.7   }},
    };

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description", qos,
      std::bind(&FkIkWaypointTester::robotDescriptionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "FK waypoint tester started. Waiting for /robot_description...");
  }

private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (tested_) return;

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(msg->data, tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse /robot_description.");
      return;
    }
    if (!tree.getChain("base", "fer_hand_tcp", chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain base->fer_hand_tcp.");
      return;
    }
    if (chain_.getNrOfJoints() != 7) {
      RCLCPP_ERROR(this->get_logger(), "Expected 7 joints, got %u.", chain_.getNrOfJoints());
      return;
    }

    runFK();
    tested_ = true;
  }

  void runFK()
  {
    // Franka FER joint limits (from fk_ik_waypoint_tester original + joint_limits.yaml)
    const std::array<double, 7> q_min = {-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973};
    const std::array<double, 7> q_max = { 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973};

    KDL::ChainFkSolverPos_recursive fk_solver(chain_);

    RCLCPP_INFO(this->get_logger(), "=== FK results for MPC waypoints ===");
    RCLCPP_INFO(this->get_logger(),
      "Paste the Python lines into WAYPOINTS in waypoint_sequencer.py");
    RCLCPP_INFO(this->get_logger(), "=====================================");

    for (const auto & [label, q_vec] : named_waypoints_) {
      // Check every joint is within limits before running FK
      bool ok = true;
      for (size_t i = 0; i < 7; ++i) {
        if (q_vec[i] < q_min[i] || q_vec[i] > q_max[i]) {
          RCLCPP_ERROR(this->get_logger(),
            "[%s] joint%zu = %.4f outside limits [%.4f, %.4f] — skipped",
            label.c_str(), i + 1, q_vec[i], q_min[i], q_max[i]);
          ok = false;
        }
      }
      if (!ok) continue;

      KDL::JntArray q_kdl(7);
      for (size_t i = 0; i < 7; ++i) q_kdl(i) = q_vec[i];

      KDL::Frame frame;
      if (fk_solver.JntToCart(q_kdl, frame) < 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s] FK failed.", label.c_str());
        continue;
      }

      double qx, qy, qz, qw;
      frame.M.GetQuaternion(qx, qy, qz, qw);

      RCLCPP_INFO(this->get_logger(), "--- %s ---", label.c_str());
      RCLCPP_INFO(this->get_logger(),
        "  position : x=%.4f  y=%.4f  z=%.4f", frame.p.x(), frame.p.y(), frame.p.z());
      RCLCPP_INFO(this->get_logger(),
        "  quaternion: qx=%.4f  qy=%.4f  qz=%.4f  qw=%.4f", qx, qy, qz, qw);
      // Ready-to-paste Python tuple for waypoint_sequencer.py
      RCLCPP_INFO(this->get_logger(),
        "  >>> (\"%s\", %.4f, %.4f, %.4f,  %.4f, %.4f, %.4f, %.4f),",
        label.c_str(),
        frame.p.x(), frame.p.y(), frame.p.z(),
        qx, qy, qz, qw);
    }

    RCLCPP_INFO(this->get_logger(), "=====================================");
    RCLCPP_INFO(this->get_logger(), "Done — Ctrl+C to exit.");
  }

  bool tested_{false};

  std::vector<std::pair<std::string, std::vector<double>>> named_waypoints_;

  KDL::Chain chain_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FkIkWaypointTester>());
  rclcpp::shutdown();
  return 0;
}
