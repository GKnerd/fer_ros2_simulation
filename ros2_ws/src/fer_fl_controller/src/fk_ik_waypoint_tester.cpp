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
    q_start_ = {0.0, -0.785398, 0.0, -2.35619, 0.0, 1.5708, 0.785398};

    forward_distances_ = {0.05, 0.10, 0.15, 0.20};

    auto qos = rclcpp::QoS(1).transient_local().reliable();

    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description",
      qos,
      std::bind(&FkIkWaypointTester::robotDescriptionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "FK/IK waypoint tester started. Waiting for /robot_description...");
  }

private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (tested_) {
      return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(msg->data, tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse /robot_description.");
      return;
    }

    if (!tree.getChain("base", "fer_hand_tcp", chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain from base to fer_hand_tcp.");
      return;
    }

    if (chain_.getNrOfJoints() != 7) {
      RCLCPP_ERROR(
        this->get_logger(),
        "KDL chain has %u joints, expected 7.",
        chain_.getNrOfJoints());
      return;
    }

    runTests();
    tested_ = true;
  }

  void runTests()
  {
    KDL::JntArray q_start_kdl(7);

    for (size_t i = 0; i < 7; ++i) {
      q_start_kdl(i) = q_start_[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain_);

    KDL::Frame start_frame;
    if (fk_solver.JntToCart(q_start_kdl, start_frame) < 0) {
      RCLCPP_ERROR(this->get_logger(), "FK failed for q_start.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start joint pose:");
    printJointArray(q_start_kdl);

    RCLCPP_INFO(
      this->get_logger(),
      "Start end-effector position: x=%.4f, y=%.4f, z=%.4f",
      start_frame.p.x(),
      start_frame.p.y(),
      start_frame.p.z());

    KDL::JntArray q_min(7);
    KDL::JntArray q_max(7);

    q_min(0) = -2.8973; q_max(0) =  2.8973;
    q_min(1) = -1.7628; q_max(1) =  1.7628;
    q_min(2) = -2.8973; q_max(2) =  2.8973;
    q_min(3) = -3.0718; q_max(3) = -0.0698;
    q_min(4) = -2.8973; q_max(4) =  2.8973;
    q_min(5) = -0.0175; q_max(5) =  3.7525;
    q_min(6) = -2.8973; q_max(6) =  2.8973;

    KDL::ChainIkSolverVel_pinv ik_vel_solver(chain_);
    KDL::ChainIkSolverPos_NR_JL ik_solver(
      chain_,
      q_min,
      q_max,
      fk_solver,
      ik_vel_solver,
      100,
      1e-6);

    for (double distance : forward_distances_) {
      KDL::Frame target_frame = start_frame;

      target_frame.p.x(start_frame.p.x() + distance);

      KDL::JntArray q_solution(7);

      int ik_result = ik_solver.CartToJnt(q_start_kdl, target_frame, q_solution);

      RCLCPP_INFO(this->get_logger(), "----------------------------------------");
      RCLCPP_INFO(this->get_logger(), "Testing forward displacement: %.2f cm", distance * 100.0);

      if (ik_result < 0) {
        RCLCPP_WARN(this->get_logger(), "IK failed for %.2f cm forward.", distance * 100.0);
        continue;
      }

      KDL::Frame check_frame;
      if (fk_solver.JntToCart(q_solution, check_frame) < 0) {
        RCLCPP_WARN(this->get_logger(), "FK check failed after IK.");
        continue;
      }

      double dx = target_frame.p.x() - check_frame.p.x();
      double dy = target_frame.p.y() - check_frame.p.y();
      double dz = target_frame.p.z() - check_frame.p.z();
      double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

      RCLCPP_INFO(this->get_logger(), "IK succeeded.");
      RCLCPP_INFO(
        this->get_logger(),
        "Target position: x=%.4f, y=%.4f, z=%.4f",
        target_frame.p.x(),
        target_frame.p.y(),
        target_frame.p.z());

      RCLCPP_INFO(
        this->get_logger(),
        "FK check position: x=%.4f, y=%.4f, z=%.4f",
        check_frame.p.x(),
        check_frame.p.y(),
        check_frame.p.z());

      RCLCPP_INFO(this->get_logger(), "Cartesian position error: %.6f m", position_error);

      RCLCPP_INFO(this->get_logger(), "Joint waypoint from IK:");
      printJointArray(q_solution);
    }
  }

  void printJointArray(const KDL::JntArray & q)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6);
    stream << "{";
    for (unsigned int i = 0; i < q.rows(); ++i) {
      stream << q(i);
      if (i + 1 < q.rows()) {
        stream << ", ";
      }
    }
    stream << "}";

    RCLCPP_INFO(this->get_logger(), "%s", stream.str().c_str());
  }

  bool tested_{false};

  std::vector<double> q_start_;
  std::vector<double> forward_distances_;

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
