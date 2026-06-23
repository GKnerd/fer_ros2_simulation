#pragma once // Header guard to prevent multiple inclusions of this file

#include <cmath>
#include <vector>

struct TrajectoryPoint { // This struct represents the output point on the trajectory, containing the position (q), velocity (dq), and acceleration (ddq) for each joint at a given time
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> ddq;
};

// Quintic (5th-order) polynomial trajectory per joint.
// Supports non-zero initial velocity so segment transitions are velocity-continuous.
// Boundary conditions: arbitrary velocity at start, zero velocity and acceleration at end.
// Holds at the goal pose for t > T.
class QuinticTrajectoryGenerator
{
public:
  void init( // receives
    const std::vector<double> & q_start,   // starting joint configuration
    const std::vector<double> & q_goal,    // final joint configuration
    double duration,                        // duration of the trajectory in seconds (must be > 0)
    const std::vector<double> & dq_start = {}) // initial joint velocities (zero if omitted)
  {
    q_start_  = q_start;
    q_goal_   = q_goal;
    T_        = duration > 0.0 ? duration : 1.0;
    n_        = q_start.size();
    dq_start_ = dq_start.empty() ? std::vector<double>(n_, 0.0) : dq_start;
    ready_    = true;
  }

  // Returns q_ref, dq_ref, ddq_ref at time t.
  // When t >= T the result is clamped to the goal with zero vel/acc.
  TrajectoryPoint eval(double t) const// t is the time since this trajectory segment started.
  {
    TrajectoryPoint pt;
    pt.q.resize(n_);
    pt.dq.resize(n_);
    pt.ddq.resize(n_);

    if (t >= T_) {
      for (size_t i = 0; i < n_; ++i) {
        pt.q[i] = q_goal_[i];
        pt.dq[i] = 0.0;
        pt.ddq[i] = 0.0;
      }
      return pt;
    }

    const double s  = t / T_;
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;

    for (size_t i = 0; i < n_; ++i) {
      const double delta = q_goal_[i] - q_start_[i];
      const double dp0   = dq_start_[i] * T_;  // normalised initial velocity

      // General quintic coefficients (zero initial velocity gives classic 10/−15/6 form)
      const double c3 = 10.0 * delta - 6.0 * dp0;
      const double c4 =  8.0 * dp0   - 15.0 * delta;
      const double c5 =  6.0 * delta  - 3.0 * dp0;

      pt.q[i]   = q_start_[i] + dp0 / T_ * t + c3 * s3 + c4 * s4 + c5 * s5;
      pt.dq[i]  = dq_start_[i] + (3.0 * c3 * s2 + 4.0 * c4 * s3 + 5.0 * c5 * s4) / T_;
      pt.ddq[i] = (6.0 * c3 * s + 12.0 * c4 * s2 + 20.0 * c5 * s3) / (T_ * T_);
    }

    return pt;
  }

  bool initialized() const { return ready_; }

private:
  std::vector<double> q_start_;
  std::vector<double> q_goal_;
  std::vector<double> dq_start_;
  double T_{1.0};
  size_t n_{0};
  bool   ready_{false};
};
