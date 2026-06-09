#pragma once // Header guard to prevent multiple inclusions of this file

#include <cmath>
#include <vector>

struct TrajectoryPoint { // This struct represents the output point on the trajectory, containing the position (q), velocity (dq), and acceleration (ddq) for each joint at a given time
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> ddq;
};

// Quintic (5th-order) polynomial trajectory per joint.
// Boundary conditions: zero velocity and acceleration at both endpoints.
// Holds at the goal pose for t > T.
class QuinticTrajectoryGenerator
{
public:
  void init( // receives
    const std::vector<double> & q_start,// starting joint configuration
    const std::vector<double> & q_goal,// final joint configuration
    double duration)// duration of the trajectory in seconds (must be > 0)  
  {
    q_start_ = q_start;
    q_goal_  = q_goal;
    T_       = duration > 0.0 ? duration : 1.0;// to prevent divison by zero in the eval function.
    n_       = q_start.size();// number of joints (degrees of freedom)
    ready_   = true;
  }

  // Returns q_ref, dq_ref, ddq_ref at time t.
  // When t >= T the result is clamped to the goal with zero vel/acc.
  TrajectoryPoint eval(double t) const// t is the time since this trajectory segment started.
  {
    TrajectoryPoint pt;
    pt.q.resize(n_);
    pt.dq.resize(n_);
    pt.ddq.resize(n_);

    const double s  = std::min(t / T_, 1.0); // current trajectory time/ total trajectory duration, clamped to [0, 1].
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;

    // Quintic blending function and its derivatives w.r.t. time 
    const double p   =  10.0 * s3  - 15.0 * s4  +  6.0 * s5;//Lynch, K. M., & Park, F. C. (2017). Modern Robotics: Mechanics, Planning, and Control. Cambridge University Press.
    const double dp  = (30.0 * s2  - 60.0 * s3  + 30.0 * s4) / T_;
    const double ddp = (60.0 * s   - 180.0 * s2 + 120.0 * s3) / (T_ * T_);

    for (size_t i = 0; i < n_; ++i) {
      const double delta = q_goal_[i] - q_start_[i];// the total change in position for joint i over the trajectory(displacement of joint i).
      pt.q[i]   = q_start_[i] + delta * p;
      pt.dq[i]  = delta * dp;
      pt.ddq[i] = delta * ddp;
    }

    return pt;
  }

  bool initialized() const { return ready_; }

private:
  std::vector<double> q_start_;
  std::vector<double> q_goal_;
  double T_{1.0};
  size_t n_{0};
  bool   ready_{false};
};

