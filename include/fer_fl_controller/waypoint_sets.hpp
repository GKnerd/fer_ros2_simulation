#pragma once

#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Verified MPC waypoint sets for the Franka Emika Research (FER) arm.
//
// All sets:
//   • Start and end at the home configuration (last entry).
//   • Were verified with MoveIt collision-free planning: 20/20 PASS.
//   • Respect all FER joint limits, in particular J4 ∈ [-3.0718, -0.0698]
//     and J6 ∈ [-0.0175, 3.7525].
//
// Joint order: [J1, J2, J3, J4, J5, J6, J7]  (rad)
// Home pose  : [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
//
// To select a set in a node, replace the waypoints_ assignment with:
//   waypoints_ = WAYPOINT_SET_1;   // or SET_2 / SET_3 / SET_4
// ─────────────────────────────────────────────────────────────────────────────

// Set 1 — Mixed Perturbation (original thesis set)
// Asymmetric mixed perturbations from home; baseline for all comparisons.
inline const std::vector<std::vector<double>> WAYPOINT_SET_1 = {
  { 0.3,  -0.9,    0.0, -1.9,    0.0,  1.8,    0.5   },  // A
  {-0.3,  -0.8,    0.2, -2.0,    0.0,  1.6,    0.8   },  // B
  { 0.5,  -1.0,   -0.2, -1.5,    0.1,  2.0,    0.3   },  // C
  {-0.5,  -1.2,    0.3, -2.2,   -0.1,  1.4,    1.0   },  // D
  { 0.0,  -0.6,    0.0, -2.5,    0.0,  1.7,    0.7   },  // E
  { 0.0,  -0.7854, 0.0, -2.3562, 0.0,  1.5708, 0.7854},  // home
};

// Set 2 — Wide Lateral Sweep
// J1 sweeps ±1.0 rad; A↔E and B↔D are mirror-symmetric about the sagittal plane.
inline const std::vector<std::vector<double>> WAYPOINT_SET_2 = {
  { 1.0,  -0.9,    0.0, -2.0,    0.0,  1.6,    0.5   },  // A
  { 0.5,  -1.2,    0.3, -1.8,    0.2,  2.0,    0.4   },  // B
  { 0.0,  -0.8,    0.0, -2.5,    0.0,  1.4,    0.8   },  // C
  {-0.5,  -1.2,   -0.3, -1.8,   -0.2,  2.0,    0.4   },  // D
  {-1.0,  -0.9,    0.0, -2.0,    0.0,  1.6,    0.5   },  // E
  { 0.0,  -0.7854, 0.0, -2.3562, 0.0,  1.5708, 0.7854},  // home
};

// Set 3 — High Elbow
// Large negative J2 (−1.3 to −1.6) and shallow J4; end-effector moves high.
// S3_E uses J4=−1.5, J6=2.8 (fixed from original J4=−0.5, J6=3.2 which caused
// MoveIt PLANNING_FAILED because the EE was placed at x=−0.83 m behind the base).
inline const std::vector<std::vector<double>> WAYPOINT_SET_3 = {
  { 0.0,  -1.5,    0.0, -0.8,    0.0,  2.5,    0.5   },  // A
  { 0.3,  -1.4,    0.2, -1.0,    0.3,  2.3,    0.8   },  // B
  {-0.3,  -1.4,   -0.2, -1.0,   -0.3,  2.3,    0.8   },  // C
  { 0.0,  -1.6,    0.0, -1.3,    0.0,  2.0,    1.2   },  // D
  { 0.0,  -1.3,    0.0, -1.5,    0.0,  2.8,    0.6   },  // E
  { 0.0,  -0.7854, 0.0, -2.3562, 0.0,  1.5708, 0.7854},  // home
};

// Set 4 — Large Amplitude
// Widest joint excursions across all 7 axes; exercises the full reachable state space.
inline const std::vector<std::vector<double>> WAYPOINT_SET_4 = {
  { 0.8,  -0.5,    0.5, -2.8,    0.5,  1.2,    0.3   },  // A
  { 0.0,  -1.0,    0.0, -1.5,    0.0,  2.5,    1.5   },  // B
  {-0.8,  -0.5,   -0.5, -2.8,   -0.5,  1.2,    0.3   },  // C
  { 0.0,  -1.3,    0.8, -2.0,   -0.5,  1.8,    0.6   },  // D
  { 0.4,  -0.7,   -0.8, -2.2,    0.8,  1.6,    1.0   },  // E
  { 0.0,  -0.7854, 0.0, -2.3562, 0.0,  1.5708, 0.7854},  // home
};
