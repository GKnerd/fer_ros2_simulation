#include "fer_fl_controller/casadi_qp_wrapper_obstacle.h"

#include <casadi/casadi.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

struct CasadiQPStateObs
{
  casadi::Function solver;
};

static casadi::Function build_qp_solver()
{
  namespace cs = casadi;

  cs::MX v = cs::MX::sym("v", 7);

  // Parameter vector (105 elements); see casadi_qp_wrapper_obstacle.h for layout
  cs::MX p = cs::MX::sym("p", 105);

  cs::MX q0       = p(cs::Slice(0,   7));
  cs::MX dq0      = p(cs::Slice(7,  14));
  cs::MX v_prev   = p(cs::Slice(14, 21));
  cs::MX q_ref    = p(cs::Slice(21, 28));
  cs::MX dq_ref   = p(cs::Slice(28, 35));
  cs::MX ddq_ref  = p(cs::Slice(35, 42));
  cs::MX M_flat   = p(cs::Slice(42, 91));
  cs::MX C_vec    = p(cs::Slice(91, 98));
  cs::MX tau_fric = p(cs::Slice(98, 105));

  // Rebuild M from column-major flat vector to 7×7
  cs::MX M = cs::MX::reshape(M_flat, 7, 7);

  // Cost weights — identical to brute_decoupled for fair benchmark comparison
  const double dt    = 0.01;
  const int    N     = 15;
  const double q_w   = 100.0;
  const double dq_w  = 1.0;
  const double u_w   = 0.01;
  const double P_q   = 50000.0;
  const double P_dq  = 300.0;

  cs::MX E_q  = q_ref  - q0;
  cs::MX E_dq = dq_ref - dq0;

  // Friction-aware model: per-joint alpha_j = mu_viscous / M_jj
  const double mu_v = 16.0;
  cs::MX M_diag = cs::MX::vertcat({
    M_flat(0),  M_flat(8),  M_flat(16), M_flat(24),
    M_flat(32), M_flat(40), M_flat(48)
  });
  cs::MX alpha_j = mu_v / M_diag;
  cs::MX beta_j  = 1.0 / alpha_j;

  // Accumulate cost over the prediction horizon
  cs::MX cost = cs::MX(0.0);
  for (int k = 1; k <= N; ++k) {
    const double t  = k * dt;
    const double t2 = t * t;

    cs::MX eat_j   = cs::MX::exp(-alpha_j * t);
    cs::MX omeat_j = 1.0 - eat_j;

    cs::MX g_q_j  = beta_j * (t - beta_j * omeat_j);
    cs::MX g_dq_j = beta_j * omeat_j;

    cs::MX a_k = E_q + E_dq * t + 0.5 * ddq_ref * t2
                 + dq0 * (t - beta_j * omeat_j);
    cs::MX b_k = E_dq + ddq_ref * t + dq0 * omeat_j;

    cs::MX e_q  = a_k - g_q_j  * v;
    cs::MX e_dq = b_k - g_dq_j * v;

    const double wq  = (k == N) ? (q_w + P_q)   : q_w;
    const double wdq = (k == N) ? (dq_w + P_dq) : dq_w;

    cost = cost + wq  * cs::MX::dot(e_q,  e_q)
                + wdq * cs::MX::dot(e_dq, e_dq);
  }

  // Control rate penalty
  cs::MX dv = v - v_prev;
  cost = cost + u_w * cs::MX::dot(dv, dv);

  // Hard torque constraint: tau_min ≤ M·v + C + tau_fric ≤ tau_max
  cs::MX tau_cons = cs::MX::mtimes(M, v) + C_vec + tau_fric;

  cs::MXDict qp_dict = {{"x", v}, {"p", p}, {"f", cost}, {"g", tau_cons}};

  cs::Dict opts;
  opts["print_time"] = false;
  opts["verbose"]    = false;

  return cs::qpsol("qp_obstacle", "osqp", qp_dict, opts);
}

// ── C interface implementation ─────────────────────────────────────────────────

extern "C" {

CasadiQPHandle casadi_qp_obstacle_create(const char * casadi_path)
{
  try {
    if (casadi_path && casadi_path[0] != '\0') {
      casadi::GlobalOptions::setCasadiPath(std::string(casadi_path));
    }

    auto * state = new CasadiQPStateObs{};
    state->solver = build_qp_solver();
    return static_cast<CasadiQPHandle>(state);
  } catch (...) {
    return nullptr;
  }
}

void casadi_qp_obstacle_destroy(CasadiQPHandle handle)
{
  delete static_cast<CasadiQPStateObs *>(handle);
}

int casadi_qp_obstacle_solve(
  CasadiQPHandle   handle,
  const double   * p_vec,
  const double   * tau_min,
  const double   * tau_max,
  const double   * v_init,
  double         * v_out,
  double         * solve_time_ms_out,
  double         * cost_out)
{
  for (int i = 0; i < 7; ++i) v_out[i] = 0.0;
  *solve_time_ms_out = 0.0;
  *cost_out = 0.0;

  if (!handle) return -1;
  auto * state = static_cast<CasadiQPStateObs *>(handle);

  auto t0 = std::chrono::high_resolution_clock::now();

  try {
    std::vector<double> p_v(p_vec,   p_vec   + 105);
    std::vector<double> lbg(tau_min, tau_min + 7);
    std::vector<double> ubg(tau_max, tau_max + 7);
    std::vector<double> x0(v_init,  v_init  + 7);

    casadi::DMDict result = state->solver(casadi::DMDict{
      {"x0",  casadi::DM(x0)},
      {"p",   casadi::DM(p_v)},
      {"lbg", casadi::DM(lbg)},
      {"ubg", casadi::DM(ubg)}
    });

    auto t1 = std::chrono::high_resolution_clock::now();
    *solve_time_ms_out =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

    casadi::Dict s = state->solver.stats();
    bool ok = s.count("success") != 0 &&
              static_cast<bool>(s.at("success"));

    if (ok) {
      const std::vector<double> & nz = result.at("x").nonzeros();
      bool finite = true;
      for (int i = 0; i < 7; ++i) {
        if (!std::isfinite(nz[i])) { finite = false; break; }
      }
      if (finite) {
        for (int i = 0; i < 7; ++i) v_out[i] = nz[i];
        *cost_out = static_cast<double>(result.at("f").scalar());
        return 0;
      }
    }
  } catch (...) {
    auto t1 = std::chrono::high_resolution_clock::now();
    *solve_time_ms_out =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  }

  return -1;
}

}
