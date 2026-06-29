
#ifndef FER_FL_CONTROLLER__CASADI_QP_WRAPPER_OBSTACLE_H
#define FER_FL_CONTROLLER__CASADI_QP_WRAPPER_OBSTACLE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void * CasadiQPHandle;

CasadiQPHandle casadi_qp_obstacle_create(const char * casadi_path);
void casadi_qp_obstacle_destroy(CasadiQPHandle handle);
int casadi_qp_obstacle_solve(
    CasadiQPHandle handle,
    const double * p_vec,
    const double * tau_min,
    const double * tau_max,
    const double * v_init,
    double       * v_out,
    double       * solve_time_ms_out,
    double       * cost_out);

#ifdef __cplusplus
}
#endif

#endif
