
#ifndef FER_FL_CONTROLLER__CASADI_QP_WRAPPER_H
#define FER_FL_CONTROLLER__CASADI_QP_WRAPPER_H // prevents the header from being included multiple times in the same translation unit

#ifdef __cplusplus
extern "C" {
#endif   // as the casadi solver is compiled as C code, we need to prevent name mangling, compiler settings and ABI incompatibilities when including this header in C++ code so communicte with the solver via a C interface

typedef void * CasadiQPHandle; // Opaque handle to the solver state. The actual structure is defined in the implementation file and is not exposed here. so ROS does not need to know about casadi types or memory management details, and we can change the implementation without affecting users of this header.

CasadiQPHandle casadi_qp_coupled_create(const char * casadi_path);// Creates and initializes the QP solver. The casadi_path argument can be used to specify the path to the Casadi installation if needed, or it can be nullptr or empty to use the default path. Returns a handle to the solver state, or nullptr on failure and do not rebuild the solver if it already exists.

void casadi_qp_coupled_destroy(CasadiQPHandle handle);// Destroys the solver instance and frees any associated resources. 
int casadi_qp_coupled_solve(  // Solves the QP problem with the given parameters and initial guess. The p_vec argument is a pointer to an array of 105 doubles containing the problem data (cost matrix, constraint matrices, etc.) in a specific order defined by the solver. The tau_min and tau_max arguments are pointers to arrays of 7 doubles specifying the lower and upper bounds on the control inputs (torques). The v_init argument is a pointer to an array of 7 doubles providing an initial guess for the optimization variables (joint accelerations). The v_out argument is a pointer to an array of 7 doubles where the optimal solution will be written. The solve_time_ms_out and cost_out arguments are pointers to doubles where the solve time in milliseconds and the optimal cost value will be written, respectively. Returns 0 on success, or a non-zero error code on failure.
    CasadiQPHandle handle,// the already-built CasADi QP.
    const double * p_vec,// contains the problem data (cost matrix, constraint matrices, etc.) in a specific order defined by the solver. The exact format and meaning of the values in this array must match what the solver expects, and it is the responsibility of the caller to ensure this. The array should have a length of 105 doubles, which corresponds to the number of parameters required by the solver for the specific QP formulation used in this application.
    const double * tau_min,
    const double * tau_max,
    const double * v_init,// initial guess (from previous cycle).
    double       * v_out,// optimal solution (joint accelerations).
    double       * solve_time_ms_out,// QP solve time in milliseconds.
    double       * cost_out);// final MPC cost.

#ifdef __cplusplus
}
#endif

#endif  
