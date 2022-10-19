
#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main(int argc, char * const argv[])
{
  DifferentialState x;
  DifferentialState y;
  DifferentialState yaw;

  Control delta;  
  Control v;
  
  OnlineData L_f;
  OnlineData L_r;
  OnlineData preDelta;
  OnlineData preV;

// Kinematic bicycle model:
  IntermediateState beta = atan((L_r * tan(delta)) / (L_f + L_r));

  DifferentialEquation f;

  f << dot(x) == v * cos(yaw + beta) / cos(beta) - preDelta + preDelta;
  f << dot(y) == v * sin(yaw + beta) / cos(beta) - preV + preV;
  f << dot(yaw) == (v * tan(delta)) / (L_r + L_f);

  Function rf;
  Function rfN;
  rf  << x << y << yaw << delta << (delta - preDelta) << v << (v - preV);
  rfN << x << y << yaw;

  BMatrix W = eye<bool>(rf.getDim());
  BMatrix WN = eye<bool>(rfN.getDim());

  // Optimal Control Problem

  const int N = 25;  // Number of steps
  const int Ni = 10;  // number of integrators
  const double Ts = 0.04;

  
  OCP ocp(0, N * Ts, N);

  ocp.subjectTo(f);
  ocp.subjectTo(-0.523 <= delta <= 0.523);
  ocp.subjectTo(-0.26 <= delta - preDelta <= 0.26);
  ocp.subjectTo(-1.9444 <= v <= 1.9444);
  ocp.subjectTo( -0.5 <= v - preV <= 0.5);
 
  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);

  

  //
  // Export the code:
  //
  OCPexport mpc(ocp);

  // See https://github.com/cho3/acado/blob/master/acado/utils/acado_types.hpp#L331..L427
  // for all options

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

  mpc.set(INTEGRATOR_TYPE, INT_RK45);
  mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);

  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
  // mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(MAX_NUM_QP_ITERATIONS, 999);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  // mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
  // mpc.set(QP_SOLVER, QP_FORCES);

  mpc.set(LEVENBERG_MARQUARDT, 1.0E2);

  mpc.set(GENERATE_TEST_FILE, YES);
  mpc.set(GENERATE_MAKE_FILE, YES);
  mpc.set(GENERATE_MATLAB_INTERFACE, YES);
  //mpc.set(GENERATE_SIMULINK_INTERFACE, YES);

  // mpc.set(USE_SINGLE_PRECISION, YES);
  // mpc.set(CG_USE_OPENMP, YES);

  if (mpc.exportCode("acado_mpc_export") != SUCCESSFUL_RETURN) {
    exit(EXIT_FAILURE);
  }

  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}

