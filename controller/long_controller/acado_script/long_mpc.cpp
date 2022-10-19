/*
 *  ACADO MPC code generator for kinematic bicycle model
 * http://acado.sourceforge.net/doc/html/d4/d26/example_013.html
 * http://acado.sourceforge.net/doc/html/db/daf/cgt_getting_started.html
 */

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
{
   
    // INTRODUCE THE VARIABLES (acadoVariables.x):
    // -------------------------
    DifferentialState x; // position
    DifferentialState v; // velocity
    DifferentialState a; // acceleration

    Control j;

    const double Ts = 0.16; // 1 step: 4 waypoints
    const int N  = 25;      // 25 * 0.16 = 4 seconds
    const int Ni = 10;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;
    // Model equations
    f << dot(x) == v;
    f << dot(v) == a;
    f << dot(a) == j;

    //  
    // Weighting matrices and reference functions (acadoVariables.y)
    //
    Function rf;
    Function rfN;

    rf  << x << v << a << j;
    rfN << x << v << a;

    // Provide defined weighting matrices:
    BMatrix W = eye<bool>(rf.getDim());
    BMatrix WN = eye<bool>(rfN.getDim());

    OCP ocp(0, N * Ts, N);

    ocp.subjectTo( f );

    // Control constraints
    ocp.subjectTo(0.0 <= x);
    ocp.subjectTo(0.0 <= v);
    ocp.subjectTo(-4.0 <= a <= 3.0);
    ocp.subjectTo(-8.0 <= j <= 8.0);

    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);

    //
    // Export the code:
    //
    OCPexport mpc( ocp );
    
    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);     
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
    mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(HOTSTART_QP, YES);
    mpc.set(GENERATE_TEST_FILE, YES);
    mpc.set(GENERATE_MAKE_FILE, YES);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
    mpc.set(FIX_INITIAL_STATE, YES);

    if (mpc.exportCode( "long_mpc_export" ) != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}



