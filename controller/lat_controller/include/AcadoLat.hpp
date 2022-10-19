/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */


#ifndef MPC_ACADO_LAT_H
#define MPC_ACADO_LAT_H

#include "adas/common/util/trajectory.h"

#include "../acado_mpc_export/acado_common.h"
#include "../acado_mpc_export/acado_auxiliary_functions.h"

#include <vector>

#include <stdio.h>
#include <string.h>

/* Some convenient definitions. */
// ACADO_NX  /* Number of differential state variables.  */
// ACADO_N   /* Number of control in the horizon time. */
// ACADO_NXA /* Number of algebraic variables. */
// ACADO_NU  /* Number of control inputs. */
// ACADO_NP  /* Number of parameters. */

// ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
// ACADO_NYN /* Number of measurements/references on node N. */

// #define ACADO_N           10        /* Number of intervals in the horizon. */

// #define NUM_STEPS   5         /* Number of real-time iterations. */
// #define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

#define NUM_STEPS   10         /* Number of real-time iterations. */



void mpcLatSetParam(double wX_, double wY_, double wYaw_, double wDelta_, 
                    double wDeltarate_, double L_f_, double L_r_);
std::vector<double> initLatAcado();
std::vector<double> resetControl();
std::vector<double> motionLatPrediction(const std::vector<double>& curState,
                                        const std::vector<double>& prevU, double vX,
                                        const adas::Trajectory& refTrajectory);
std::vector<double> updateLatStates(std::vector<double> state, 
                                    double deltaRate, double vX);   
std::vector<double> calculateLatRefStatesPolyfit(const Eigen::VectorXd &coeff,
                                                 const double &refV);
std::vector<double> calculateLatRefStates(const adas::Trajectory& refTrajectory);                                       
std::vector<double> runLatMpcAcado(std::vector<double> states,
                                   std::vector<double> refStates,
                                   double vX, const adas::Trajectory& refTrajectory,
                                   std::vector<double> preSteering);
#endif /* MPC_ACADO_LAT_H */