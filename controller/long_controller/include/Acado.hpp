/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#ifndef MPC_ACADO_LONG_H
#define MPC_ACADO_LONG_H

#include "adas/common/util/trajectory.h"

#include "../acado_mpc_export/acado_common.h"
#include "../acado_mpc_export/acado_auxiliary_functions.h"

#include <vector>

#include <stdio.h>

inline double constrain(double amt, double low, double high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}
inline double kmh2Mps(double kmh)
{
    return ((kmh * 1000.0) / 3600.0);
}

inline double mps2Kmh(double mps)
{
    return ((mps * 3600.0) / 1000.0);
}

inline double deg2Rad(double deg)
{
    return ((deg * M_PI) / 180.0);
}

inline double rad2Deg(double rad)
{
    return ((rad * 180.0) / M_PI);
}

std::vector<double> initAcado();
std::vector<double> motionPrediction(const std::vector<double>& curStates,
                                     const std::vector<double>& prevU);
std::vector<double> updateStates(std::vector<double> state, double jerk);
std::vector<double> calculateRefStates(const adas::Trajectory& refTrajectory);
std::vector<double> runMpcAcado(std::vector<double> states,
                                std::vector<double> refStates);

#endif /* MPC_ACADO_LONG_H */