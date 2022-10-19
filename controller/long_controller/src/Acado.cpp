/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#include "Acado.hpp"

#include <math.h>

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

const double ts_{0.16};
const double e_{2.71828};

void initWeight()
{
    real_t wX = 200.0;
    real_t wV = 10000.0;
    real_t wA = 10.0;
    real_t wJ = 10.0;

    for (int i = 0; i < ACADO_N; i++)
    {
        // Setup diagonal entries
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 0] = wX;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 1] = wV;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 2] = wA;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 3] = wJ;
    }
    acadoVariables.WN[(ACADO_NYN + 1) * 0] = wX;
    acadoVariables.WN[(ACADO_NYN + 1) * 1] = wV;
    acadoVariables.WN[(ACADO_NYN + 1) * 2] = wJ;
}

std::vector<double> initAcado()
{
    // Reset all solver memory
	memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
	memset(&acadoVariables, 0, sizeof(acadoVariables));

    // Initialize the solver
    acado_initializeSolver();

    for (int i = 0; i < (ACADO_N + 1) * ACADO_NX; ++i)
    {
        acadoVariables.x[i] = 0.0;
    }
    for (int i = 0; i < ACADO_N * ACADO_NU; ++i)
    {
        acadoVariables.u[i] = 0.0;
    }
    for (int i = 0; i < ACADO_N * ACADO_NY; ++i)
    {
        acadoVariables.y[i] = 0.0;
    }
    for (int i = 0; i < ACADO_NYN; ++i)
    {
        acadoVariables.yN[i] = 0.0;
    }

    acado_preparationStep();
    std::vector<double> controlOutputAcc;

    for (int i = 0; i < ACADO_N; ++i)
    {
        for (int j = 0; j < ACADO_NU; ++j)
        {
            controlOutputAcc.push_back(acadoVariables.u[i * ACADO_NU + j]);
        }
    }
    initWeight();
    return controlOutputAcc;
}

std::vector<double> updateStates(std::vector<double> state, double jerk)
{
    double x0 = state.at(0);
    double v0 = state.at(1);
    double a0 = state.at(2);

    double x1 = x0 + ts_ * v0;
    double v1 = v0 + ts_ * a0;
    double a1 = a0 + ts_ * jerk;

    return {x1, v1, a1};
}

std::vector<double> motionPrediction(const std::vector<double>& curState,
                                     const std::vector<double>& prevU)
{
    std::vector<double> result;
    std::vector<double> predictX;
    std::vector<double> predictV;
    std::vector<double> predictA;

    predictX.push_back(curState.at(0));
    predictV.push_back(curState.at(1));
    predictA.push_back(curState.at(2));

    for (int i = 0; i < ACADO_N; i++)
    {
        double curX = predictX.at(i);
        double curV = predictV.at(i);
        double curA = predictA.at(i);
        double curJ = prevU.at(i);

        predictX.push_back(curX + ts_ * curV);
        predictV.push_back(curV + ts_ * curA);
        predictA.push_back(curA + ts_ * curJ);
    }

    std::vector<std::vector<double>> predictStates = {predictX, predictV, predictA};

    for (unsigned int i = 0; i < ACADO_N; ++i)
    {
        for (unsigned int j = 0; j < ACADO_NX; ++j)           
        {
            result.push_back(predictStates[j][i]);
        }
    }

    return result;
}

std::vector<double> calculateRefStates(const adas::Trajectory& refTrajectory)
{
    std::vector<double> result;
    std::vector<double> refX;
    std::vector<double> refV;
    std::vector<double> refA;
    std::vector<double> refJ;

    for (unsigned int i = 0; i < ACADO_N; i++)
    {
        if (i == 0)
        {
            refX.push_back(0.0);
            refV.push_back(refTrajectory.trajectory_.at(i * 4).v_);
            refA.push_back(refTrajectory.trajectory_.at(i * 4).a_);
            refJ.push_back(0.0);
        }
        else
        {
            if ((i * 4) < refTrajectory.trajectory_.size())
            {
                refX.push_back(refTrajectory.trajectory_.at(i * 4).x_);
                if (refTrajectory.trajectory_.at(i * 4).v_ < kmh2Mps(1.0))
                {
                    refV.push_back(0.0);
                }
                else
                {
                    refV.push_back(refTrajectory.trajectory_.at(i * 4).v_);
                }
                refA.push_back(refTrajectory.trajectory_.at(i * 4).a_);
                refJ.push_back((refTrajectory.trajectory_.at(i * 4).a_ - refTrajectory.trajectory_.at((i - 1) * 4).a_) / 0.04);
            }
            else
            {
                refX.push_back(refTrajectory.trajectory_.at(refTrajectory.trajectory_.size() - 1).x_);
                if (refTrajectory.trajectory_.at(refTrajectory.trajectory_.size() - 1).v_ < kmh2Mps(1.0))
                {
                    refV.push_back(0.0);
                }
                else
                {
                    refV.push_back(refTrajectory.trajectory_.at(refTrajectory.trajectory_.size() - 1).v_);
                }
                refA.push_back(refTrajectory.trajectory_.at(refTrajectory.trajectory_.size() - 1).a_);
                refJ.push_back(0);
            }
        }
        // std::cout << "Reference: " << refX[i] << ", " << refV[i] << ", " << refA[i] << ", " << refJ[i] << std::endl;      
    }

    std::vector<std::vector<double>> refStates = {refX, refV, refA, refJ};

    for (unsigned int i = 0; i < ACADO_N; ++i)
    {
        for (unsigned int j = 0; j < ACADO_NY; ++j)          
        {
            result.push_back(refStates[j][i]);
        }
    }

    return result;
}

std::vector<double> runMpcAcado(std::vector<double> states,
                                std::vector<double> refStates)
{
    unsigned int i, j;
    int iter;

    for (i = 0; i < (ACADO_N + 1) * ACADO_NX; ++i)  
    {
        if (i < states.size())
        {
            acadoVariables.x[ i ] = (real_t) states[i];
        }
    }
    for (i = 0; i < ACADO_NX; ++i)
    {
        if (i < states.size())
        {
            acadoVariables.x0[i] = (real_t)states[i];
        }
    }

    // Initialize the measurements/reference.
    for (i = 0; i < ACADO_N * ACADO_NY; ++i)
    {
        if (i < refStates.size())
        {
            acadoVariables.y[i] = (real_t)refStates[i];
        }
    }
    for (i = 0; i < ACADO_NYN; ++i)
    {
        if ((ACADO_NY * (ACADO_N - 1) + i) < refStates.size())
        {
            acadoVariables.yN[i] = (real_t)refStates[ACADO_NY * (ACADO_N - 1) + i];
        }
    }

    // Prepare first step
    acado_preparationStep();

    for(iter = 0; iter < 10; ++iter)
	{
        /* Perform the feedback step. */
		acado_feedbackStep( );
        
		/* Prepare for the next step. */
		acado_preparationStep();
	}

    std::vector<double> controlOutputAcc;

    // real_t *u = acado_getVariablesU();
    // for (int i = 0; i < ACADO_N; ++i)
    // {
    //     for (int j = 0; j < ACADO_NU; ++j)
    //     {
    //         controlOutputAcc.push_back((double)u[i * ACADO_NU + j]);
    //     }
    // }

    for (i = 0; i < ACADO_N + 1; ++i)
	{
		for (j = 0; j < ACADO_NX; ++j)
        {
            if (j == ACADO_NX - 1)
            {
                if ((i * ACADO_NX + j) < (sizeof(acadoVariables.x) / sizeof(acadoVariables.x[0])))
                {
                    controlOutputAcc.push_back(acadoVariables.x[i * ACADO_NX + j]);
                }
            }
        }
	}

    // acado_printDifferentialVariables();
	// acado_printControlVariables();

    return controlOutputAcc;
}