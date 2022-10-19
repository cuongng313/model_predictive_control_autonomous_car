/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#include "AcadoUnified.hpp"

#include <math.h>

/* Global variables used by the solver/ */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

const double ts_{0.04};
const double td_{0.001};

// DifferentialState weight
double wX{0};
double wY{0};
double wYaw{0};


// Control signal weight
double wDelta{0};
double wDeltarate{0};
double wV{0};
double wVRate{0};
double k{1.0};

double L_f{1.425};
double L_r{1.425};

void setParam(double wX_, double wY_, double wYaw_, double wDelta_, 
                    double wDeltarate_, double wV_, double wVRate_,
                    double L_f_, double L_r_, double k_)
{
    wX = wX_;
    wY = wY_;
    wYaw = wYaw_;

    wDelta = wDelta_;
    wDeltarate = wDeltarate_;
    wV = wV_;
    wVRate = wVRate_;
    k = k_;

    L_f = L_f_;
    L_r = L_r_;
}

void initUnifiedWeight()
{
    for (int i = 0; i < ACADO_N; i++)
    {
		// Setup diagonal entries
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 0] = wX;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 1] = wY;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 2] = wYaw;

        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 4] = wDelta;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 5] = wDeltarate;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 6] = wV;
        acadoVariables.W[ACADO_NY * ACADO_NY * i + (ACADO_NY + 1) * 7] = wVRate;
    
    }
    
    acadoVariables.WN[(ACADO_NYN + 1) * 0] = wX * k;
    acadoVariables.WN[(ACADO_NYN + 1) * 1] = wY * k;
    acadoVariables.WN[(ACADO_NYN + 1) * 2] = wYaw * k;

    std::cout << " W = " << std::endl;
    for (int i = 0; i <  ACADO_NY * (ACADO_N - 1); i++)
    {
        
        std::cout << acadoVariables.W[7*i] << "    " 
                  << acadoVariables.W[7*i + 1] << "    "
                  << acadoVariables.W[7*i + 2] << "    "
                  << acadoVariables.W[7*i + 3] << "    "
                  << acadoVariables.W[7*i + 4] << "    "
                  << acadoVariables.W[7*i + 5] << "    "
                  << acadoVariables.W[7*i + 6] << std::endl;  
    }
}

std::vector<std::vector<double>> resetControl()
{
    for (int i = 0; i < ACADO_NX * (ACADO_N + 1); ++i)
    {
        acadoVariables.x[i] = 0.0;                // Initialize both state and control variables into one vector 
    }
    for (int i = 0; i < ACADO_NX; ++i)
    {
        acadoVariables.x0[i] = 0.0;                // Initialize both state and control variables into one vector 
    }
    for (int i = 0; i < ACADO_NU * ACADO_N; ++i)
    {
        acadoVariables.u[i] = 0.0;                // Initialize control variable in horizon into one vector
    }
    for (int i = 0; i < ACADO_NY * ACADO_N; ++i)
    {
        acadoVariables.y[i] = 0.0;
    }
    for (int i = 0; i < ACADO_NYN; ++i)
    {
        acadoVariables.yN[i] = 0.0;
    }

    acado_preparationStep();

    std::vector<double> controlOutputSteerAngle;
	std::vector<double> controlOutputVelocity;

	for (int i = 0; i < ACADO_N; ++i)
	{
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				controlOutputSteerAngle.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
			else 
			{
				controlOutputVelocity.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
		}
	}
    return {controlOutputSteerAngle, controlOutputVelocity};
}

std::vector<std::vector<double>> initUnifiedAcado()
{
    // Initialize the solver
    acado_initializeSolver();

    //Initialize the variable vector
    for (int i = 0; i < ACADO_NX * (ACADO_N + 1); ++i)
    {
        acadoVariables.x[i] = 0.0;                // Initialize both state and control variables into one vector 
    }
    for (int i = 0; i < ACADO_NX; ++i)
    {
        acadoVariables.x0[i] = 0.0;                // Initialize both state and control variables into one vector 
    }
    for (int i = 0; i < ACADO_NU * ACADO_N; ++i)
    {
        acadoVariables.u[i] = 0.0;                // Initialize control variable in horizon into one vector
    }
    for (int i = 0; i < ACADO_NY * ACADO_N; ++i)
    {
        acadoVariables.y[i] = 0.0;
    }
    for (int i = 0; i < ACADO_NYN; ++i)
    {
        acadoVariables.yN[i] = 0.0;
    }

    acado_preparationStep();
    std::vector<double> controlOutputSteerAngle;
	std::vector<double> controlOutputVelocity;

	for (int i = 0; i < ACADO_N; ++i)
	{
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				controlOutputSteerAngle.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
			else 
			{
				controlOutputVelocity.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
		}
	}
    initUnifiedWeight();
    return {controlOutputSteerAngle, controlOutputVelocity};
}

std::vector<double> updateUnifiedStates(std::vector<double> state, double preSteer, double preVel)
{
    double x0 = state.at(0);
    double y0 = state.at(1);
    double yaw0 = state.at(2);

    double beta =  atan((L_r * tan(preSteer)) / (L_f + L_r));

    // std::cout  << "Beta angle: " << beta << std::endl;

    // kinematic model considering the side slip angle and use the velocity of the rear wheel
    double x1 = x0 + ts_ * preVel * cos(yaw0 + beta) / cos(beta);
    double y1 = y0 + ts_ * preVel * sin(yaw0 + beta) / cos(beta);
    double yaw1 = yaw0 + ts_ * (preVel * tan(preSteer)) / (L_f + L_r);

    // for (int i = 0; i < ts_/td_; i++)
    // {
    //     x0 = x0 + td_ * vX * cos(yaw0 + beta) / cos(beta);
    //     y0 = y0 + td_ * vX * sin(yaw0 + beta) / cos(beta);
    //     yaw0 = yaw0 + td_ * (vX * tan(delta0)) / (L_f + L_r); 
    // }

    return {x1, y1, yaw1};
}

std::vector<double> motionUnifiedPrediction(const std::vector<double>& curState,
                                    const std::vector<std::vector<double>>& prevU)
{
    std::vector<double> preSteeringAngle = prevU[0];
    std::vector<double> preVelocity = prevU[1];
	
    std::vector<std::vector<double>> predictedStates;
    std::vector<double> result;
    
    predictedStates.push_back(curState);

    for (int i = 0; i < ACADO_N; i++)
    {
        std::vector<double> state = predictedStates.at(i);

        // double yawTmp = state.at(2);
        // double vXtmp = state.at(3);

        // if (yawTmp > M_PI)
		// {
		// 	yawTmp -= 2. * M_PI;
		// }
		// if (yawTmp < -M_PI)
		// {
		// 	yawTmp += 2. * M_PI;
		// }

        // state.pop_back();
        // state.push_back(yawTmp);

        std::vector<double> nextState = updateUnifiedStates(state, preSteeringAngle.at(i), preVelocity.at(i));
        predictedStates.push_back(nextState);
    }
    for (int i = 0; i < (ACADO_N + 1); i++)
    {
        for (int j = 0; j < ACADO_NX; j++)         
        {
            result.push_back(predictedStates[i][j]);
        }
    }
    return result;
}

std::vector<double> calculateUnifiedRefStates(const adas::Trajectory& refTrajectory)
{
    std::vector<double> result;
    std::vector<double> refX;
    std::vector<double> refY;
    std::vector<double> refYaw;

    std::vector<double> refDelta;
    std::vector<double> refDeltarate;
    std::vector<double> refV;
    std::vector<double> refVRate;

    for (unsigned int i = 0; i < ACADO_N; i++)
    {
        double theta = refTrajectory.trajectory_.at(i+1).theta_;
        refX.push_back(refTrajectory.trajectory_.at(i+1).x_);
        refY.push_back(refTrajectory.trajectory_.at(i+1).y_);
        refYaw.push_back(theta);

        // refDelta.push_back(0); 
        refDelta.push_back(refTrajectory.trajectory_.at(i+1).vtheta_);
        refDeltarate.push_back(0);

        refV.push_back(refTrajectory.trajectory_.at(i).v_);
        refVRate.push_back(0); 
    }

    std::vector<std::vector<double>> refStates = {refX, refY, refYaw, refDelta, refDeltarate, refV, refVRate};

    for (unsigned int i = 0; i < ACADO_N; ++i)
    {
        for (unsigned int j = 0; j < ACADO_NY; ++j)          
        {
            result.push_back(refStates[j][i]);
        }
    }

    return result;
}

std::vector<std::vector<double>> runUnifiedMpcAcado(std::vector<double> states, 
                                                    std::vector<double> refStates,
                                            std::vector<std::vector<double>> preControl,
                                            const adas::Trajectory& refTrajectory)
{
    int i, iter;
    acado_timer t;

    //Setup online data variable for ACADO
    for (int i = 0; i < ACADO_N; i++)
    {       
        acadoVariables.od[ACADO_NOD*i] = L_f;
        acadoVariables.od[ACADO_NOD*i + 1] = L_r;
        acadoVariables.od[ACADO_NOD*i + 2] = preControl[0][i];
        acadoVariables.od[ACADO_NOD*i + 3] = preControl[1][i];
     }
    acadoVariables.od[ACADO_NOD*ACADO_N] = L_f;
    acadoVariables.od[ACADO_NOD*ACADO_N + 1] = L_r; 
    acadoVariables.od[ACADO_NOD*ACADO_N + 2] = preControl[0][ACADO_N - 1];
    acadoVariables.od[ACADO_NOD*ACADO_N + 3] = preControl[1][ACADO_N - 1];

    // Update the states
    for (i = 0; i < ACADO_NX * (ACADO_N + 1); ++i)  
    {
        acadoVariables.x[i] = (real_t) states[i];
    }
    for (i = 0; i < ACADO_NX; ++i)
    {
        acadoVariables.x0[i] = (real_t)states[i];
    }

    // Initialize the measurements/reference.
    for (i = 0; i < ACADO_NY * ACADO_N; ++i)
    {
        acadoVariables.y[i] = (real_t)refStates[i];
    }
    for (i = 0; i < ACADO_NYN; ++i)
    {
        acadoVariables.yN[i] = (real_t)refStates[ACADO_NY * (ACADO_N - 1) + i];
    }


    // std::cout << "[delta, preDelta, delta - preDelta]" << std::endl;
    // for (int i = 0; i < ACADO_N; i++)
    // {
    //     std::cout <<  i << "    "
    //               << acadoWorkspace.Dy[5*i + 3] << "    "
    //               << preSteering.at(i) << "    "
    //               << acadoWorkspace.Dy[5*i + 4]  << std::endl;
    // }

    // std::cout << "Vx: " << vX << std::endl;

   

    // std::cout << "[ x, xRef, y, yRef, yaw, yawRef, velocity, desired velocity]" << std::endl;
    // for (int i = 0; i < ACADO_N; i ++)
    // {
    //     std::cout << i << "   ";
    //     std::cout <<  acadoVariables.x[3*i] << "  "
    //               <<  acadoVariables.y[7*i] << "  "
    //               <<  acadoVariables.x[3*i + 1] << "  "
    //               <<  acadoVariables.y[7*i + 1] << "  "
    //               <<  acadoVariables.x[3*i + 2]  << "  "
    //               <<  acadoVariables.y[7*i + 2]  << "  "
    //               <<  preControl[1][i]  << "   "
    //               <<  refTrajectory.trajectory_.at(i).v_  <<  std::endl;

    //     // std::cout << i << "   ";
    //     // std::cout <<  acadoVariables.y[8*i] << "    "
    //     //           <<  acadoVariables.y[8*i + 1] << "   "
    //     //           <<  acadoVariables.y[8*i + 2]  << "   "
    //     //           <<  acadoVariables.y[8*i + 3]  << std::endl;

    //     // std::cout << i << "   ";
    //     // std::cout <<  acadoVariables.x[4*i] - acadoVariables.y[8*i] << "    "
    //     //           <<  acadoVariables.x[4*i + 1] - acadoVariables.y[8*i + 1] << "   "
    //     //           <<  acadoVariables.x[4*i + 2] - acadoVariables.y[8*i + 2]  << "   "
    //     //           <<  acadoVariables.x[4*i + 3] - acadoVariables.y[8*i + 3] << std::endl;

    // }

    // std::cout << "error data: [y, yaw]" << std::endl;
    // for (int i = 0; i < ACADO_N; i ++)
    // {
    //     std::cout << i << "  " << acadoVariables.x[4*i + 1] - acadoVariables.y[5*i + 1] << "    "
    //               << acadoVariables.x[4*i + 2] - acadoVariables.y[5*i + 2] << std::endl;
    // }

    // Prepare first step
    acado_preparationStep();

    // Get the time before start of the loop
    acado_tic(&t);

    for(iter = 0; iter < NUM_STEPS; ++iter)
	{
        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */    

		// printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
        
		/* Prepare for the next step. */
		acado_preparationStep();
	}

    // acado_shiftControls(0);
    // acado_shiftControls(0);

    // Perform the feedback step
    // acado_feedbackStep();

    // printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT());

    
    // Read the elapsed time
    real_t te = acado_toc(&t);
    real_t Jcost = acado_getObjective();

    std::cout << "Objective value: " << Jcost << std::endl;

    // printf("\n\nEnd of the RTI loop. \n\n\n");

    // Eye-candy
    // printf("\n\n Average time of real-time iteration:   %.3g microseconds\n\n", 1e6 * te);

    std::vector<double> controlOutputSteerAngle;
    std::vector<double> controlOutputVelocity;

    real_t *u = acado_getVariablesU();
    for (int i = 0; i < ACADO_N; ++i)
    {
        for (int j = 0; j < ACADO_NU; ++j)
        {
            if (j == 0)
            {
                controlOutputSteerAngle.push_back((double)u[i * ACADO_NU + j]);
            }
            else
            {
                controlOutputVelocity.push_back((double)u[i * ACADO_NU + j]);
            }
            
        }
    }

    std::cout << "[ yaw, yawRef,error, preSteer, Steer, refSteer]" << std::endl;
    for (int i = 0; i < ACADO_N; i ++)
    {
        std::cout << i << "   ";
        std::cout <<  acadoVariables.x[3*i + 2] << "    "
                  <<  acadoVariables.y[7*i + 2] << "   "
                  << acadoVariables.x[3*i + 2] - acadoVariables.y[7*i + 2] << "   "
                  << preControl[0][i] << "    " 
                  << controlOutputSteerAngle.at(i) << "    "
                  << refTrajectory.trajectory_.at(i + 1).vtheta_ << std::endl;
    }

    return {controlOutputSteerAngle, controlOutputVelocity};
}
