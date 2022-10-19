/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#ifndef MPC_LOWPASS_H
#define MPC_LOWPASS_H

namespace adas
{

namespace dds
{

namespace mpc
{

class LowPass 
{
public:
    LowPass() : ready_(false), last_val_(0) { a_ = 1; b_ = 0; }
    LowPass(double tau, double ts) : ready_(false), last_val_(0) { setParams(tau, ts); }
    void setParams(double tau, double ts)
    {
        a_ = 1 / (tau / ts + 1);
        b_ = tau / ts / (tau / ts + 1);
    }

    double get() { return last_val_; }
    double filt(double val)
    {
        if (ready_) 
        {
            val = a_ * val + b_ * last_val_;
        } 
        else 
        {
            ready_ = true;
        }
        last_val_ = val;
        return val;
    }
private:
    bool ready_;
    double a_;
    double b_;
    double last_val_;
};

} // mpc

} // dds

} // adas

#endif /* MPC_LOWPASS_H */
