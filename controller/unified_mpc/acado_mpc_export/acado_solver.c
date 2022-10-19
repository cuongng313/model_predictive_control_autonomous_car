/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 3 + 2];

acadoWorkspace.state[18] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[19] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[20] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.state[21] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.state[22] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.state[23] = acadoVariables.od[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 3] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 3 + 3];
acadoWorkspace.d[lRun1 * 3 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 3 + 4];
acadoWorkspace.d[lRun1 * 3 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 3 + 5];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[11];

acadoWorkspace.evGu[lRun1 * 6] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 6 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGu[lRun1 * 6 + 2] = acadoWorkspace.state[14];
acadoWorkspace.evGu[lRun1 * 6 + 3] = acadoWorkspace.state[15];
acadoWorkspace.evGu[lRun1 * 6 + 4] = acadoWorkspace.state[16];
acadoWorkspace.evGu[lRun1 * 6 + 5] = acadoWorkspace.state[17];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
const real_t* od = in + 5;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = u[0];
out[4] = (u[0]-od[2]);
out[5] = u[1];
out[6] = (u[1]-od[3]);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[7];
tmpQ1[4] = + tmpQ2[8];
tmpQ1[5] = + tmpQ2[9];
tmpQ1[6] = + tmpQ2[14];
tmpQ1[7] = + tmpQ2[15];
tmpQ1[8] = + tmpQ2[16];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[21] +tmpObjS[28];
tmpR2[1] = +tmpObjS[22] +tmpObjS[29];
tmpR2[2] = +tmpObjS[23] +tmpObjS[30];
tmpR2[3] = +tmpObjS[24] +tmpObjS[31];
tmpR2[4] = +tmpObjS[25] +tmpObjS[32];
tmpR2[5] = +tmpObjS[26] +tmpObjS[33];
tmpR2[6] = +tmpObjS[27] +tmpObjS[34];
tmpR2[7] = +tmpObjS[35] +tmpObjS[42];
tmpR2[8] = +tmpObjS[36] +tmpObjS[43];
tmpR2[9] = +tmpObjS[37] +tmpObjS[44];
tmpR2[10] = +tmpObjS[38] +tmpObjS[45];
tmpR2[11] = +tmpObjS[39] +tmpObjS[46];
tmpR2[12] = +tmpObjS[40] +tmpObjS[47];
tmpR2[13] = +tmpObjS[41] +tmpObjS[48];
tmpR1[0] = + tmpR2[3] + tmpR2[4];
tmpR1[1] = + tmpR2[5] + tmpR2[6];
tmpR1[2] = + tmpR2[10] + tmpR2[11];
tmpR1[3] = + tmpR2[12] + tmpR2[13];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 25; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[runObj * 4];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 4 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 4 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 7] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 7 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 7 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 7 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 7 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 7 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 7 + 6] = acadoWorkspace.objValueOut[6];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.Q1[ runObj * 9 ]), &(acadoWorkspace.Q2[ runObj * 21 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 14 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[75];
acadoWorkspace.objValueIn[1] = acadoVariables.x[76];
acadoWorkspace.objValueIn[2] = acadoVariables.x[77];
acadoWorkspace.objValueIn[3] = acadoVariables.od[100];
acadoWorkspace.objValueIn[4] = acadoVariables.od[101];
acadoWorkspace.objValueIn[5] = acadoVariables.od[102];
acadoWorkspace.objValueIn[6] = acadoVariables.od[103];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] += + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[3] + Gx1[2]*Gx2[6];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[7];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[8];
Gx3[3] = + Gx1[3]*Gx2[0] + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[6];
Gx3[4] = + Gx1[3]*Gx2[1] + Gx1[4]*Gx2[4] + Gx1[5]*Gx2[7];
Gx3[5] = + Gx1[3]*Gx2[2] + Gx1[4]*Gx2[5] + Gx1[5]*Gx2[8];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[3] + Gx1[8]*Gx2[6];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[4] + Gx1[8]*Gx2[7];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[5] + Gx1[8]*Gx2[8];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5];
Gu2[2] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[2] + Gx1[5]*Gu1[4];
Gu2[3] = + Gx1[3]*Gu1[1] + Gx1[4]*Gu1[3] + Gx1[5]*Gu1[5];
Gu2[4] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[8]*Gu1[4];
Gu2[5] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[8]*Gu1[5];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4];
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] = R11[0] + (real_t)1.0000000000000000e+02;
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] = R11[2];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] = R11[3] + (real_t)1.0000000000000000e+02;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] = acadoWorkspace.H[(iCol * 100) + (iRow * 2)];
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 100 + 50) + (iRow * 2)];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] = acadoWorkspace.H[(iCol * 100) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 100 + 50) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] = + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] = + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2];
dNew[1] = + acadoWorkspace.QN1[3]*dOld[0] + acadoWorkspace.QN1[4]*dOld[1] + acadoWorkspace.QN1[5]*dOld[2];
dNew[2] = + acadoWorkspace.QN1[6]*dOld[0] + acadoWorkspace.QN1[7]*dOld[1] + acadoWorkspace.QN1[8]*dOld[2];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6];
RDy1[1] = + R2[7]*Dy1[0] + R2[8]*Dy1[1] + R2[9]*Dy1[2] + R2[10]*Dy1[3] + R2[11]*Dy1[4] + R2[12]*Dy1[5] + R2[13]*Dy1[6];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6];
QDy1[1] = + Q2[7]*Dy1[0] + Q2[8]*Dy1[1] + Q2[9]*Dy1[2] + Q2[10]*Dy1[3] + Q2[11]*Dy1[4] + Q2[12]*Dy1[5] + Q2[13]*Dy1[6];
QDy1[2] = + Q2[14]*Dy1[0] + Q2[15]*Dy1[1] + Q2[16]*Dy1[2] + Q2[17]*Dy1[3] + Q2[18]*Dy1[4] + Q2[19]*Dy1[5] + Q2[20]*Dy1[6];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[3] + E1[4]*Gx1[6];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[4] + E1[4]*Gx1[7];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[5] + E1[4]*Gx1[8];
H101[3] += + E1[1]*Gx1[0] + E1[3]*Gx1[3] + E1[5]*Gx1[6];
H101[4] += + E1[1]*Gx1[1] + E1[3]*Gx1[4] + E1[5]*Gx1[7];
H101[5] += + E1[1]*Gx1[2] + E1[3]*Gx1[5] + E1[5]*Gx1[8];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 6; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[3] + Hx[2]*Gx[6];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[4] + Hx[2]*Gx[7];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[5] + Hx[2]*Gx[8];
A01[3] = + Hx[3]*Gx[0] + Hx[4]*Gx[3] + Hx[5]*Gx[6];
A01[4] = + Hx[3]*Gx[1] + Hx[4]*Gx[4] + Hx[5]*Gx[7];
A01[5] = + Hx[3]*Gx[2] + Hx[4]*Gx[5] + Hx[5]*Gx[8];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 100) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4];
acadoWorkspace.A[(row * 100) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5];
acadoWorkspace.A[(row * 100 + 50) + (col * 2)] = + Hx[3]*E[0] + Hx[4]*E[2] + Hx[5]*E[4];
acadoWorkspace.A[(row * 100 + 50) + (col * 2 + 1)] = + Hx[3]*E[1] + Hx[4]*E[3] + Hx[5]*E[5];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2];
acadoWorkspace.evHxd[1] = + Hx[3]*tmpd[0] + Hx[4]*tmpd[1] + Hx[5]*tmpd[2];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
const real_t* od = in + 5;
/* Vector of auxiliary variables; number of elements: 10. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(1.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(1.0000000000000000e+00);

/* Compute outputs: */
out[0] = (u[0]-od[2]);
out[1] = (u[1]-od[3]);
out[2] = a[0];
out[3] = a[1];
out[4] = a[2];
out[5] = a[3];
out[6] = a[4];
out[7] = a[5];
out[8] = a[6];
out[9] = a[7];
out[10] = a[8];
out[11] = a[9];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 25; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 9 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 3-3 ]), &(acadoWorkspace.evGx[ lRun1 * 9 ]), &(acadoWorkspace.d[ lRun1 * 3 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 9-9 ]), &(acadoWorkspace.evGx[ lRun1 * 9 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 6 ]), &(acadoWorkspace.E[ lRun3 * 6 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 6 ]), &(acadoWorkspace.E[ lRun3 * 6 ]) );
}

for (lRun1 = 0; lRun1 < 24; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 9 + 9 ]), &(acadoWorkspace.E[ lRun3 * 6 ]), &(acadoWorkspace.QE[ lRun3 * 6 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 6 ]), &(acadoWorkspace.QE[ lRun3 * 6 ]) );
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 6 ]) );
for (lRun2 = lRun1; lRun2 < 25; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 6 ]), &(acadoWorkspace.evGx[ lRun2 * 9 ]), &(acadoWorkspace.H10[ lRun1 * 6 ]) );
}
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 * 4 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 25; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 6 ]), &(acadoWorkspace.QE[ lRun5 * 6 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 25; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 25; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 6 ]), &(acadoWorkspace.QE[ lRun5 * 6 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( &(acadoWorkspace.Q1[ 9 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.d[ 3 ]), &(acadoWorkspace.Qd[ 3 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.Qd[ 6 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.d[ 9 ]), &(acadoWorkspace.Qd[ 9 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.Qd[ 15 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.Qd[ 18 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.d[ 21 ]), &(acadoWorkspace.Qd[ 21 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.d[ 27 ]), &(acadoWorkspace.Qd[ 27 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.d[ 33 ]), &(acadoWorkspace.Qd[ 33 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.d[ 39 ]), &(acadoWorkspace.Qd[ 39 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.Qd[ 42 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.Qd[ 45 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.d[ 51 ]), &(acadoWorkspace.Qd[ 51 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.Qd[ 54 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.d[ 57 ]), &(acadoWorkspace.Qd[ 57 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 189 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 198 ]), &(acadoWorkspace.d[ 63 ]), &(acadoWorkspace.Qd[ 63 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 207 ]), &(acadoWorkspace.d[ 66 ]), &(acadoWorkspace.Qd[ 66 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.d[ 69 ]), &(acadoWorkspace.Qd[ 69 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 25; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 6 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.conValueIn[5] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun1 * 4 + 3];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 2] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 2 + 1] = acadoWorkspace.conValueOut[1];

acadoWorkspace.evHx[lRun1 * 6] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 6 + 1] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 6 + 2] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 6 + 3] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 6 + 4] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 6 + 5] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[11];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];

acado_multHxC( &(acadoWorkspace.evHx[ 6 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 6 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.A01[ 12 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.A01[ 18 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.A01[ 24 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.A01[ 30 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.A01[ 36 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.A01[ 42 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.A01[ 48 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.A01[ 54 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.A01[ 66 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.A01[ 72 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.A01[ 78 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.A01[ 84 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.A01[ 90 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.A01[ 96 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 102 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.A01[ 108 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.A01[ 114 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.A01[ 126 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.A01[ 132 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 138 ]), &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.A01[ 138 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.A01[ 144 ]) );

for (lRun2 = 0; lRun2 < 24; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 6 + 6 ]), &(acadoWorkspace.E[ lRun4 * 6 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[50] = acadoWorkspace.evHu[2];
acadoWorkspace.A[51] = acadoWorkspace.evHu[3];
acadoWorkspace.A[102] = acadoWorkspace.evHu[4];
acadoWorkspace.A[103] = acadoWorkspace.evHu[5];
acadoWorkspace.A[152] = acadoWorkspace.evHu[6];
acadoWorkspace.A[153] = acadoWorkspace.evHu[7];
acadoWorkspace.A[204] = acadoWorkspace.evHu[8];
acadoWorkspace.A[205] = acadoWorkspace.evHu[9];
acadoWorkspace.A[254] = acadoWorkspace.evHu[10];
acadoWorkspace.A[255] = acadoWorkspace.evHu[11];
acadoWorkspace.A[306] = acadoWorkspace.evHu[12];
acadoWorkspace.A[307] = acadoWorkspace.evHu[13];
acadoWorkspace.A[356] = acadoWorkspace.evHu[14];
acadoWorkspace.A[357] = acadoWorkspace.evHu[15];
acadoWorkspace.A[408] = acadoWorkspace.evHu[16];
acadoWorkspace.A[409] = acadoWorkspace.evHu[17];
acadoWorkspace.A[458] = acadoWorkspace.evHu[18];
acadoWorkspace.A[459] = acadoWorkspace.evHu[19];
acadoWorkspace.A[510] = acadoWorkspace.evHu[20];
acadoWorkspace.A[511] = acadoWorkspace.evHu[21];
acadoWorkspace.A[560] = acadoWorkspace.evHu[22];
acadoWorkspace.A[561] = acadoWorkspace.evHu[23];
acadoWorkspace.A[612] = acadoWorkspace.evHu[24];
acadoWorkspace.A[613] = acadoWorkspace.evHu[25];
acadoWorkspace.A[662] = acadoWorkspace.evHu[26];
acadoWorkspace.A[663] = acadoWorkspace.evHu[27];
acadoWorkspace.A[714] = acadoWorkspace.evHu[28];
acadoWorkspace.A[715] = acadoWorkspace.evHu[29];
acadoWorkspace.A[764] = acadoWorkspace.evHu[30];
acadoWorkspace.A[765] = acadoWorkspace.evHu[31];
acadoWorkspace.A[816] = acadoWorkspace.evHu[32];
acadoWorkspace.A[817] = acadoWorkspace.evHu[33];
acadoWorkspace.A[866] = acadoWorkspace.evHu[34];
acadoWorkspace.A[867] = acadoWorkspace.evHu[35];
acadoWorkspace.A[918] = acadoWorkspace.evHu[36];
acadoWorkspace.A[919] = acadoWorkspace.evHu[37];
acadoWorkspace.A[968] = acadoWorkspace.evHu[38];
acadoWorkspace.A[969] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1020] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1021] = acadoWorkspace.evHu[41];
acadoWorkspace.A[1070] = acadoWorkspace.evHu[42];
acadoWorkspace.A[1071] = acadoWorkspace.evHu[43];
acadoWorkspace.A[1122] = acadoWorkspace.evHu[44];
acadoWorkspace.A[1123] = acadoWorkspace.evHu[45];
acadoWorkspace.A[1172] = acadoWorkspace.evHu[46];
acadoWorkspace.A[1173] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1224] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1225] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1274] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1275] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1326] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1327] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1376] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1377] = acadoWorkspace.evHu[55];
acadoWorkspace.A[1428] = acadoWorkspace.evHu[56];
acadoWorkspace.A[1429] = acadoWorkspace.evHu[57];
acadoWorkspace.A[1478] = acadoWorkspace.evHu[58];
acadoWorkspace.A[1479] = acadoWorkspace.evHu[59];
acadoWorkspace.A[1530] = acadoWorkspace.evHu[60];
acadoWorkspace.A[1531] = acadoWorkspace.evHu[61];
acadoWorkspace.A[1580] = acadoWorkspace.evHu[62];
acadoWorkspace.A[1581] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1632] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1633] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1682] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1683] = acadoWorkspace.evHu[67];
acadoWorkspace.A[1734] = acadoWorkspace.evHu[68];
acadoWorkspace.A[1735] = acadoWorkspace.evHu[69];
acadoWorkspace.A[1784] = acadoWorkspace.evHu[70];
acadoWorkspace.A[1785] = acadoWorkspace.evHu[71];
acadoWorkspace.A[1836] = acadoWorkspace.evHu[72];
acadoWorkspace.A[1837] = acadoWorkspace.evHu[73];
acadoWorkspace.A[1886] = acadoWorkspace.evHu[74];
acadoWorkspace.A[1887] = acadoWorkspace.evHu[75];
acadoWorkspace.A[1938] = acadoWorkspace.evHu[76];
acadoWorkspace.A[1939] = acadoWorkspace.evHu[77];
acadoWorkspace.A[1988] = acadoWorkspace.evHu[78];
acadoWorkspace.A[1989] = acadoWorkspace.evHu[79];
acadoWorkspace.A[2040] = acadoWorkspace.evHu[80];
acadoWorkspace.A[2041] = acadoWorkspace.evHu[81];
acadoWorkspace.A[2090] = acadoWorkspace.evHu[82];
acadoWorkspace.A[2091] = acadoWorkspace.evHu[83];
acadoWorkspace.A[2142] = acadoWorkspace.evHu[84];
acadoWorkspace.A[2143] = acadoWorkspace.evHu[85];
acadoWorkspace.A[2192] = acadoWorkspace.evHu[86];
acadoWorkspace.A[2193] = acadoWorkspace.evHu[87];
acadoWorkspace.A[2244] = acadoWorkspace.evHu[88];
acadoWorkspace.A[2245] = acadoWorkspace.evHu[89];
acadoWorkspace.A[2294] = acadoWorkspace.evHu[90];
acadoWorkspace.A[2295] = acadoWorkspace.evHu[91];
acadoWorkspace.A[2346] = acadoWorkspace.evHu[92];
acadoWorkspace.A[2347] = acadoWorkspace.evHu[93];
acadoWorkspace.A[2396] = acadoWorkspace.evHu[94];
acadoWorkspace.A[2397] = acadoWorkspace.evHu[95];
acadoWorkspace.A[2448] = acadoWorkspace.evHu[96];
acadoWorkspace.A[2449] = acadoWorkspace.evHu[97];
acadoWorkspace.A[2498] = acadoWorkspace.evHu[98];
acadoWorkspace.A[2499] = acadoWorkspace.evHu[99];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - acadoWorkspace.evH[49];

acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - acadoWorkspace.evH[49];

acado_macHxd( &(acadoWorkspace.evHx[ 6 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.d[ 3 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.d[ 9 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.d[ 21 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 27 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.d[ 33 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.d[ 39 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.d[ 51 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.lbA[ 38 ]), &(acadoWorkspace.ubA[ 38 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.d[ 57 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.d[ 63 ]), &(acadoWorkspace.lbA[ 44 ]), &(acadoWorkspace.ubA[ 44 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 138 ]), &(acadoWorkspace.d[ 66 ]), &(acadoWorkspace.lbA[ 46 ]), &(acadoWorkspace.ubA[ 46 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.d[ 69 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];

for (lRun2 = 0; lRun2 < 175; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 14 ]), &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 28 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 42 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 56 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 70 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 84 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 98 ]), &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 112 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 126 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 140 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 154 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 182 ]), &(acadoWorkspace.Dy[ 91 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 196 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 210 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 224 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 238 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 266 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 294 ]), &(acadoWorkspace.Dy[ 147 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 308 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 322 ]), &(acadoWorkspace.Dy[ 161 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 48 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 21 ]), &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 42 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 63 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 84 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 105 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 126 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 147 ]), &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 168 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 189 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 210 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 231 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.QDy[ 33 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 273 ]), &(acadoWorkspace.Dy[ 91 ]), &(acadoWorkspace.QDy[ 39 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 294 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 315 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 357 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.QDy[ 51 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 378 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 399 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.QDy[ 57 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 441 ]), &(acadoWorkspace.Dy[ 147 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 462 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 483 ]), &(acadoWorkspace.Dy[ 161 ]), &(acadoWorkspace.QDy[ 69 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 504 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 72 ]) );

acadoWorkspace.QDy[75] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[76] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[77] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];

acadoWorkspace.QDy[3] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[4] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[74];

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 25; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 6 ]), &(acadoWorkspace.QDy[ lRun2 * 3 + 3 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[1] += + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[2] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[3] += + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[4] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[5] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[6] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[7] += + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[8] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[9] += + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[10] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[11] += + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[12] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[13] += + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[14] += + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[15] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[16] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[17] += + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[18] += + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[19] += + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[20] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[21] += + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[22] += + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[23] += + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[24] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[25] += + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[26] += + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[27] += + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[28] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[29] += + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[30] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[31] += + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[32] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[33] += + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[34] += + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[35] += + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[36] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[37] += + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[38] += + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[39] += + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[40] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[41] += + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[42] += + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[43] += + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[44] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[45] += + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[46] += + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[47] += + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[48] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[49] += + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[2];

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[2];
acadoWorkspace.lbA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.lbA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.lbA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.lbA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.lbA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.lbA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.lbA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.lbA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.lbA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.lbA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.lbA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.lbA[49] -= acadoWorkspace.pacA01Dx0[49];

acadoWorkspace.ubA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.ubA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.ubA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.ubA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.ubA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.ubA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.ubA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.ubA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.ubA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.ubA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.ubA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.ubA[49] -= acadoWorkspace.pacA01Dx0[49];

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];

acadoVariables.x[3] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[0];
acadoVariables.x[4] += + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[1];
acadoVariables.x[5] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[2];
acadoVariables.x[6] += + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[3];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[4];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[5];
acadoVariables.x[9] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[6];
acadoVariables.x[10] += + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[7];
acadoVariables.x[11] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[8];
acadoVariables.x[12] += + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[9];
acadoVariables.x[13] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[10];
acadoVariables.x[14] += + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[11];
acadoVariables.x[15] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[12];
acadoVariables.x[16] += + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[13];
acadoVariables.x[17] += + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[14];
acadoVariables.x[18] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[15];
acadoVariables.x[19] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[16];
acadoVariables.x[20] += + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[17];
acadoVariables.x[21] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[18];
acadoVariables.x[22] += + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[19];
acadoVariables.x[23] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[20];
acadoVariables.x[24] += + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[21];
acadoVariables.x[25] += + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[22];
acadoVariables.x[26] += + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[23];
acadoVariables.x[27] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[24];
acadoVariables.x[28] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[25];
acadoVariables.x[29] += + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[26];
acadoVariables.x[30] += + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[27];
acadoVariables.x[31] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[28];
acadoVariables.x[32] += + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[29];
acadoVariables.x[33] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[30];
acadoVariables.x[34] += + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[31];
acadoVariables.x[35] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[32];
acadoVariables.x[36] += + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[33];
acadoVariables.x[37] += + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[34];
acadoVariables.x[38] += + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[35];
acadoVariables.x[39] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[36];
acadoVariables.x[40] += + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[37];
acadoVariables.x[41] += + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[38];
acadoVariables.x[42] += + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[39];
acadoVariables.x[43] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[40];
acadoVariables.x[44] += + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[41];
acadoVariables.x[45] += + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[42];
acadoVariables.x[46] += + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[43];
acadoVariables.x[47] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[44];
acadoVariables.x[48] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[45];
acadoVariables.x[49] += + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[46];
acadoVariables.x[50] += + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[47];
acadoVariables.x[51] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[48];
acadoVariables.x[52] += + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[49];
acadoVariables.x[53] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[50];
acadoVariables.x[54] += + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[51];
acadoVariables.x[55] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[52];
acadoVariables.x[56] += + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[53];
acadoVariables.x[57] += + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[54];
acadoVariables.x[58] += + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[55];
acadoVariables.x[59] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[56];
acadoVariables.x[60] += + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[57];
acadoVariables.x[61] += + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[58];
acadoVariables.x[62] += + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[59];
acadoVariables.x[63] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[60];
acadoVariables.x[64] += + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[61];
acadoVariables.x[65] += + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[62];
acadoVariables.x[66] += + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[63];
acadoVariables.x[67] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[64];
acadoVariables.x[68] += + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[65];
acadoVariables.x[69] += + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[66];
acadoVariables.x[70] += + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[67];
acadoVariables.x[71] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[68];
acadoVariables.x[72] += + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[69];
acadoVariables.x[73] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[70];
acadoVariables.x[74] += + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[71];
acadoVariables.x[75] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[72];
acadoVariables.x[76] += + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[73];
acadoVariables.x[77] += + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[2] + acadoWorkspace.d[74];

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 6 ]), &(acadoWorkspace.x[ lRun2 * 2 ]), &(acadoVariables.x[ lRun1 * 3 + 3 ]) );
}
}
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -5.2300000000000002e-01;
acadoVariables.lbValues[1] = -1.9443999999999999e+00;
acadoVariables.lbValues[2] = -5.2300000000000002e-01;
acadoVariables.lbValues[3] = -1.9443999999999999e+00;
acadoVariables.lbValues[4] = -5.2300000000000002e-01;
acadoVariables.lbValues[5] = -1.9443999999999999e+00;
acadoVariables.lbValues[6] = -5.2300000000000002e-01;
acadoVariables.lbValues[7] = -1.9443999999999999e+00;
acadoVariables.lbValues[8] = -5.2300000000000002e-01;
acadoVariables.lbValues[9] = -1.9443999999999999e+00;
acadoVariables.lbValues[10] = -5.2300000000000002e-01;
acadoVariables.lbValues[11] = -1.9443999999999999e+00;
acadoVariables.lbValues[12] = -5.2300000000000002e-01;
acadoVariables.lbValues[13] = -1.9443999999999999e+00;
acadoVariables.lbValues[14] = -5.2300000000000002e-01;
acadoVariables.lbValues[15] = -1.9443999999999999e+00;
acadoVariables.lbValues[16] = -5.2300000000000002e-01;
acadoVariables.lbValues[17] = -1.9443999999999999e+00;
acadoVariables.lbValues[18] = -5.2300000000000002e-01;
acadoVariables.lbValues[19] = -1.9443999999999999e+00;
acadoVariables.lbValues[20] = -5.2300000000000002e-01;
acadoVariables.lbValues[21] = -1.9443999999999999e+00;
acadoVariables.lbValues[22] = -5.2300000000000002e-01;
acadoVariables.lbValues[23] = -1.9443999999999999e+00;
acadoVariables.lbValues[24] = -5.2300000000000002e-01;
acadoVariables.lbValues[25] = -1.9443999999999999e+00;
acadoVariables.lbValues[26] = -5.2300000000000002e-01;
acadoVariables.lbValues[27] = -1.9443999999999999e+00;
acadoVariables.lbValues[28] = -5.2300000000000002e-01;
acadoVariables.lbValues[29] = -1.9443999999999999e+00;
acadoVariables.lbValues[30] = -5.2300000000000002e-01;
acadoVariables.lbValues[31] = -1.9443999999999999e+00;
acadoVariables.lbValues[32] = -5.2300000000000002e-01;
acadoVariables.lbValues[33] = -1.9443999999999999e+00;
acadoVariables.lbValues[34] = -5.2300000000000002e-01;
acadoVariables.lbValues[35] = -1.9443999999999999e+00;
acadoVariables.lbValues[36] = -5.2300000000000002e-01;
acadoVariables.lbValues[37] = -1.9443999999999999e+00;
acadoVariables.lbValues[38] = -5.2300000000000002e-01;
acadoVariables.lbValues[39] = -1.9443999999999999e+00;
acadoVariables.lbValues[40] = -5.2300000000000002e-01;
acadoVariables.lbValues[41] = -1.9443999999999999e+00;
acadoVariables.lbValues[42] = -5.2300000000000002e-01;
acadoVariables.lbValues[43] = -1.9443999999999999e+00;
acadoVariables.lbValues[44] = -5.2300000000000002e-01;
acadoVariables.lbValues[45] = -1.9443999999999999e+00;
acadoVariables.lbValues[46] = -5.2300000000000002e-01;
acadoVariables.lbValues[47] = -1.9443999999999999e+00;
acadoVariables.lbValues[48] = -5.2300000000000002e-01;
acadoVariables.lbValues[49] = -1.9443999999999999e+00;
acadoVariables.ubValues[0] = 5.2300000000000002e-01;
acadoVariables.ubValues[1] = 1.9443999999999999e+00;
acadoVariables.ubValues[2] = 5.2300000000000002e-01;
acadoVariables.ubValues[3] = 1.9443999999999999e+00;
acadoVariables.ubValues[4] = 5.2300000000000002e-01;
acadoVariables.ubValues[5] = 1.9443999999999999e+00;
acadoVariables.ubValues[6] = 5.2300000000000002e-01;
acadoVariables.ubValues[7] = 1.9443999999999999e+00;
acadoVariables.ubValues[8] = 5.2300000000000002e-01;
acadoVariables.ubValues[9] = 1.9443999999999999e+00;
acadoVariables.ubValues[10] = 5.2300000000000002e-01;
acadoVariables.ubValues[11] = 1.9443999999999999e+00;
acadoVariables.ubValues[12] = 5.2300000000000002e-01;
acadoVariables.ubValues[13] = 1.9443999999999999e+00;
acadoVariables.ubValues[14] = 5.2300000000000002e-01;
acadoVariables.ubValues[15] = 1.9443999999999999e+00;
acadoVariables.ubValues[16] = 5.2300000000000002e-01;
acadoVariables.ubValues[17] = 1.9443999999999999e+00;
acadoVariables.ubValues[18] = 5.2300000000000002e-01;
acadoVariables.ubValues[19] = 1.9443999999999999e+00;
acadoVariables.ubValues[20] = 5.2300000000000002e-01;
acadoVariables.ubValues[21] = 1.9443999999999999e+00;
acadoVariables.ubValues[22] = 5.2300000000000002e-01;
acadoVariables.ubValues[23] = 1.9443999999999999e+00;
acadoVariables.ubValues[24] = 5.2300000000000002e-01;
acadoVariables.ubValues[25] = 1.9443999999999999e+00;
acadoVariables.ubValues[26] = 5.2300000000000002e-01;
acadoVariables.ubValues[27] = 1.9443999999999999e+00;
acadoVariables.ubValues[28] = 5.2300000000000002e-01;
acadoVariables.ubValues[29] = 1.9443999999999999e+00;
acadoVariables.ubValues[30] = 5.2300000000000002e-01;
acadoVariables.ubValues[31] = 1.9443999999999999e+00;
acadoVariables.ubValues[32] = 5.2300000000000002e-01;
acadoVariables.ubValues[33] = 1.9443999999999999e+00;
acadoVariables.ubValues[34] = 5.2300000000000002e-01;
acadoVariables.ubValues[35] = 1.9443999999999999e+00;
acadoVariables.ubValues[36] = 5.2300000000000002e-01;
acadoVariables.ubValues[37] = 1.9443999999999999e+00;
acadoVariables.ubValues[38] = 5.2300000000000002e-01;
acadoVariables.ubValues[39] = 1.9443999999999999e+00;
acadoVariables.ubValues[40] = 5.2300000000000002e-01;
acadoVariables.ubValues[41] = 1.9443999999999999e+00;
acadoVariables.ubValues[42] = 5.2300000000000002e-01;
acadoVariables.ubValues[43] = 1.9443999999999999e+00;
acadoVariables.ubValues[44] = 5.2300000000000002e-01;
acadoVariables.ubValues[45] = 1.9443999999999999e+00;
acadoVariables.ubValues[46] = 5.2300000000000002e-01;
acadoVariables.ubValues[47] = 1.9443999999999999e+00;
acadoVariables.ubValues[48] = 5.2300000000000002e-01;
acadoVariables.ubValues[49] = 1.9443999999999999e+00;
acadoVariables.lbAValues[0] = -2.6000000000000001e-01;
acadoVariables.lbAValues[1] = -5.0000000000000000e-01;
acadoVariables.lbAValues[2] = -2.6000000000000001e-01;
acadoVariables.lbAValues[3] = -5.0000000000000000e-01;
acadoVariables.lbAValues[4] = -2.6000000000000001e-01;
acadoVariables.lbAValues[5] = -5.0000000000000000e-01;
acadoVariables.lbAValues[6] = -2.6000000000000001e-01;
acadoVariables.lbAValues[7] = -5.0000000000000000e-01;
acadoVariables.lbAValues[8] = -2.6000000000000001e-01;
acadoVariables.lbAValues[9] = -5.0000000000000000e-01;
acadoVariables.lbAValues[10] = -2.6000000000000001e-01;
acadoVariables.lbAValues[11] = -5.0000000000000000e-01;
acadoVariables.lbAValues[12] = -2.6000000000000001e-01;
acadoVariables.lbAValues[13] = -5.0000000000000000e-01;
acadoVariables.lbAValues[14] = -2.6000000000000001e-01;
acadoVariables.lbAValues[15] = -5.0000000000000000e-01;
acadoVariables.lbAValues[16] = -2.6000000000000001e-01;
acadoVariables.lbAValues[17] = -5.0000000000000000e-01;
acadoVariables.lbAValues[18] = -2.6000000000000001e-01;
acadoVariables.lbAValues[19] = -5.0000000000000000e-01;
acadoVariables.lbAValues[20] = -2.6000000000000001e-01;
acadoVariables.lbAValues[21] = -5.0000000000000000e-01;
acadoVariables.lbAValues[22] = -2.6000000000000001e-01;
acadoVariables.lbAValues[23] = -5.0000000000000000e-01;
acadoVariables.lbAValues[24] = -2.6000000000000001e-01;
acadoVariables.lbAValues[25] = -5.0000000000000000e-01;
acadoVariables.lbAValues[26] = -2.6000000000000001e-01;
acadoVariables.lbAValues[27] = -5.0000000000000000e-01;
acadoVariables.lbAValues[28] = -2.6000000000000001e-01;
acadoVariables.lbAValues[29] = -5.0000000000000000e-01;
acadoVariables.lbAValues[30] = -2.6000000000000001e-01;
acadoVariables.lbAValues[31] = -5.0000000000000000e-01;
acadoVariables.lbAValues[32] = -2.6000000000000001e-01;
acadoVariables.lbAValues[33] = -5.0000000000000000e-01;
acadoVariables.lbAValues[34] = -2.6000000000000001e-01;
acadoVariables.lbAValues[35] = -5.0000000000000000e-01;
acadoVariables.lbAValues[36] = -2.6000000000000001e-01;
acadoVariables.lbAValues[37] = -5.0000000000000000e-01;
acadoVariables.lbAValues[38] = -2.6000000000000001e-01;
acadoVariables.lbAValues[39] = -5.0000000000000000e-01;
acadoVariables.lbAValues[40] = -2.6000000000000001e-01;
acadoVariables.lbAValues[41] = -5.0000000000000000e-01;
acadoVariables.lbAValues[42] = -2.6000000000000001e-01;
acadoVariables.lbAValues[43] = -5.0000000000000000e-01;
acadoVariables.lbAValues[44] = -2.6000000000000001e-01;
acadoVariables.lbAValues[45] = -5.0000000000000000e-01;
acadoVariables.lbAValues[46] = -2.6000000000000001e-01;
acadoVariables.lbAValues[47] = -5.0000000000000000e-01;
acadoVariables.lbAValues[48] = -2.6000000000000001e-01;
acadoVariables.lbAValues[49] = -5.0000000000000000e-01;
acadoVariables.ubAValues[0] = 2.6000000000000001e-01;
acadoVariables.ubAValues[1] = 5.0000000000000000e-01;
acadoVariables.ubAValues[2] = 2.6000000000000001e-01;
acadoVariables.ubAValues[3] = 5.0000000000000000e-01;
acadoVariables.ubAValues[4] = 2.6000000000000001e-01;
acadoVariables.ubAValues[5] = 5.0000000000000000e-01;
acadoVariables.ubAValues[6] = 2.6000000000000001e-01;
acadoVariables.ubAValues[7] = 5.0000000000000000e-01;
acadoVariables.ubAValues[8] = 2.6000000000000001e-01;
acadoVariables.ubAValues[9] = 5.0000000000000000e-01;
acadoVariables.ubAValues[10] = 2.6000000000000001e-01;
acadoVariables.ubAValues[11] = 5.0000000000000000e-01;
acadoVariables.ubAValues[12] = 2.6000000000000001e-01;
acadoVariables.ubAValues[13] = 5.0000000000000000e-01;
acadoVariables.ubAValues[14] = 2.6000000000000001e-01;
acadoVariables.ubAValues[15] = 5.0000000000000000e-01;
acadoVariables.ubAValues[16] = 2.6000000000000001e-01;
acadoVariables.ubAValues[17] = 5.0000000000000000e-01;
acadoVariables.ubAValues[18] = 2.6000000000000001e-01;
acadoVariables.ubAValues[19] = 5.0000000000000000e-01;
acadoVariables.ubAValues[20] = 2.6000000000000001e-01;
acadoVariables.ubAValues[21] = 5.0000000000000000e-01;
acadoVariables.ubAValues[22] = 2.6000000000000001e-01;
acadoVariables.ubAValues[23] = 5.0000000000000000e-01;
acadoVariables.ubAValues[24] = 2.6000000000000001e-01;
acadoVariables.ubAValues[25] = 5.0000000000000000e-01;
acadoVariables.ubAValues[26] = 2.6000000000000001e-01;
acadoVariables.ubAValues[27] = 5.0000000000000000e-01;
acadoVariables.ubAValues[28] = 2.6000000000000001e-01;
acadoVariables.ubAValues[29] = 5.0000000000000000e-01;
acadoVariables.ubAValues[30] = 2.6000000000000001e-01;
acadoVariables.ubAValues[31] = 5.0000000000000000e-01;
acadoVariables.ubAValues[32] = 2.6000000000000001e-01;
acadoVariables.ubAValues[33] = 5.0000000000000000e-01;
acadoVariables.ubAValues[34] = 2.6000000000000001e-01;
acadoVariables.ubAValues[35] = 5.0000000000000000e-01;
acadoVariables.ubAValues[36] = 2.6000000000000001e-01;
acadoVariables.ubAValues[37] = 5.0000000000000000e-01;
acadoVariables.ubAValues[38] = 2.6000000000000001e-01;
acadoVariables.ubAValues[39] = 5.0000000000000000e-01;
acadoVariables.ubAValues[40] = 2.6000000000000001e-01;
acadoVariables.ubAValues[41] = 5.0000000000000000e-01;
acadoVariables.ubAValues[42] = 2.6000000000000001e-01;
acadoVariables.ubAValues[43] = 5.0000000000000000e-01;
acadoVariables.ubAValues[44] = 2.6000000000000001e-01;
acadoVariables.ubAValues[45] = 5.0000000000000000e-01;
acadoVariables.ubAValues[46] = 2.6000000000000001e-01;
acadoVariables.ubAValues[47] = 5.0000000000000000e-01;
acadoVariables.ubAValues[48] = 2.6000000000000001e-01;
acadoVariables.ubAValues[49] = 5.0000000000000000e-01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[18] = acadoVariables.u[index * 2];
acadoWorkspace.state[19] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[20] = acadoVariables.od[index * 4];
acadoWorkspace.state[21] = acadoVariables.od[index * 4 + 1];
acadoWorkspace.state[22] = acadoVariables.od[index * 4 + 2];
acadoWorkspace.state[23] = acadoVariables.od[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[75] = xEnd[0];
acadoVariables.x[76] = xEnd[1];
acadoVariables.x[77] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[75];
acadoWorkspace.state[1] = acadoVariables.x[76];
acadoWorkspace.state[2] = acadoVariables.x[77];
if (uEnd != 0)
{
acadoWorkspace.state[18] = uEnd[0];
acadoWorkspace.state[19] = uEnd[1];
}
else
{
acadoWorkspace.state[18] = acadoVariables.u[48];
acadoWorkspace.state[19] = acadoVariables.u[49];
}
acadoWorkspace.state[20] = acadoVariables.od[100];
acadoWorkspace.state[21] = acadoVariables.od[101];
acadoWorkspace.state[22] = acadoVariables.od[102];
acadoWorkspace.state[23] = acadoVariables.od[103];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[75] = acadoWorkspace.state[0];
acadoVariables.x[76] = acadoWorkspace.state[1];
acadoVariables.x[77] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 24; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[48] = uEnd[0];
acadoVariables.u[49] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49];
kkt = fabs( kkt );
for (index = 0; index < 50; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 50; ++index)
{
prd = acadoWorkspace.y[index + 50];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 7 */
real_t tmpDy[ 7 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 7] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 7];
acadoWorkspace.Dy[lRun1 * 7 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 7 + 1];
acadoWorkspace.Dy[lRun1 * 7 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 7 + 2];
acadoWorkspace.Dy[lRun1 * 7 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 7 + 3];
acadoWorkspace.Dy[lRun1 * 7 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 7 + 4];
acadoWorkspace.Dy[lRun1 * 7 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 7 + 5];
acadoWorkspace.Dy[lRun1 * 7 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 7 + 6];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[75];
acadoWorkspace.objValueIn[1] = acadoVariables.x[76];
acadoWorkspace.objValueIn[2] = acadoVariables.x[77];
acadoWorkspace.objValueIn[3] = acadoVariables.od[100];
acadoWorkspace.objValueIn[4] = acadoVariables.od[101];
acadoWorkspace.objValueIn[5] = acadoVariables.od[102];
acadoWorkspace.objValueIn[6] = acadoVariables.od[103];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 7] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 14] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 21] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 28] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 35] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 42];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 1] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 8] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 15] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 22] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 29] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 36] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 43];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 2] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 9] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 16] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 23] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 30] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 37] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 44];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 3] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 10] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 17] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 24] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 31] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 38] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 45];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 4] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 11] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 18] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 25] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 32] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 39] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 46];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 5] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 12] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 19] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 26] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 33] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 40] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 47];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 6] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 13] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 20] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 27] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 34] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 41] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 48];
objVal += + acadoWorkspace.Dy[lRun1 * 7]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 7 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 7 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 7 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 7 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 7 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 7 + 6]*tmpDy[6];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

