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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
const real_t* od = in + 4;
/* Vector of auxiliary variables; number of elements: 7. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(u[0]));
a[1] = (atan(((od[1]*a[0])/(od[0]+od[1]))));
a[2] = (cos((xd[2]+a[1])));
a[3] = (cos(a[1]));
a[4] = (sin((xd[2]+a[1])));
a[5] = (cos(a[1]));
a[6] = (tan(u[0]));

/* Compute outputs: */
out[0] = ((((od[2]*a[2])/a[3])-od[3])+od[3]);
out[1] = ((od[2]*a[4])/a[5]);
out[2] = ((od[2]*a[6])/(od[1]+od[0]));
}

void acado_rhs_ext(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 15;
const real_t* od = in + 16;
/* Vector of auxiliary variables; number of elements: 34. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(u[0]));
a[1] = (atan(((od[1]*a[0])/(od[0]+od[1]))));
a[2] = (cos((xd[2]+a[1])));
a[3] = (cos(a[1]));
a[4] = (sin((xd[2]+a[1])));
a[5] = (cos(a[1]));
a[6] = (tan(u[0]));
a[7] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[2]+a[1]))));
a[8] = (xd[9]*a[7]);
a[9] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[10] = (xd[10]*a[7]);
a[11] = (xd[11]*a[7]);
a[12] = (cos((xd[2]+a[1])));
a[13] = (xd[9]*a[12]);
a[14] = ((real_t)(1.0000000000000000e+00)/a[5]);
a[15] = (xd[10]*a[12]);
a[16] = (xd[11]*a[12]);
a[17] = (xd[14]*a[7]);
a[18] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[0])),2)));
a[19] = ((real_t)(1.0000000000000000e+00)/(od[0]+od[1]));
a[20] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[1]*a[0])/(od[0]+od[1])),2))));
a[21] = (((od[1]*a[18])*a[19])*a[20]);
a[22] = (a[21]*a[7]);
a[23] = ((real_t)(-1.0000000000000000e+00)*(sin(a[1])));
a[24] = (a[21]*a[23]);
a[25] = (a[9]*a[9]);
a[26] = (xd[14]*a[12]);
a[27] = (((od[1]*a[18])*a[19])*a[20]);
a[28] = (a[27]*a[12]);
a[29] = ((real_t)(-1.0000000000000000e+00)*(sin(a[1])));
a[30] = (a[27]*a[29]);
a[31] = (a[14]*a[14]);
a[32] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[0])),2)));
a[33] = ((real_t)(1.0000000000000000e+00)/(od[1]+od[0]));

/* Compute outputs: */
out[0] = ((((od[2]*a[2])/a[3])-od[3])+od[3]);
out[1] = ((od[2]*a[4])/a[5]);
out[2] = ((od[2]*a[6])/(od[1]+od[0]));
out[3] = ((od[2]*a[8])*a[9]);
out[4] = ((od[2]*a[10])*a[9]);
out[5] = ((od[2]*a[11])*a[9]);
out[6] = ((od[2]*a[13])*a[14]);
out[7] = ((od[2]*a[15])*a[14]);
out[8] = ((od[2]*a[16])*a[14]);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (((od[2]*a[17])*a[9])+(((od[2]*a[22])*a[9])-(((od[2]*a[2])*a[24])*a[25])));
out[13] = (((od[2]*a[26])*a[14])+(((od[2]*a[28])*a[14])-(((od[2]*a[4])*a[30])*a[31])));
out[14] = ((od[2]*a[32])*a[33]);
}

/* Fixed step size:0.004 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
rk_eta[3] = 1.0000000000000000e+00;
rk_eta[4] = 0.0000000000000000e+00;
rk_eta[5] = 0.0000000000000000e+00;
rk_eta[6] = 0.0000000000000000e+00;
rk_eta[7] = 1.0000000000000000e+00;
rk_eta[8] = 0.0000000000000000e+00;
rk_eta[9] = 0.0000000000000000e+00;
rk_eta[10] = 0.0000000000000000e+00;
rk_eta[11] = 1.0000000000000000e+00;
rk_eta[12] = 0.0000000000000000e+00;
rk_eta[13] = 0.0000000000000000e+00;
rk_eta[14] = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[15] = rk_eta[15];
acadoWorkspace.rk_xxx[16] = rk_eta[16];
acadoWorkspace.rk_xxx[17] = rk_eta[17];
acadoWorkspace.rk_xxx[18] = rk_eta[18];
acadoWorkspace.rk_xxx[19] = rk_eta[19];

for (run1 = 0; run1 < 10; ++run1)
{
acadoWorkspace.rk_xxx[0] = + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + rk_eta[14];
acado_rhs_ext( acadoWorkspace.rk_xxx, acadoWorkspace.rk_kkk );
acadoWorkspace.rk_xxx[0] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[0] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[1] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[2] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[3] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[4] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[5] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[6] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[7] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[8] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[9] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[10] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[11] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[12] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[13] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)1.3333333333333333e-03*acadoWorkspace.rk_kkk[14] + rk_eta[14];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 15 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[15] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[16] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[17] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[18] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[19] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[20] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[21] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[22] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[23] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[24] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[25] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[26] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[27] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[28] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)2.6666666666666666e-03*acadoWorkspace.rk_kkk[29] + rk_eta[14];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 30 ]) );
rk_eta[0] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[0] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[30];
rk_eta[1] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[1] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[31];
rk_eta[2] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[2] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[32];
rk_eta[3] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[3] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[33];
rk_eta[4] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[4] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[34];
rk_eta[5] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[5] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[35];
rk_eta[6] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[6] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[36];
rk_eta[7] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[7] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[37];
rk_eta[8] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[8] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[38];
rk_eta[9] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[9] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[39];
rk_eta[10] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[10] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[40];
rk_eta[11] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[11] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[41];
rk_eta[12] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[12] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[42];
rk_eta[13] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[13] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[43];
rk_eta[14] += + (real_t)1.0000000000000000e-03*acadoWorkspace.rk_kkk[14] + (real_t)3.0000000000000001e-03*acadoWorkspace.rk_kkk[44];
acadoWorkspace.rk_ttt += 1.0000000000000001e-01;
}
error = 0;
return error;
}

