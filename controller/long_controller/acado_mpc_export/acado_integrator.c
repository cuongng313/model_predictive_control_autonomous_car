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

/* Compute outputs: */
out[0] = xd[1];
out[1] = xd[2];
out[2] = u[0];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(1.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(1.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim6_triangular( real_t* const A, real_t* const b )
{

b[5] = b[5]/A[35];
b[4] -= + A[29]*b[5];
b[4] = b[4]/A[28];
b[3] -= + A[23]*b[5];
b[3] -= + A[22]*b[4];
b[3] = b[3]/A[21];
b[2] -= + A[17]*b[5];
b[2] -= + A[16]*b[4];
b[2] -= + A[15]*b[3];
b[2] = b[2]/A[14];
b[1] -= + A[11]*b[5];
b[1] -= + A[10]*b[4];
b[1] -= + A[9]*b[3];
b[1] -= + A[8]*b[2];
b[1] = b[1]/A[7];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim6_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 6; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (5); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*6+i]);
	for( j=(i+1); j < 6; j++ ) {
		temp = fabs(A[j*6+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 6; ++k)
{
	acadoWorkspace.rk_dim6_swap = A[i*6+k];
	A[i*6+k] = A[indexMax*6+k];
	A[indexMax*6+k] = acadoWorkspace.rk_dim6_swap;
}
	acadoWorkspace.rk_dim6_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim6_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*6+i];
	for( j=i+1; j < 6; j++ ) {
		A[j*6+i] = -A[j*6+i]/A[i*6+i];
		for( k=i+1; k < 6; k++ ) {
			A[j*6+k] += A[j*6+i] * A[i*6+k];
		}
		b[j] += A[j*6+i] * b[i];
	}
}
det *= A[35];
det = fabs(det);
acado_solve_dim6_triangular( A, b );
return det;
}

void acado_solve_dim6_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim6_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim6_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim6_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim6_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim6_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim6_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim6_bPerm[1] += A[6]*acadoWorkspace.rk_dim6_bPerm[0];

acadoWorkspace.rk_dim6_bPerm[2] += A[12]*acadoWorkspace.rk_dim6_bPerm[0];
acadoWorkspace.rk_dim6_bPerm[2] += A[13]*acadoWorkspace.rk_dim6_bPerm[1];

acadoWorkspace.rk_dim6_bPerm[3] += A[18]*acadoWorkspace.rk_dim6_bPerm[0];
acadoWorkspace.rk_dim6_bPerm[3] += A[19]*acadoWorkspace.rk_dim6_bPerm[1];
acadoWorkspace.rk_dim6_bPerm[3] += A[20]*acadoWorkspace.rk_dim6_bPerm[2];

acadoWorkspace.rk_dim6_bPerm[4] += A[24]*acadoWorkspace.rk_dim6_bPerm[0];
acadoWorkspace.rk_dim6_bPerm[4] += A[25]*acadoWorkspace.rk_dim6_bPerm[1];
acadoWorkspace.rk_dim6_bPerm[4] += A[26]*acadoWorkspace.rk_dim6_bPerm[2];
acadoWorkspace.rk_dim6_bPerm[4] += A[27]*acadoWorkspace.rk_dim6_bPerm[3];

acadoWorkspace.rk_dim6_bPerm[5] += A[30]*acadoWorkspace.rk_dim6_bPerm[0];
acadoWorkspace.rk_dim6_bPerm[5] += A[31]*acadoWorkspace.rk_dim6_bPerm[1];
acadoWorkspace.rk_dim6_bPerm[5] += A[32]*acadoWorkspace.rk_dim6_bPerm[2];
acadoWorkspace.rk_dim6_bPerm[5] += A[33]*acadoWorkspace.rk_dim6_bPerm[3];
acadoWorkspace.rk_dim6_bPerm[5] += A[34]*acadoWorkspace.rk_dim6_bPerm[4];


acado_solve_dim6_triangular( A, acadoWorkspace.rk_dim6_bPerm );
b[0] = acadoWorkspace.rk_dim6_bPerm[0];
b[1] = acadoWorkspace.rk_dim6_bPerm[1];
b[2] = acadoWorkspace.rk_dim6_bPerm[2];
b[3] = acadoWorkspace.rk_dim6_bPerm[3];
b[4] = acadoWorkspace.rk_dim6_bPerm[4];
b[5] = acadoWorkspace.rk_dim6_bPerm[5];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 6.6666666666666671e-03, -1.3333333333333333e-03, 
1.2000000000000000e-02, 4.0000000000000001e-03 };


/* Fixed step size:0.016 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[3] = rk_eta[15];

for (run = 0; run < 10; ++run)
{
if( run > 0 ) {
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 4] = rk_eta[i * 3 + 3];
acadoWorkspace.rk_diffsPrev2[i * 4 + 1] = rk_eta[i * 3 + 4];
acadoWorkspace.rk_diffsPrev2[i * 4 + 2] = rk_eta[i * 3 + 5];
acadoWorkspace.rk_diffsPrev2[i * 4 + 3] = rk_eta[i + 12];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 3; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 12 ]) );
for (j = 0; j < 3; ++j)
{
tmp_index1 = (run1 * 3) + (j);
acadoWorkspace.rk_A[tmp_index1 * 6] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4)];
acadoWorkspace.rk_A[tmp_index1 * 6 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 6 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4 + 2)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 6) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 6 + 3] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4)];
acadoWorkspace.rk_A[tmp_index1 * 6 + 4] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 6 + 5] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4 + 2)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 6) + (j + 3)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 3] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 3 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 3 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
}
det = acado_solve_dim6_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim6_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 3];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 3 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 3 + 2];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 3; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 3] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 3 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 3 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
}
acado_solve_dim6_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim6_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 3];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 3 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 3 + 2];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 3; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 12 ]) );
for (j = 0; j < 3; ++j)
{
tmp_index1 = (run1 * 3) + (j);
acadoWorkspace.rk_A[tmp_index1 * 6] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4)];
acadoWorkspace.rk_A[tmp_index1 * 6 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 6 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4 + 2)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 6) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 6 + 3] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4)];
acadoWorkspace.rk_A[tmp_index1 * 6 + 4] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 6 + 5] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 4 + 2)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 6) + (j + 3)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 3] = - acadoWorkspace.rk_diffsTemp2[(i * 12) + (run1)];
acadoWorkspace.rk_b[i * 3 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 12) + (run1 + 4)];
acadoWorkspace.rk_b[i * 3 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 12) + (run1 + 8)];
}
if( 0 == run1 ) {
det = acado_solve_dim6_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim6_perm );
}
 else {
acado_solve_dim6_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim6_perm );
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 3];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 3 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 3 + 2];
}
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 4) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 4) + (run1)] += + acadoWorkspace.rk_diffK[i * 2]*(real_t)1.2000000000000000e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)4.0000000000000001e-03;
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 3; ++j)
{
tmp_index1 = (i * 3) + (j);
tmp_index2 = (run1) + (j * 4);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 12) + (tmp_index2 + 3)];
}
}
acado_solve_dim6_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim6_perm );
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 3];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 3 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 3 + 2];
}
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 4) + (run1 + 3)] = + acadoWorkspace.rk_diffK[i * 2]*(real_t)1.2000000000000000e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)4.0000000000000001e-03;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)1.2000000000000000e-02 + acadoWorkspace.rk_kkk[1]*(real_t)4.0000000000000001e-03;
rk_eta[1] += + acadoWorkspace.rk_kkk[2]*(real_t)1.2000000000000000e-02 + acadoWorkspace.rk_kkk[3]*(real_t)4.0000000000000001e-03;
rk_eta[2] += + acadoWorkspace.rk_kkk[4]*(real_t)1.2000000000000000e-02 + acadoWorkspace.rk_kkk[5]*(real_t)4.0000000000000001e-03;
if( run == 0 ) {
for (i = 0; i < 3; ++i)
{
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 3] = acadoWorkspace.rk_diffsNew2[(i * 4) + (j)];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 12] = acadoWorkspace.rk_diffsNew2[(i * 4) + (j + 3)];
}
}
}
else {
for (i = 0; i < 3; ++i)
{
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 3] = + acadoWorkspace.rk_diffsNew2[i * 4]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 3] += + acadoWorkspace.rk_diffsNew2[i * 4 + 1]*acadoWorkspace.rk_diffsPrev2[j + 4];
rk_eta[tmp_index2 + 3] += + acadoWorkspace.rk_diffsNew2[i * 4 + 2]*acadoWorkspace.rk_diffsPrev2[j + 8];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 12] = acadoWorkspace.rk_diffsNew2[(i * 4) + (j + 3)];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 4]*acadoWorkspace.rk_diffsPrev2[j + 3];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 4 + 1]*acadoWorkspace.rk_diffsPrev2[j + 7];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 4 + 2]*acadoWorkspace.rk_diffsPrev2[j + 11];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 1.0000000000000001e-01;
}
for (i = 0; i < 3; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



