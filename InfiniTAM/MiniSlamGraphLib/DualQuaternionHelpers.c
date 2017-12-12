
#include "dq.h"




void dq_cr_homo( dq_t O, double R[3][3], const double d[3] )
{
   dq_t QR, QT;

#ifdef DQ_CHECK
   assert( fabs(mat3_det(R) - 1.) < DQ_PRECISION );
#endif /* DQ_CHECK */

   dq_cr_rotation_matrix( QR, R );
   dq_cr_translation_vector( QT, d );
   dq_op_mul( O, QT, QR );
}


void dq_cr_rotation_matrix( dq_t O, double R[3][3] )
{
   double Rminus[3][3], Rplus[3][3], Rinv[3][3], B[3][3], eye[3][3];
   double s[3];
   double z2, tz, sz, cz;

#ifdef DQ_CHECK
   assert( fabs(mat3_det(R) - 1.) < DQ_PRECISION );
#endif /* DQ_CHECK */

   /* B = (R-I)(R+I)^{-1} */
   mat3_eye( eye );
   mat3_sub( Rminus, R, eye );
   mat3_add( Rplus, R, eye );
   mat3_inv( Rinv, Rplus );
   mat3_mul( B, Rminus, Rinv );

   /*
    *      0 -b_z  b_y
    * B = b_z  0  -b_x
    *    -b_y b_x   0
    *
    * b = { b_x b_y b_z }
    *
    * s           = b / ||b||
    * tan(theta/2) = ||b||
    */
   s[0] = B[2][1];
   s[1] = B[0][2];
   s[2] = B[1][0];
   tz   = vec3_norm( s );
   /* Avoid normalizing 0. vectors. */
   if (tz > 0.) {
      s[0] /= tz;
      s[1] /= tz;
      s[2] /= tz;
   }
   z2   = atan(tz);

    /*
     * Build the rotational part.
     */
    sz = sin( z2 );
    cz = cos( z2 );
    O[0] = cz;
    O[1] = sz*s[0];
    O[2] = sz*s[1];
    O[3] = sz*s[2];
    O[4] = 0.;
    O[5] = 0.;
    O[6] = 0.;
    O[7] = 0.;
}


void dq_cr_translation_vector( dq_t O, const double t[3] )
{
   O[0] = 1.;
   O[1] = 0.;
   O[2] = 0.;
   O[3] = 0.;
   O[4] = t[0] / 2.;
   O[5] = t[1] / 2.;
   O[6] = t[2] / 2.;
   O[7] = 0.;
}


void dq_op_mul( dq_t PQ, const dq_t P, const dq_t Q )
{
   dq_t T;
   /* Multiplication table:
    *
    *  Q1*Q2 | Q2.1  Q2.i  Q2.j  Q2.k  Q2.ei  Q2.ej  Q2.ek  Q2.e
    *  ------+---------------------------------------------------
    *  Q1.1  |   1     i     j     k    ei     ej     ek      e
    *  Q1.i  |   i    -1     k    -j    -e     ek     -ej    ei
    *  Q1.j  |   j    -k    -1     i    -ek    -e     ei     ej
    *  Q1.k  |   k     j    -i    -1    ej     -ei    -e     ek
    *  Q1.ei |  ei    -e    ek    -ej    0      0      0      0
    *  Q1.ej |  ej    -ek   -e    ei     0      0      0      0
    *  Q1.ek |  ek    ej    -ei   -e     0      0      0      0
    *  Q1.e  |   e    ei     ej   ek     0      0      0      0
    *
    *  We can also decomopose the problem into quaternion multiplication:
    *
    *  Q = q + \epsilon q0
    *  P = p + \epsilon p0
    *  Q*P = q*p + \epsilon(q*p0 + q0*p)
    *
    *  We can treat quaternion multiplication as:
    *
    *  q1 = (r1, v1)
    *  q2 = (r2, v2)
    *  q1*q2 = (r1*r2-v1.v2, r1*v2 + r2*v1 + v1 x v2)
    */
   /* Real quaternion. */
   T[0] = P[0]*Q[0] - P[1]*Q[1] - P[2]*Q[2] - P[3]*Q[3];
   T[1] = P[0]*Q[1] + P[1]*Q[0] + P[2]*Q[3] - P[3]*Q[2];
   T[2] = P[0]*Q[2] + P[2]*Q[0] - P[1]*Q[3] + P[3]*Q[1];
   T[3] = P[0]*Q[3] + P[3]*Q[0] + P[1]*Q[2] - P[2]*Q[1];

   /* Dual unit Quaternion. */
   T[4] = P[4]*Q[0] + P[0]*Q[4] + P[7]*Q[1] + P[1]*Q[7] -
          P[6]*Q[2] + P[2]*Q[6] + P[5]*Q[3] - P[3]*Q[5];
   T[5] = P[5]*Q[0] + P[0]*Q[5] + P[6]*Q[1] - P[1]*Q[6] +
          P[7]*Q[2] + P[2]*Q[7] - P[4]*Q[3] + P[3]*Q[4];
   T[6] = P[6]*Q[0] + P[0]*Q[6] - P[5]*Q[1] + P[1]*Q[5] +
          P[4]*Q[2] - P[2]*Q[4] + P[7]*Q[3] + P[3]*Q[7];
   T[7] = P[7]*Q[0] + P[0]*Q[7] - P[1]*Q[4] - P[4]*Q[1] -
          P[2]*Q[5] - P[5]*Q[2] - P[3]*Q[6] - P[6]*Q[3];

   /* Copy over results. */
   memcpy( PQ, T, sizeof(dq_t) );
}


void mat3_eye( double M[3][3] )
{
   M[0][0] = 1.;
   M[0][1] = 0.;
   M[0][2] = 0.;
   M[1][0] = 0.;
   M[1][1] = 1.;
   M[1][2] = 0.;
   M[2][0] = 0.;
   M[2][1] = 0.;
   M[2][2] = 1.;
}


void mat3_add( double out[3][3], double A[3][3], double B[3][3] )
{
   int c,r;
   for (c=0; c<3; c++) {
      for (r=0; r<3; r++) {
         out[r][c] = A[r][c] + B[r][c];
      }
   }
}


void mat3_sub( double out[3][3], double A[3][3], double B[3][3] )
{
   int c,r;
   for (c=0; c<3; c++) {
      for (r=0; r<3; r++) {
         out[r][c] = A[r][c] - B[r][c];
      }
   }
}

void mat3_inv( double out[3][3], double in[3][3] )
{
   double det;

   det = mat3_det(in);

#ifdef DQ_CHECK
   assert( fabs(det) > DQ_PRECISION );
#endif /* DQ_CHECK */

   out[0][0] = (in[1][1]*in[2][2] - in[2][1]*in[1][2])/det;
   out[0][1] = (in[0][2]*in[2][1] - in[2][2]*in[0][1])/det;
   out[0][2] = (in[0][1]*in[1][2] - in[1][1]*in[0][2])/det;
   out[1][0] = (in[1][2]*in[2][0] - in[2][2]*in[1][0])/det;
   out[1][1] = (in[0][0]*in[2][2] - in[2][0]*in[0][2])/det;
   out[1][2] = (in[0][2]*in[1][0] - in[1][2]*in[0][0])/det;
   out[2][0] = (in[1][0]*in[2][1] - in[2][0]*in[1][1])/det;
   out[2][1] = (in[0][1]*in[2][0] - in[2][1]*in[0][0])/det;
   out[2][2] = (in[0][0]*in[1][1] - in[1][0]*in[0][1])/det;
}


void mat3_mul( double AB[3][3], double A[3][3], double B[3][3] )
{
   int c,r;
   double T[3][3];
   for (c=0; c<3; c++) {
      for (r=0; r<3; r++) {
         T[r][c] = A[r][0]*B[0][c] + A[r][1]*B[1][c] + A[r][2]*B[2][c];
      }
   }
   memcpy( AB, T, sizeof(double)*3*3 );
}


