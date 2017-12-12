#define DQ_PRECISION    1e-10 

typedef double dq_t[8];

void dq_cr_rotation_matrix( dq_t O, double R[3][3] );


void dq_cr_translation( dq_t O, double t, const double s[3] );


void dq_cr_homo( dq_t O, double R[3][3], const double d[3] );