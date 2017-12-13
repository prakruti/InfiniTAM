#define DQ_PRECISION    1e-10 

typedef double dq_t[8];

void dq_cr_rotation_matrix( dq_t O, double R[3][3] );


void dq_cr_translation( dq_t O, double t, const double s[3] );

// Convert SE3 to dual quaternion
void dq_se3_to_dquat( dq_t O, const ORUtils::SE3Pose in_SE3)
void dq_cr_homo( dq_t O, double R[3][3], const double d[3] );

void dq_op_extract( double R[3][3], double d[3], const dq_t Q );


