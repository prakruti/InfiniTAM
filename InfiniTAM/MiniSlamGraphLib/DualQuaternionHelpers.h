#define DQ_PRECISION    1e-10 

typedef double dq_t[8];



// Convert SE3 to dual quaternion
dq_t dq_se3_to_dquat(const ORUtils::SE3Pose in_SE3)

// Convert dual quaternion to SE3
ORUtils::SE3Pose dq_dquat_to_se3(const dq_t Q )

// some functions on dual quaternions
void dq_op_norm2(double *real, double *dual, const dq_t Q );
dq_t  dq_op_add(const dq_t P, const dq_t Q );

// helper functions needed from SE3->DQuat
void dq_op_mul( dq_t PQ, const dq_t P, const dq_t Q ) ;
void dq_cr_translation_vector( dq_t O, const double t[3] );
void dq_cr_rotation_matrix( dq_t O, double R[3][3] );

// more helper functions
void mat3_mul( double AB[3][3], double A[3][3], double B[3][3] );
void mat3_inv( double out[3][3], double in[3][3] );
void mat3_sub( double out[3][3], double A[3][3], double B[3][3] );
void mat3_add( double out[3][3], double A[3][3], double B[3][3] );
void mat3_eye( double M[3][3] );
double mat3_det( double M[3][3] );
double vec3_norm( const double v[3] );
double vec3_dot( const double u[3], const double v[3] );















