#ifndef INC_ORIENTATION_EKF_RP_CONFIG_H_
#define INC_ORIENTATION_EKF_RP_CONFIG_H_

// ===================== DIMENZIJE =====================
#define EKF_STATE_DIM     6
#define EKF_MEAS_ACCEL    3

// ===================== INDEKSI =====================
#define IDX_ROLL    0
#define IDX_PITCH   1
#define IDX_YAW     2
#define IDX_BIAS_X  3
#define IDX_BIAS_Y  4
#define IDX_BIAS_Z  5

// ===================== MATRICE ŠUMA (KONSTANTE) =====================
#define Q_ANGLE     0.0001f
#define Q_BIAS      0.00001f
#define R_ACCEL     1.0f

// ===================== INICIJALIZACIJA =====================
#define P_INIT      0.1f

#endif /* INC_ORIENTATION_EKF_RP_CONFIG_H_ */
