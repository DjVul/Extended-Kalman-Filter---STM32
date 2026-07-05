#include "orientation_ekf_rp.h"

static float x[EKF_STATE_DIM];                        			// Stanje
static float P[EKF_STATE_DIM][EKF_STATE_DIM];                  	// Kovarijansa
static float Q[EKF_STATE_DIM][EKF_STATE_DIM];                  	// Procesni šum
static float R_accel[EKF_MEAS_ACCEL][EKF_MEAS_ACCEL];          	// Merni šum

// Pomocne funkcije

static void init_QPR(void)
{
	memset(Q, 0, sizeof(Q));

	Q[IDX_ROLL][IDX_ROLL] = Q_ANGLE;
	Q[IDX_PITCH][IDX_PITCH] = Q_ANGLE;
	Q[IDX_YAW][IDX_YAW] = Q_ANGLE;

	Q[IDX_BIAS_X][IDX_BIAS_X] = Q_BIAS;
	Q[IDX_BIAS_Y][IDX_BIAS_Y] = Q_BIAS;
	Q[IDX_BIAS_Z][IDX_BIAS_Z] = Q_BIAS;

	memset(P, 0, sizeof(P));

	for(uint8_t i = 0; i < EKF_STATE_DIM; i++)
		P[i][i] = P_INIT;


	memset(R_accel, 0, sizeof(R_accel));

	for(uint8_t i = 0; i < EKF_MEAS_ACCEL; i++)
		R_accel[i][i] = R_ACCEL;
}



/* Inicijalizacija:
 * Resetuje filter na početno stanje
 * */
void ekf_init(void)
{
	// Inicijalizuj sve parametre za racunanje kovarijanse
	init_QPR();

	// Inicijalizuj pocetno stanje sistema
	memset(x, 0, sizeof(x));

}

/* Predikcija:
 * poziva se svaki put kad stigne žiroskop
 * Ulaz: dt [s], gx, gy, gz [rad/s] (SIROVI PODACI, SA BIASOM)
 * */
void ekf_predict(const float dt, const float gx, const float gy, const float gz)
{
	// Ekstrapoliranje novog stanja
	x[IDX_ROLL] += (gx - x[IDX_BIAS_X])*dt;
	x[IDX_PITCH]+= (gy - x[IDX_BIAS_Y])*dt;
	x[IDX_YAW] 	+= (gz - x[IDX_BIAS_Z])*dt;

	// Ekstrapoliranje procesnog šuma
	// P (n+1,n) = F * P (n,n) * F^t + Q

	// Matrica F prelaska stanja
    float F[EKF_STATE_DIM][EKF_STATE_DIM] = {
        {1, 0, 0, -dt, 0, 0},
        {0, 1, 0, 0, -dt, 0},
        {0, 0, 1, 0, 0, -dt},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };

    // Privremene matrice za množenje
    float FP[EKF_STATE_DIM][EKF_STATE_DIM];     // F * P
    float FPFt[EKF_STATE_DIM][EKF_STATE_DIM];   // F * P * F^T

    // FP = P * F^T
    matrix_mul_transpose((float*)P, (float*)F, (float*)FP,
                         EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);

    // FPFt = F * FP = F * P * F^T
    matrix_mul((float*)F, (float*)FP, (float*)FPFt,
               EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);

    // P = FPFt + Q
    matrix_add((float*)FPFt, (float*)Q, (float*)P,
               EKF_STATE_DIM, EKF_STATE_DIM);
}

/* Korekcija:
 * Poziva se kada stigne podatak sa akcelerometra
 * Ulaz: ax, ay, az [m/s^2]
 * */
void ekf_update_accel(const float ax, const float ay, const float az)
{
	    // Update (rezidual): y = z - h(x)
	    // h(x) - očekivano merenje akcelerometra na osnovu trenutnih uglova
	    float roll = x[IDX_ROLL];
	    float pitch = x[IDX_PITCH];
	    float g = 9.81f;

	    float h[EKF_MEAS_ACCEL];
	    h[0] = -g * sinf(pitch);                 // očekivano ax
	    h[1] =  g * cosf(pitch) * sinf(roll);    // očekivano ay
	    h[2] =  g * cosf(pitch) * cosf(roll);    // očekivano az

	    // Update (razlika između stvarnog i očekivanog merenja)
	    float y[EKF_MEAS_ACCEL];
	    y[0] = ax - h[0];
	    y[1] = ay - h[1];
	    y[2] = az - h[2];

	    // Jakobijan merenja H (3x6)
	    float H[EKF_MEAS_ACCEL][EKF_STATE_DIM] = {0};

	    // Prva vrsta (ax = -g * sin(pitch))
	    H[0][IDX_PITCH] = -g * cosf(pitch);

	    // Druga vrsta (ay = g * cos(pitch) * sin(roll))
	    H[1][IDX_ROLL]  =  g * cosf(pitch) * cosf(roll);
	    H[1][IDX_PITCH] = -g * sinf(pitch) * sinf(roll);

	    // Treća vrsta (az = g * cos(pitch) * cos(roll))
	    H[2][IDX_ROLL]  = -g * cosf(pitch) * sinf(roll);
	    H[2][IDX_PITCH] = -g * sinf(pitch) * cosf(roll);

	    // S = H * P * H^T + R
	    float HP[EKF_MEAS_ACCEL][EKF_STATE_DIM];       // H * P (3x6)
	    float HPHt[EKF_MEAS_ACCEL][EKF_MEAS_ACCEL];    // H * P * H^T (3x3)
	    float S[EKF_MEAS_ACCEL][EKF_MEAS_ACCEL];       // HPHt + R (3x3)

	    // HP = H * P
	    matrix_mul((float*)H, (float*)P, (float*)HP,
	               EKF_MEAS_ACCEL, EKF_STATE_DIM, EKF_STATE_DIM);

	    // HPHt = HP * H^T
	    matrix_mul_transpose((float*)HP, (float*)H, (float*)HPHt,
	                         EKF_MEAS_ACCEL, EKF_STATE_DIM, EKF_MEAS_ACCEL);

	    // S = HPHt + R_accel
	    matrix_add((float*)HPHt, (float*)R_accel, (float*)S,
	               EKF_MEAS_ACCEL, EKF_MEAS_ACCEL);

	    // Inverzija S (3x3) - za Kalmanovo pojačanje

	    float S_inv[EKF_MEAS_ACCEL][EKF_MEAS_ACCEL];
	    if (!matrix_inv_3x3(S, S_inv)) {
	        // Ako je S singularna, preskoči update
	        return;
	    }

	    // Kalmanovo pojačanje: K = P * H^T * S^-1

	    float Ht[EKF_STATE_DIM][EKF_MEAS_ACCEL];       // H^T (6x3)
	    float PHt[EKF_STATE_DIM][EKF_MEAS_ACCEL];      // P * H^T (6x3)
	    float K[EKF_STATE_DIM][EKF_MEAS_ACCEL];        // Kalmanov pojačanje (6x3)

	    // Ht = H^T
	    matrix_transpose((float*)H, (float*)Ht,
	                     EKF_MEAS_ACCEL, EKF_STATE_DIM);

	    // PHt = P * H^T
	    matrix_mul((float*)P, (float*)Ht, (float*)PHt,
	               EKF_STATE_DIM, EKF_STATE_DIM, EKF_MEAS_ACCEL);

	    // K = PHt * S^-1
	    matrix_mul((float*)PHt, (float*)S_inv, (float*)K,
	               EKF_STATE_DIM, EKF_MEAS_ACCEL, EKF_MEAS_ACCEL);

	    // Korekcija stanja: x = x + K * y
	    float Ky[EKF_STATE_DIM];  // K * y (6x1)
	    matrix_mul((float*)K, y, Ky,
	               EKF_STATE_DIM, EKF_MEAS_ACCEL, 1);

	    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
	        x[i] += Ky[i];
	    }

	    // Korekcija kovarijanse: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
	    float KH[EKF_STATE_DIM][EKF_STATE_DIM];       // K * H (6x6)
	    float I_KH[EKF_STATE_DIM][EKF_STATE_DIM];     // I - K*H (6x6)
	    float T[EKF_STATE_DIM][EKF_STATE_DIM];        // Privremena matrica
	    float I[EKF_STATE_DIM][EKF_STATE_DIM];		  // Jedinicna matrica

	    // KH = K * H
	    matrix_mul((float*)K, (float*)H, (float*)KH,
	               EKF_STATE_DIM, EKF_MEAS_ACCEL, EKF_STATE_DIM);

	    memset(I, 0, sizeof(I));
	    for(uint8_t i = 0; i < EKF_STATE_DIM; i++)
	    {
	    	I[i][i] = 1;
	    }
	    // I_KH = I - KH
	    matrix_sub(I, KH, I_KH, EKF_STATE_DIM, EKF_STATE_DIM);

	    // T = I_KH * P
	    matrix_mul((float*)I_KH, (float*)P, (float*)T,
	               EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);

	    // P = T * I_KH^T
	    matrix_mul_transpose((float*)T, (float*)I_KH, (float*)P,
	                         EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);

	    // P = P + K * R * K^T
	    float KR[EKF_STATE_DIM][EKF_MEAS_ACCEL];      // K * R (6x3)
	    float KRKt[EKF_STATE_DIM][EKF_STATE_DIM];     // K * R * K^T (6x6)

	    // KR = K * R
	    matrix_mul((float*)K, (float*)R_accel, (float*)KR,
	               EKF_STATE_DIM, EKF_MEAS_ACCEL, EKF_MEAS_ACCEL);

	    // KRKt = KR * K^T
	    matrix_mul_transpose((float*)KR, (float*)K, (float*)KRKt,
	                         EKF_STATE_DIM, EKF_MEAS_ACCEL, EKF_STATE_DIM);

	    // P = P + KRKt
	    matrix_add((float*)P, (float*)KRKt, (float*)P,
	               EKF_STATE_DIM, EKF_STATE_DIM);

}


/**
 * Dohvata procenjene uglove u STEPENIMA
 *
 * Konvertuje unutrašnje stanje filtera (koje je u radijanima) u stepene.
 * Koristi se za prikaz korisniku
 *
 *  roll Pokazivač na float za Roll ugao [°] (može biti NULL)
 *  pitch Pokazivač na float za Pitch ugao [°] (može biti NULL)
 *  yaw Pokazivač na float za Yaw ugao [°] (može biti NULL)
 */
void ekf_get_angles(float *roll, float *pitch, float *yaw) {
    if (roll)  *roll  = RAD_TO_DEG(x[IDX_ROLL]);
    if (pitch) *pitch = RAD_TO_DEG(x[IDX_PITCH]);
    if (yaw)   *yaw   = RAD_TO_DEG(x[IDX_YAW]);
}

/**
 * Dohvata procenjene uglove u RADIJANIMA
 *
 * Konvertuje unutrašnje stanje filtera (koje je u radijanima) u stepene.
 * Koristi se za prikaz korisniku
 *
 *  roll Pokazivač na float za Roll ugao [rad] (može biti NULL)
 *  pitch Pokazivač na float za Pitch ugao [rad] (može biti NULL)
 *  yaw Pokazivač na float za Yaw ugao [rad] (može biti NULL)
 */
void ekf_get_radians(float *roll, float *pitch, float *yaw) {
    if (roll)  *roll  = x[IDX_ROLL];
    if (pitch) *pitch = x[IDX_PITCH];
    if (yaw)   *yaw   = x[IDX_YAW];
}

/**
 * Dohvata celokupno stanje filtera (uglove i biase).
 *
 * Ova funkcija kopira čitav vektor stanja iz filtera u korisničku strukturu.
 * Stanje sadrži: Roll, Pitch, Yaw (u radijanima) i biase žiroskopa (u rad/s).
 */
void ekf_get_state(ekf_state_t *state) {
    if (state) {
        state->roll   = x[IDX_ROLL];
        state->pitch  = x[IDX_PITCH];
        state->yaw    = x[IDX_YAW];
        state->bias_x = x[IDX_BIAS_X];
        state->bias_y = x[IDX_BIAS_Y];
        state->bias_z = x[IDX_BIAS_Z];
    }
}
