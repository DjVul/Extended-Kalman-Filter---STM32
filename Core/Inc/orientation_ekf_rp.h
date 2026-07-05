#ifndef INC_ORIENTATION_EKF_RP_H_
#define INC_ORIENTATION_EKF_RP_H_

#include "orientation_ekf_rp_config.h"
#include "linear_algebra.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

// ===================== TIPOVI =====================
typedef struct {
    float roll;          // radijani
    float pitch;         // radijani
    float yaw;           // radijani
} ekf_angles_t;

typedef struct {
    float roll;          // radijani
    float pitch;         // radijani
    float yaw;           // radijani
    float bias_x;        // rad/s
    float bias_y;        // rad/s
    float bias_z;        // rad/s
} ekf_state_t;

// ==================== FUNKCIJE =====================

#define DEG_TO_RAD(deg) ((deg) * 3.141592653589793f / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / 3.141592653589793f)

/* Inicijalizacija:
 * Resetuje filter na početno stanje
 * */
void ekf_init(void);

/* Predikcija:
 * poziva se svaki put kad stigne žiroskop
 * Ulaz: dt [s], gx, gy, gz [rad/s] (SIROVI PODACI, SA BIASOM)
 * */
void ekf_predict(const float dt, const float gx, const float gy, const float gz);

/* Korekcija:
 * Poziva se kada stigne akcelerometar
 * Ulaz: ax, ay, az [m/s^2]
 * */
void ekf_update_accel(const float ax, const float ay, const float az);


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
void ekf_get_angles(float *roll, float *pitch, float *yaw);

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
void ekf_get_radians(float *roll, float *pitch, float *yaw);

/**
 * Dohvata celokupno stanje filtera (uglove i biase).
 *
 * Ova funkcija kopira čitav vektor stanja iz filtera u korisničku strukturu.
 * Stanje sadrži: Roll, Pitch, Yaw (u radijanima) i biase žiroskopa (u rad/s).
 */
void ekf_get_state(ekf_state_t *state);


#endif /* INC_ORIENTATION_EKF_RP_H_ */
