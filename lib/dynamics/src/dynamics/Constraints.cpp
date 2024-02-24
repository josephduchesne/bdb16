#include "dynamics/Constraints.h"

namespace Constraints {

/**
 * @brief Limit a velocity
 * 
 * @param v_old Existing velocity
 * @param v_new Intended new velocity
 * @param v_max Maximum velocity
 * 
 * @return float capped velocity
 */
float capVelocity(float v_new, const float v_max) {
    return constrain(v_new, -v_max, v_max);  // capped velocity
}

/**
 * @brief Limit the acceleration of a velocity
 * 
 * @param v_old Existing velocity
 * @param v_new Intended new velocity
 * @param dt    Time step (seconds)
 * @param a_max Maximum acceleration
 * 
 * @return float acceleration capped velocity
 */
float capAccleration(float v_old, float v_new, const float dt, const float a_max) {
    if (dt > 0.0f) {  // cap acceleration, provided dt > 0. 
        float a = (v_new - v_old)/dt;       // compute accleration
        a = constrain(a, -a_max, a_max);    // cap it
        v_new = v_old + a*dt;               // recalculate v_new with v_old and capped acceleration
    }
    return v_new;
}


/**
 * @brief Return v_new limited by v_max and a_max
 * 
 * @param v_old Existing velocity
 * @param v_new Intended new velocity
 * @param dt    Time step (seconds)
 * @param v_max Maximum velocity
 * @param a_max Maximum acceleration
 * @return float 
 */
float capVelocityAndAcceleration(float v_old, float v_new, const float dt, const float v_max, const float a_max) {
    // cap velocity
    v_new = capVelocity(v_new, v_max);
    v_new = capAccleration(v_old, v_new, dt, a_max);
    
    return v_new;
}

}