#pragma once

#include <Arduino.h>

namespace Constraints {

/**
 * @brief Limit a velocity
 * 
 * @param v_new Intended new velocity
 * @param v_max Maximum velocity
 * 
 * @return float capped velocity
 */
float capVelocity(float v_new, const float v_max);

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
float capAccleration(float v_old, float v_new, const float dt, const float a_max);


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
float capVelocityAndAcceleration(float v_old, float v_new, const float dt, const float v_max, const float a_max);

}