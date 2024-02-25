#pragma once

#include "Arduino.h"

class DifferentialModel
{
public:
    /**
     * @brief Construct a new Differential Model object
     * 
     * @param wheel_diameter        Wheel diameter in meters
     * @param wheel_distance        Nominal wheel distance in meters (center to center?)
     * @param drive_kV              Drive motor kV (divided by gear reduction)
     * @param battery_voltage       Battery peak nominal voltage (@todo: Possibly make this dynamic based on filtered battery readings)
     * @param v_max                 Max velocity (m/s)            
     * @param a_max                 Max acceleration (m/s^2)
     * @param w_max                 Max angular velocity (rad/s)
     * @param wa_max                Max angular acceleration (rad/s^2)
     */
    DifferentialModel(float wheel_diameter, float wheel_distance, float drive_kV, float battery_voltage, float v_max, float a_max, float w_max, float wa_max);

    /**
     * @brief Convert a throttle value (1.0 to -1.0) to a velocity value (m/s)
     * 
     * @param throttle 
     * @return float 
     */
    float escThrottleToVelocity(const float throttle);

    /**
     * @brief Convert linear velocity back to a throttle value
     * 
     * @param velocity The velocity input
     * @return float   Reconstituted throttle output
     */
    float velocityToEscThrottle(const float velocity);

    /**
     * @brief Apply linear constraints to a left or right channel
     * 
     * @param old Previous in throttle output value (last timestep)
     * @param in New throttle input
     * @param dt Timestep since last output
     * 
     * @return velocity+acceleration constrained throttle value 
     */
    float applyLinearConstraints(float old, float in, const float dt);
        
    void differentialVelocityToTwist(float L, float R, float& v, float& w);

    void twistToDifferentialVelocity(float v, float w, float& L, float& R);

    /**
     * @brief Constrain linear and angular velocity
     * 
     * @param old_v Old linear velocity
     * @param old_w Old angulage velocity
     * @param new_v Desired linear velocity
     * @param new_w Desired angular velocity
     * @param dt    Delta time (seconds)
     * @param out_v Output constrained velocity
     * @param out_w Output constrained angular velocity
     */
    void applyConstraints(float old_v, float old_w, float new_v, float new_w, const float dt, float& out_v, float& out_w);

    void scaleArcadeInput(float in_v, float in_w, float& out_v, float& out_w) {
        out_v = constrain(in_v, -1.0f, 1.0f) * v_max;
        out_w = constrain(in_w, -1.0f, 1.0f) * w_max;
    }

    static constexpr float rpm_to_rad_per_sec = (M_PI*2.0f/60.0f);

    const float wheel_diameter;
    const float wheel_radius;
    const float wheel_distance;
    const float drive_kV;
    float battery_voltage;
    const float v_max;
    const float a_max;
    const float w_max;
    const float wa_max;

private:


};
