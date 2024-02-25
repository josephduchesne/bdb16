#include "dynamics/DifferentialModel.h"
#include "dynamics/Constraints.h"
#include <math.h>

DifferentialModel::DifferentialModel(float wheel_diameter, float wheel_distance, float drive_kV, float battery_voltage, float v_max, float a_max, float w_max, float wa_max)
    : 
            wheel_diameter(wheel_diameter), wheel_distance(wheel_distance), drive_kV(drive_kV), battery_voltage(battery_voltage),
            v_max(v_max), a_max(a_max), w_max(w_max), wa_max(wa_max), wheel_radius(wheel_diameter/2.0f)
    {
        
    }

float DifferentialModel::escThrottleToVelocity(const float throttle) {
    // throttle is in units of max achievable velocity (100% throttle = velocity at kV*Vbatt)
    // radius * maxRPM * rpm/rad = m/s
    return constrain(throttle, -1.0f, 1.0f) * drive_kV * battery_voltage * rpm_to_rad_per_sec * wheel_radius;
}

float DifferentialModel::velocityToEscThrottle(const float velocity) {
    float out = velocity/(drive_kV * battery_voltage * rpm_to_rad_per_sec * wheel_radius);
    return constrain(out, -1.0f, 1.0f);
}


float DifferentialModel::applyLinearConstraints(float old, float in, const float dt) {
    // todo: do this only in the velocity domain... This function shoud be removed
    old = escThrottleToVelocity(old);
    in = escThrottleToVelocity(in);
    float out = Constraints::capVelocityAndAcceleration(old, in, dt, v_max, a_max);
    // Serial2.printf("Velocity %0.4f->%0.4f capped at %0.4f\n", old, in, out);
    return velocityToEscThrottle(out);
}

void DifferentialModel::differentialVelocityToTwist(float L, float R, float& v, float& w) {
    v = (L+R)/2.0f;
    w = (R-L)/wheel_distance;
}

void DifferentialModel::twistToDifferentialVelocity(float v, float w, float& L, float& R) {
    L = v - 0.5f*w*wheel_distance;
    R = v + 0.5f*w*wheel_distance;
}

void DifferentialModel::applyConstraints(float old_v, float old_w, float new_v, float new_w, 
                                         const float dt, float& out_v, float& out_w)
{
    
    // apply linear constraints
    out_v = Constraints::capVelocityAndAcceleration(old_v, new_v, dt, v_max, a_max);

    // apply angular constraints
    out_w = Constraints::capVelocityAndAcceleration(old_w, new_w, dt, w_max, wa_max);
}