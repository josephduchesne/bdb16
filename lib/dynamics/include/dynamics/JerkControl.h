#pragma once

#include "Arduino.h"
#include <math.h>

namespace Constraints {

/**
 * @brief Class for generating jerk controlled trajectories
 * 
 * Based on:
 * JERK LIMITED VELOCITY PROFILE GENERATION FOR HIGH SPEED INDUSTRIAL ROBOT TRAJECTORIES
 * by Soon Yong Jeong, Yun Jong Choi, PooGyeon Park, Seung Gap Choi, 2005
 * https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=b0908986c0d43573a90f790c0480f6ec5c834262
 * 
 */
class JerkControl {
public:

    /**
     * @brief Construct a new Jerk Control object
     * 
     * @param V Maximum velocity 
     * @param A Maximum acceleration
     * @param D Maximum deceleration
     * @param J Maximum jerk
     */
    JerkControl(float V, float A, float D, float J) 
    : Cbase_({V,A,D,J}), C_(Cbase_)
    {}

    typedef struct ConstraintSet {
        float V;
        float A;
        float D;
        float J;
    } ConstraintSet;

    ConstraintSet Cbase_; // base constraints
    ConstraintSet C_; // currently active constraints


    /**
     * @brief start/end position, and start/end velocity
     * 
     */
    float p0_, pf_, v0_, vf_;

    // Peak velocity for the current equation
    float vp_;


    /**
     * @brief start/end times of each of the 7 phases
     * 
     * e.g. phase 1 is t[0]..t[1], phase N is t[N-1]..t[N]
     * all the way to t7 is t[7]..t[8]
     * 
     * t[0] is alyways 0
     * 
     */
    float t[8] = {0};
    float T[7] = {0};  // Durations of each of the 7 motion phases

    /**
     * @brief Compute Vpeak, assuming no constant velocity region 
     * 
     * @return true  There is no constant velocity region (valid results)
     * @return false There is a constant velocity region (no valid results)
     */
    bool computeVpeakNoConstV(float& vp) {

        // Case (T4 = 0, T2>0, T6>0)
        //vp = v0_ + C_.A * C_.A / C_.J + 
        return false;
    }


    // https://github.com/mahmoud-a-ali/scurve_traj_generation?tab=readme-ov-file ?

    /**
     * @brief Calculate parameters for a time optimal trajectory given the inputs
     * 
     * Assumes acceleration first profile. Deceleration first profile is not yet implemented.
     * 
     * @param p0 start position
     * @param pf final position
     * @param v0 start velocity
     * @param vf final velocity
     * @return float Time optimal duration
     */
    float timeOptimalTrajectory(float p0, float pf, float v0, float vf) {
        p0_ = p0;
        pf_ = pf;
        v0_ = v0;
        vf_ = vf;

        C_ = Cbase_; // copy the default constraint set into the working constraint set

        // populate tables?
        

        // Determin Tmin (piecewise eq. 9)
        float Tmin;
        if (v0_ <= vf_) { // cases 1,2
            if ((vf_ - v0) <= C_.A * C_.A / C_.J) {  // case 1
                Tmin = 2.0f * sqrtf((vf_- v0_)/C_.J);
            } else { // case 2
                Tmin = (vf_ - v0_)/C_.A + C_.A/C_.J;
            }
        } else {  // cases 3,4
            if ((v0_ - vf) <= C_.D * C_.D / C_.J) {  // case 3
                Tmin = 2.0f * sqrtf((v0_- vf_)/C_.J);
            } else { // case 4
                Tmin = (v0_ - vf_)/C_.D + C_.D/C_.J;
            }
        }

        // Determine if AFP or DFP (piecewise eq. 10)
        float Lmin = (v0_ + vf_)/2.0f * Tmin;
        //if (v0_ <= vf_ && Lmin < A.)

        // determine if there are constant accleration or deceleration regions (paper eq. 7)
        //if ( vf-v0 > )

    }

};

}