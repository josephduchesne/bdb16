#include <Arduino.h>
#include <unity.h>
#include "dynamics/DifferentialModel.h"

void test_function_should_convert_throttle_to_velocity(void) {
    //todo: Move throttle scale into Differential Model
    DifferentialModel dm(0.055f, 0.152f, 1000.0f/5.0f, 16.8f, 5.3f, 8.47f, 18.25f, 111.4f);
    
    // radius * RPM * rpm/rad = m/s
    constexpr float full_throttle_speed = 0.055f/2.0f*(1000.0f/5.0f*16.8f)*M_PI*2.0f/60.0f;
    TEST_ASSERT_EQUAL_FLOAT(full_throttle_speed, dm.escThrottleToVelocity(1.0f));             // basic +
    TEST_ASSERT_EQUAL_FLOAT(-full_throttle_speed, dm.escThrottleToVelocity(-1.0f));           // basic -
    TEST_ASSERT_EQUAL_FLOAT(full_throttle_speed/2.0f, dm.escThrottleToVelocity(0.5f));        // complex +
    TEST_ASSERT_EQUAL_FLOAT(0.0f, dm.escThrottleToVelocity(0.0f));                            // zero
}

void test_function_should_convert_velocity_to_throttle(void) {
    DifferentialModel dm(0.055f, 0.152f, 1000.0f/5.0f, 16.8f, 5.3f, 8.47f, 18.25f, 111.4f);

    // radius * RPM * rpm/rad = m/s
    constexpr float full_throttle_speed = 0.055f/2.0f*(1000.0f/5.0f*16.8f)*M_PI*2.0f/60.0f;
    TEST_ASSERT_EQUAL_FLOAT(1.0, dm.velocityToEscThrottle(full_throttle_speed));             // basic +
    TEST_ASSERT_EQUAL_FLOAT(-1.0, dm.velocityToEscThrottle(-full_throttle_speed));           // basic -
    TEST_ASSERT_EQUAL_FLOAT(0.5f, dm.velocityToEscThrottle(full_throttle_speed/2.0f));       // complex +
    TEST_ASSERT_EQUAL_FLOAT(0.0f, dm.velocityToEscThrottle(0.0f));                           // zero
}

void test_function_should_apply_linear_constraints(void) {
    DifferentialModel dm(0.055f, 0.152f, 1000.0f/5.0f, 16.8f, 5.3f, 8.47f, 18.25f, 111.4f);
    constexpr float ft = 0.055f/2.0f*(1000.0f/5.0f*16.8f)*M_PI*2.0f/60.0f;  // full throttle

    TEST_ASSERT_EQUAL_FLOAT(0.5f, dm.applyLinearConstraints(0.5f, 0.5f, 0.01f));          // basic +, under v_max
    TEST_ASSERT_EQUAL_FLOAT(5.3f/ft, dm.applyLinearConstraints(0.0f, 1.0f, 0.0f));     // basic +, over v_max
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, dm.applyLinearConstraints(-0.5f, -0.5f, 0.01f));       // basic -, under v_max
    TEST_ASSERT_EQUAL_FLOAT(-5.3f/ft, dm.applyLinearConstraints(-1.0f, -1.0f, 0.0f));              // basic -, over v_max
    TEST_ASSERT_EQUAL_FLOAT(0.5f+8.47f*0.01f/ft, dm.applyLinearConstraints(0.5, 1.0, 0.01f));   // +, over a_max

}
void test_diff_to_twist() {
    DifferentialModel dm(0.055f, 0.152f, 1000.0f/5.0f, 16.8f, 5.3f, 8.47f, 18.25f, 111.4f);
    constexpr float ft = 0.055f/2.0f*(1000.0f/5.0f*16.8f)*M_PI*2.0f/60.0f;                              // full throttle

    float v, w;
    dm.differentialVelocityToTwist(0.0f, 0.0f, v, w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v);          // no velocity
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w);          // no angular velocity

    dm.differentialVelocityToTwist(1.0f, 1.0f, v, w);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, v);          // forward 1m/s
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w);          // no angular velocity

    dm.differentialVelocityToTwist(-0.5f, 0.5f, v, w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v);          // no velocity
    TEST_ASSERT_EQUAL_FLOAT(1.0/0.152, w);          // spinning on the spot, ccw (+)
}

void test_twist_to_diff() {
    DifferentialModel dm(0.055f, 0.152f, 1000.0f/5.0f, 16.8f, 5.3f, 8.47f, 18.25f, 111.4f);
    constexpr float ft = 0.055f/2.0f*(1000.0f/5.0f*16.8f)*M_PI*2.0f/60.0f;                              // full throttle

    float L, R;
    dm.twistToDifferentialVelocity(0.0f, 0.0f, L, R);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, L);          // no L
    TEST_ASSERT_EQUAL_FLOAT(0.0f, R);          // no R

    dm.twistToDifferentialVelocity(1.0f, 0.0f, L, R);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, L);          // forward 1m/s
    TEST_ASSERT_EQUAL_FLOAT(1.0f, R);          // forward 1m/s

    dm.twistToDifferentialVelocity(-1.0f, 0.0f, L, R);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, L);          // rev 1m/s
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, R);          // rev 1m/s

    dm.twistToDifferentialVelocity(0.0, 1.0/0.152, L, R);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, L);          // no velocity
    TEST_ASSERT_EQUAL_FLOAT(0.5, R);          // spinning on the spot, ccw (+)
}

void test_scale_arcade_inputs() {
    DifferentialModel dm(0.055f, 0.152f, 1000.0f/5.0f, 16.8f, 5.3f, 8.47f, 18.25f, 111.4f);

    float v_out, w_out;
    dm.scaleArcadeInput(1.0, 0.0, v_out, w_out);
    TEST_ASSERT_EQUAL_FLOAT(5.3f, v_out);          // max v
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w_out);          // max w

    dm.scaleArcadeInput(0.5, 0.0, v_out, w_out);
    TEST_ASSERT_EQUAL_FLOAT(5.3f/2.0f, v_out);     // max V
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w_out);          // max w

    dm.scaleArcadeInput(-1.0, 0.0, v_out, w_out);
    TEST_ASSERT_EQUAL_FLOAT(-5.3f, v_out);         // max v
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w_out);          // max w

    dm.scaleArcadeInput(0.0, 1.0, v_out, w_out);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v_out);          // max v
    TEST_ASSERT_EQUAL_FLOAT(18.25f, w_out);        // max w

    dm.scaleArcadeInput(0.0, -1.0, v_out, w_out);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v_out);          // max v
    TEST_ASSERT_EQUAL_FLOAT(-18.25f, w_out);       // max w
}

void test_apply_constraints() {
    DifferentialModel dm(0.055f, 0.152f, 1000.0f/5.0f, 16.8f, 5.3f, 8.47f, 18.25f, 111.4f);
   
    float out_v, out_w;

    // input, from zero, 100% stick. Gets linear acceleration bounded 
    dm.applyConstraints(0.0f, 0.0f, 5.3f, 0.0f, 0.05, out_v, out_w);
    TEST_ASSERT_EQUAL_FLOAT(8.47f*0.05, out_v);       // a little linear acceleration
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out_w);       // no angular


    // input, at speed, 100% stick. Not bounded
    dm.applyConstraints(5.3f, 0.0f, 5.3f, 0.0f, 0.05, out_v, out_w);
    TEST_ASSERT_EQUAL_FLOAT(5.3f, out_v);       // max velocity
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out_w);       // no angular


    // input, at speed, 100% stick. Not bounded
    dm.applyConstraints(-4.0f, 0.0f, -4.0f, 0.0f, 0.05, out_v, out_w);
    TEST_ASSERT_EQUAL_FLOAT(-4.0f, out_v);       // max velocity
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out_w);       // no angular

    // input, at speed, over 100% stick. Velocity bounded
    dm.applyConstraints(5.3f, 0.0f, 5.4f, 0.0f, 0.05, out_v, out_w);
    TEST_ASSERT_EQUAL_FLOAT(5.3f, out_v);       // max velocity
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out_w);       // no angular

    // rotate on the spot, from zero, wa limited
    dm.applyConstraints(0.0f, 0.0f, 0.0f, 50.0f, 0.05, out_v, out_w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out_v);       // no velocity
    TEST_ASSERT_EQUAL_FLOAT(111.4*0.05f, out_w);       // some angular

    // rotate on the spot, well under limits
    dm.applyConstraints(0.0f, 5.0f, 0.0f, 5.0f, 0.05, out_v, out_w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out_v);       // no velocity
    TEST_ASSERT_EQUAL_FLOAT(5.0f, out_w);       // unbounded angular

    // rotate on the spot, beyond w_max
    dm.applyConstraints(0.0f, 18.25f, 0.0f, 20.0f, 0.05, out_v, out_w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out_v);       // no velocity
    TEST_ASSERT_EQUAL_FLOAT(18.25f, out_w);       // bounded angular

    // rotate on the spot, beyond w_max
    dm.applyConstraints(0.0f, -18.25f, 0.0f, -20.0f, 0.05, out_v, out_w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out_v);       // no velocity
    TEST_ASSERT_EQUAL_FLOAT(-18.25f, out_w);       // wa bounded angular
}

// TODO: Test saturation/desaturation corner cases

int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_function_should_convert_throttle_to_velocity);
    RUN_TEST(test_function_should_convert_velocity_to_throttle);
    RUN_TEST(test_function_should_apply_linear_constraints);
    RUN_TEST(test_diff_to_twist);
    RUN_TEST(test_twist_to_diff);
    RUN_TEST(test_scale_arcade_inputs);
    RUN_TEST(test_apply_constraints);
    return UNITY_END();
}

/**
  * For Arduino framework
  */
void setup() {
    // Wait ~500ms before the Unity test runner
    // establishes connection with a board Serial interface
    delay(500);

    runUnityTests();
}
void loop() {}