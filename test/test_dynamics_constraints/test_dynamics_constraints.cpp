#include <Arduino.h>
#include <unity.h>
#include "dynamics/Constraints.h"

using namespace Constraints;

void test_function_should_cap_velocity(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, capVelocity(0.0f, 1.0f));             // no-op
    TEST_ASSERT_EQUAL_FLOAT(0.0f, capVelocity(1.0f, 0.0f));             // no velocity allowed
    TEST_ASSERT_EQUAL_FLOAT(3.0f, capVelocity(5.0f, 3.0f));             // cap high
    TEST_ASSERT_EQUAL_FLOAT(-100.0f, capVelocity(-200.0f, 100.0f));     // cap low
}

void test_function_should_cap_accleration(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.1f, capAccleration(0.0f, 0.1f, 0.05f, 10.0f));        // no change
    TEST_ASSERT_EQUAL_FLOAT(-0.1f, capAccleration(0.0f, -0.1f, 0.05f, 10.0f));      // negative no change
    TEST_ASSERT_EQUAL_FLOAT(100.0f, capAccleration(0.0f, 100.0f, 0.0f, 10.0f));     // bad input (zero dt), no limit applied
    TEST_ASSERT_EQUAL_FLOAT(0.5f, capAccleration(0.0f, 1.0f, 0.05f, 10.0f));        // Typical use, velocity capped by acceleration
    TEST_ASSERT_EQUAL_FLOAT(-0.55f, capAccleration(0.0f, -1.0f, 0.05f, 11.0f));     // Typical use, negative velocity capped by acceleration
}

void test_function_should_cap_v_and_a(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.1f, capVelocityAndAcceleration(0.0f, 0.1f, 0.05f, 1.0f, 10.0f));          // no caps applied
    TEST_ASSERT_EQUAL_FLOAT(0.0f, capVelocityAndAcceleration(0.0f, 0.1f, 0.05f, 0.0f, 10.0f));          // v cap: no velocity allowed
    TEST_ASSERT_EQUAL_FLOAT(3.0f, capVelocityAndAcceleration(4.9f, 5.0f, 0.05f, 3.0f, 1000.0f));        // v cap high
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, capVelocityAndAcceleration(-9.0f, -20.0f, 0.05f, 10.0f, 1000.0f));  // v cap low
    TEST_ASSERT_EQUAL_FLOAT(100.0f, capVelocityAndAcceleration(0.0f, 100.0f, 0.0f, 100.0f, 10.0f));     // a cap bad input (zero dt), no a limit applied
    TEST_ASSERT_EQUAL_FLOAT(0.5f, capVelocityAndAcceleration(0.0f, 1.0f, 0.05f, 2.0f, 10.0f));          // velocity capped by acceleration
    TEST_ASSERT_EQUAL_FLOAT(-0.55f, capVelocityAndAcceleration(0.0f, -1.0f, 0.05f, 2.0f, 11.0f));       // negative velocity capped by acceleration
}

int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_function_should_cap_velocity);
    RUN_TEST(test_function_should_cap_accleration);
    RUN_TEST(test_function_should_cap_v_and_a);
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