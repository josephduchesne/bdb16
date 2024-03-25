#include <Arduino.h>
#include <unity.h>
#include <dshot/esc.h>

void test_dshot_convert_throttle_3d(void) {
    TEST_ASSERT_EQUAL_UINT16(0, DShot::ESC::convertThrottle3D(0.0f));
    TEST_ASSERT_EQUAL_UINT16(0, DShot::ESC::convertThrottle3D(0.000001f));
    TEST_ASSERT_EQUAL_UINT16(0, DShot::ESC::convertThrottle3D(-0.00003f));
    TEST_ASSERT_EQUAL_UINT16(1047, DShot::ESC::convertThrottle3D(5.0f));       // max direction1
    TEST_ASSERT_EQUAL_UINT16(1047, DShot::ESC::convertThrottle3D(1.0f));       // max direction1
    TEST_ASSERT_EQUAL_UINT16(1046, DShot::ESC::convertThrottle3D(0.999f));       // max direction1 - 0.1%
    TEST_ASSERT_EQUAL_UINT16(48, DShot::ESC::convertThrottle3D(0.001f));   // min non-zero direction1
    TEST_ASSERT_EQUAL_UINT16(2047, DShot::ESC::convertThrottle3D(-1.0f));    // max direction2
    TEST_ASSERT_EQUAL_UINT16(2047, DShot::ESC::convertThrottle3D(-5.0f));       // max direction2
    TEST_ASSERT_EQUAL_UINT16(2046, DShot::ESC::convertThrottle3D(-0.999f));    // max direction2 - 0.1%
    TEST_ASSERT_EQUAL_UINT16(1048, DShot::ESC::convertThrottle3D(-0.001f));  // min non-zero direction2
}

int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_dshot_convert_throttle_3d);
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