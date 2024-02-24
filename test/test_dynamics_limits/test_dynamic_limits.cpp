#include <Arduino.h>
#include <unity.h>
//#include "file_to_test.h"



void test_function_should_doBlahAndBlah(void) {
    // test stuff
    TEST_ASSERT_EQUAL(4, 3+1);
}

void test_function_should_doAlsoDoBlah(void) {
    // more test stuff
    TEST_ASSERT_EQUAL(2, 1+1);

}

int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_function_should_doBlahAndBlah);
    RUN_TEST(test_function_should_doAlsoDoBlah);
    return UNITY_END();
}

/**
  * For Arduino framework
  */
void setup() {
    // Wait ~200ms before the Unity test runner
    // establishes connection with a board Serial interface
    delay(200);

    runUnityTests();
}
void loop() {}