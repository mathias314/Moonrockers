/**
 * @file test_mc.cpp
 * @brief Tests written for the FRC motor controllers. Run with "pio test -f test_mc".
 * @date 2021-12-13
 */
#include <Arduino.h>
#include "SparkMax.h"
#include <unity.h>

SparkMax mtr1(1);
SparkMax mtr4(4);

uint8_t canDat[8] = {0};
uint32_t canId = 0;

/**
 * @brief Dummy function for "sending" can messages (but actually just set some global variables.)
 */
void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t* dat) {
    memset(canDat, 0, sizeof(canDat));
    memcpy(canDat, dat, len);
    canId = id;
}


void printDat(uint8_t dat[]) {
    Serial.print("{");
    for (int i = 0; i < 8; i++) {
        Serial.print(dat[i], HEX);
        Serial.print(", ");
    }
    Serial.println("}");
}

/**
 * @brief Test some basic power commands with the SPARK.
 */
void test_spark_run(void) {
    const uint32_t SET_POWER_ID = 0x2050080;

    // Test some different power values
    mtr1.setPower(1.0);
    TEST_ASSERT_EQUAL(1.0, *((float*)canDat));
    TEST_ASSERT_EQUAL_HEX32(SET_POWER_ID | 0x1, canId);
    mtr4.setPower(1.0);
    TEST_ASSERT_EQUAL(1.0, *((float*)canDat));
    TEST_ASSERT_EQUAL_HEX32(SET_POWER_ID | 0x4, canId);
    mtr1.setPower(0.3);
    TEST_ASSERT_EQUAL(0.3, *((float*)canDat));
    TEST_ASSERT_EQUAL_HEX32(SET_POWER_ID | 0x1, canId);
    mtr1.setPower(-1.0);
    TEST_ASSERT_EQUAL(-1.0, *((float*)canDat));
    TEST_ASSERT_EQUAL_HEX32(SET_POWER_ID | 0x1, canId);
    mtr1.setPower(0.0);
    TEST_ASSERT_EQUAL(0.0, *((float*)canDat));
    TEST_ASSERT_EQUAL_HEX32(SET_POWER_ID | 0x1, canId);
}

/**
 * @brief Test sending the keep-alive command with the SPARK.
 */
void test_spark_keep_alive(void) {
    const uint32_t KEEP_ALIVE_ID = 0x2052C80;
    uint8_t compDat[] = {bit(4) | bit(1), 0, 0, 0, 0, 0, 0, 0};

    delay(20); // There is a 10ms limit on sending...
    FrcMotorController::sendKeepAlive();
    TEST_ASSERT_EQUAL_HEX8_ARRAY(compDat, canDat, 8);
    TEST_ASSERT_EQUAL_HEX32(KEEP_ALIVE_ID, canId);
}

/**
 * @brief Send clear faults message with SPARK.
 */
void test_spark_clear_faults(void) {
    const uint32_t CLEAR_FAULTS_ID = 0x2051B80;
    uint8_t compDat[] = {0, 0, 0, 0, 0, 0, 0, 0};

    mtr1.clearFaults();
    compDat[0] = bit(1);
    TEST_ASSERT_EQUAL_HEX32(CLEAR_FAULTS_ID | 0x1, canId);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(compDat, canDat, 8);
    mtr4.clearFaults();
    compDat[0] = bit(4);
    TEST_ASSERT_EQUAL_HEX32(CLEAR_FAULTS_ID | 0x4, canId);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(compDat, canDat, 8);
}

void setup() {
    delay(1000);

    FrcMotorController::addCanSender(canSend);

    UNITY_BEGIN();
    RUN_TEST(test_spark_run);
    RUN_TEST(test_spark_keep_alive);
    RUN_TEST(test_spark_clear_faults);
    UNITY_END();
}

void loop() {
    delay(500);
}