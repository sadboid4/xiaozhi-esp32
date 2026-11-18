#pragma once

#include <driver/gpio.h>

namespace duybot {

// L298N motor driver. ENA/ENB jumpers must be removed for PWM control.
inline constexpr gpio_num_t kMotorLeftEnable = GPIO_NUM_1;
inline constexpr gpio_num_t kMotorRightEnable = GPIO_NUM_2;
inline constexpr gpio_num_t kMotorLeftIn1 = GPIO_NUM_11;
inline constexpr gpio_num_t kMotorLeftIn2 = GPIO_NUM_12;
inline constexpr gpio_num_t kMotorRightIn1 = GPIO_NUM_13;
inline constexpr gpio_num_t kMotorRightIn2 = GPIO_NUM_14;

// Chassis sensors.
inline constexpr gpio_num_t kUltrasonicTrigger = GPIO_NUM_10;
inline constexpr gpio_num_t kUltrasonicEcho = GPIO_NUM_18;
inline constexpr gpio_num_t kLineSensor = GPIO_NUM_3;
inline constexpr gpio_num_t kTouchSensor = GPIO_NUM_8;

}  // namespace duybot
