/*
 * HC-SR04 Distance Sensor - Source
 * Purpose: Implement trigger pulse and echo timing to compute distance.
 * Notes: Consider using RMT/timers for precise pulse measurement.
 */

#include "project_config.h"
#include "hcsr04.h"
#include "hal_gpio_timers.h"


/*  */