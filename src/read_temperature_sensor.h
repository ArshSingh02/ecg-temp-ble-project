#ifndef READ_TEMPERATURE_SENSOR_H
#define READ_TEMPERATURE_SENSOR_H

#include <zephyr/logging/log.h>
#include "zephyr/drivers/sensor.h"


int read_temperature_sensor(const struct device *temp_sensor, float *temperature_degC);

#endif