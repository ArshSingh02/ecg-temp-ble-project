#include "read_temperature_sensor.h"

LOG_MODULE_REGISTER(read_temperature_sensor, LOG_LEVEL_DBG);

int read_temperature_sensor(const struct device *temp_sensor, float *temperature_degC) {
    /*  Fetch-n-get temperature sensor data

        INPUTS:
            temp_sensor (const struct device *) - temperature sensor device
            temperature_degC (float *) - pointer to store temperature in degrees Celsius

        RETURNS:
            0 - success
            Otherwise, error code

    */

    struct sensor_value sensor_vals = {.val1 = 0, .val2 = 0};

    int err = sensor_sample_fetch(temp_sensor);
    if (err != 0) {
        LOG_ERR("Temperature sensor fetch(): %d", err);
        return err;
    }
    else {
        err = sensor_channel_get(temp_sensor, SENSOR_CHAN_AMBIENT_TEMP, &sensor_vals);
        if (err != 0) {
            LOG_ERR("Temperature sensor get(): %d", err);
            return err;
        }
    }
        
        // data returned in kPa
        *temperature_degC = sensor_value_to_float(&sensor_vals);

        LOG_INF("Temperature (deg C): %f", (double)*temperature_degC);

        return 0;
}          