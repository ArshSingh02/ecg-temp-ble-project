#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h> 
#include <zephyr/smf.h> 

#include "read_temperature_sensor.h"
#include "ble-lib.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define MEASUREMENT_DELAY_MS 1000

// function declarations

const struct device *const temp_sensor = DEVICE_DT_GET_ONE(jedec_jc_42_4_temp);

float temperature_degC;

K_EVENT_DEFINE(errors);





// State Framework
enum states { INIT, IDLE, MEASURE, BLUETOOTH,ERROR };

int state = INIT;

static const struct smf_state states[];
struct s_object {
    struct smf_ctx ctx;
} s_obj;


static void init_run(void *o) {

}

int main(void) {

    int ret;
 
    ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);

    if (!device_is_ready(temp_sensor)) {
            LOG_ERR("Temperature sensor %s is not ready", temp_sensor->name);
            return -1;
    }
    else {
        LOG_INF("Temperature sensor %s is ready", temp_sensor->name);
    }


    // read the temperature every MEASUREMENT_DELAY_MS
    while (1) {

        ret = read_temperature_sensor(temp_sensor, &temperature_degC);
        if (ret != 0) {
            LOG_ERR("There was a problem reading the temperature sensor (%d)", ret);
            return ret;
        }

        LOG_INF("Temperature: %f", (double)temperature_degC);

        k_msleep(MEASUREMENT_DELAY_MS);

    }

    return 0;
}