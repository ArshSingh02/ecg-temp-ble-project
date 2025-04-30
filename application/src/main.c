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

// Define button events
K_EVENT_DEFINE(button_events);
// Represent Button with Mask
#define MEASURE_DATA BIT(0)
#define CLEAR_LED BIT(1)
#define RESET_DEVICE BIT(2)

// Configure LEDs and Buttons
static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
static const struct gpio_dt_spec battery_led = GPIO_DT_SPEC_GET(DT_ALIAS(battery), gpios);
static const struct gpio_dt_spec average_hr_led = GPIO_DT_SPEC_GET(DT_ALIAS(avgheartrate), gpios);

static const struct gpio_dt_spec measure_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec clear_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

static struct gpio_callback measure_button_cb;
void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_event_post(&button_events, MEASURE_DATA);
}
static struct gpio_callback clear_button_cb;
void clear_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_event_post(&button_events, CLEAR_LED);
}
static struct gpio_callback reset_button_cb;
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_event_post(&button_events, RESET_DEVICE);
}


// State Framework
enum states { INIT, IDLE, MEASURE, BLUETOOTH,ERROR };

int state = INIT;

static const struct smf_state states[];
struct s_object {
    struct smf_ctx ctx;
} s_obj;


static void init_run(void *o) {
    if (!device_is_ready(heartbeat_led.port) ||
    !device_is_ready(battery_led.port) ||
    !device_is_ready(average_hr_led.port) ||
    !device_is_ready(measure_button.port) ||
    !device_is_ready(clear_button.port) ||
    !device_is_ready(reset_button.port)) {
    LOG_ERR("GPIO0 device not ready.");
    smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
    }

    gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&battery_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&average_hr_led, GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&measure_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&measure_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_configure_dt(&clear_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&clear_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&measure_button_cb, measure_button_callback, BIT(measure_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&measure_button, &measure_button_cb);

    gpio_init_callback(&clear_button_cb, clear_button_callback, BIT(clear_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&clear_button, &clear_button_cb);

    gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&reset_button, &reset_button_cb);

    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}

static const struct smf_state states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [MEASURE] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [BLUETOOTH] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [ERROR] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
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