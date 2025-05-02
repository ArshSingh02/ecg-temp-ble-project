#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h> 
#include <zephyr/smf.h> 

#include "read_temperature_sensor.h"
#include "heart_beat_peak_detection.h"
#include "ble-lib.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define MEASUREMENT_DELAY_MS 1000
#define BATTERY_MEASURE_INTERVAL_MS 60000
#define PWM_PERIOD_USEC 1000

#define ECG_SAMPLE_RATE_HZ 1000
#define ECG_DURATION_SEC 30
#define ECG_BUFFER_SIZE (ECG_SAMPLE_RATE_HZ * ECG_DURATION_SEC)


// function declarations

const struct device *const temp_sensor = DEVICE_DT_GET_ONE(jedec_jc_42_4_temp);

float temperature_degC;

K_EVENT_DEFINE(errors);

// Define button events
K_EVENT_DEFINE(app_events);
// Represent Button with Mask
#define MEASURE_DATA BIT(0)
#define CLEAR_LED BIT(1)
#define RESET_DEVICE BIT(2)
#define BATTERY_TIMER_EVENT BIT(3)

void battery_timer_handler(struct k_timer *timer_id) {
    k_event_post(&app_events, BATTERY_TIMER_EVENT);
}
K_TIMER_DEFINE(battery_timer, battery_timer_handler, NULL);


// Define ADC Channel Configuration
#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                  \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),    \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),          \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))        \
}

static const struct adc_dt_spec adc_vadc = ADC_DT_SPEC_GET_BY_ALIAS(vadc);
static int16_t adc_buf;

static const struct adc_dt_spec adc_diff = ADC_DT_SPEC_GET_BY_ALIAS(diffadc);
static int16_t ecg_buffer[ECG_BUFFER_SIZE];

static const struct pwm_dt_spec pwm1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm1));

// Configure LEDs and Buttons
static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
// static const struct gpio_dt_spec battery_led = GPIO_DT_SPEC_GET(DT_ALIAS(batterylevel), gpios);
static const struct gpio_dt_spec average_hr_led = GPIO_DT_SPEC_GET(DT_ALIAS(avgheartrate), gpios);

static const struct gpio_dt_spec measure_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec clear_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

static struct gpio_callback measure_button_cb;
void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_event_post(&app_events, MEASURE_DATA);
}
static struct gpio_callback clear_button_cb;
void clear_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_event_post(&app_events, CLEAR_LED);
}
static struct gpio_callback reset_button_cb;
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_event_post(&app_events, RESET_DEVICE);
}

// Initialize Threads
void heartbeat_thread(void *, void *, void *) {
    while (1) {
        gpio_pin_toggle_dt(&heartbeat_led);
        k_msleep(500);
        gpio_pin_toggle_dt(&heartbeat_led);
        k_msleep(500);
    } 
}
K_THREAD_DEFINE(heartbeat_thread_id, 1024, heartbeat_thread, NULL, NULL, NULL, 5, 0, 0);

// Helper Functions
float check_battery_and_update_pwm(void) {
    struct adc_sequence seq = {
        .buffer = &adc_buf,
        .buffer_size = sizeof(adc_buf),
        .resolution = 12,
        .channels = BIT(adc_vadc.channel_id),
    };
    adc_sequence_init_dt(&adc_vadc, &seq);

    if (adc_read(adc_vadc.dev, &seq) == 0) {
        int32_t val_mv = adc_buf;
        if (adc_raw_to_millivolts_dt(&adc_vadc, &val_mv) == 0) {
            float battery_pct = CLAMP(val_mv / 3000.0f, 0.0f, 1.0f);
            uint32_t pulse_width = (uint32_t)(battery_pct * PWM_PERIOD_USEC);

            int ret = pwm_set_pulse_dt(&pwm1, pulse_width * 1000);
            if (ret < 0) {
                LOG_ERR("PWM set failed: %d", ret);
            }

            LOG_INF("Battery level: %d mV → %.1f%% brightness", val_mv, battery_pct * 100);
            return battery_pct;
        } else {
            LOG_ERR("ADC to millivolts conversion failed");
        }
    } else {
        LOG_ERR("ADC read failed");
    }
    return -1.0f;
}


float measure_average_heart_rate(void) {

    struct adc_sequence seq = {
        .buffer = &ecg_buffer[0],
        .buffer_size = sizeof(int16_t),
        .resolution = 12,
        .oversampling = 4,
        .channels = BIT(adc_diff.channel_id),
    };
    adc_sequence_init_dt(&adc_diff, &seq);

    for (int i = 0; i < ECG_BUFFER_SIZE; i++) {
        seq.buffer = &ecg_buffer[i];
        int ret = adc_read(adc_diff.dev, &seq);
        if (ret < 0) {
            LOG_ERR("ADC read failed at index %d (%d)", i, ret);
            return -1.0f;
        }
        k_busy_wait(1000);
    }

    LOG_HEXDUMP_INF(ecg_buffer, sizeof(ecg_buffer), "ECG Buffer (HEX)");

    float bpm = compute_bpm(ecg_buffer, ECG_BUFFER_SIZE, ECG_SAMPLE_RATE_HZ, ECG_DURATION_SEC);
    return bpm;
}




// State Framework
enum states { INIT, IDLE, MEASURE, BATTERY, BLUETOOTH, ERROR };

int state = INIT;

static const struct smf_state states[];
struct s_object {
    struct smf_ctx ctx;
} s_obj;


static void init_run(void *o) {
    if (!device_is_ready(heartbeat_led.port) ||
    !device_is_ready(average_hr_led.port) ||
    !device_is_ready(measure_button.port) ||
    !device_is_ready(clear_button.port) ||
    !device_is_ready(reset_button.port)) {
    LOG_ERR("GPIO0 device not ready.");
    smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
    }

    gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
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

    LOG_INF("Initial battery check on boot...");
    check_battery_and_update_pwm();

    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}

static void idle_entry(void *o) {
    LOG_INF("Entering IDLE State");
    k_timer_start(&battery_timer, K_NO_WAIT, K_MSEC(BATTERY_MEASURE_INTERVAL_MS));
}

static void idle_run(void *o) {
    uint32_t events = k_event_wait(&app_events, MEASURE_DATA | BATTERY_TIMER_EVENT, true, K_FOREVER);
    if (events & MEASURE_DATA) {
        smf_set_state(SMF_CTX(&s_obj), &states[MEASURE]);
    } else if (events & BATTERY_TIMER_EVENT) {
        smf_set_state(SMF_CTX(&s_obj), &states[BATTERY]);
    }
}

static void battery_entry(void *o) {
    LOG_INF("BATTERY ENTRY: Setting up ADC");

    if (!device_is_ready(adc_vadc.dev)) {
        LOG_ERR("Battery ADC not ready");
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    if (adc_channel_setup_dt(&adc_vadc) < 0) {
        LOG_ERR("Failed to setup battery ADC channel");
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }
}

static void battery_run(void *o) {
    check_battery_and_update_pwm();
    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}

static void measure_entry(void *o) {
    LOG_INF("Measure ENTRY");
    if (!device_is_ready(adc_diff.dev)) {
        LOG_ERR("ADC not ready");
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }
}

static void measure_run(void *o) {
    float bpm = measure_average_heart_rate();
    if (bpm < 0) {
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    LOG_INF("Computed Average Heart Rate: %.1f BPM", bpm);

    gpio_pin_set_dt(&average_hr_led, 1);
    k_sleep(K_SECONDS(1));
    gpio_pin_set_dt(&average_hr_led, 0);

    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}


static const struct smf_state states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL, NULL),
    [MEASURE] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [BATTERY] = SMF_CREATE_STATE(battery_entry, battery_run, NULL, NULL, NULL),
    [BLUETOOTH] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [ERROR] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
};

int main(void) {

    /* int ret;
 
    ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);

    if (!device_is_ready(temp_sensor)) {
            LOG_ERR("Temperature sensor %s is not ready", temp_sensor->name);
            return -1;
    }
    else {
        LOG_INF("Temperature sensor %s is ready", temp_sensor->name);
    }
    */
   
    smf_set_initial(SMF_CTX(&s_obj), &states[INIT]);
    // read the temperature every MEASUREMENT_DELAY_MS
    while (1) {

        smf_run_state(SMF_CTX(&s_obj));
        k_timer_start(&battery_timer, K_NO_WAIT, K_MSEC(BATTERY_MEASURE_INTERVAL_MS));

        /* ret = read_temperature_sensor(temp_sensor, &temperature_degC);
        if (ret != 0) {
            LOG_ERR("There was a problem reading the temperature sensor (%d)", ret);
            return ret;
        }

        LOG_INF("Temperature: %f", (double)temperature_degC);

        k_msleep(MEASUREMENT_DELAY_MS);
        */
    }

    return 0;
}