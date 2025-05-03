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

#define ERROR_LED_ON_TIME_MS 500
#define ERROR_LED_PERIOD_USEC (ERROR_LED_ON_TIME_MS * 1000)

static struct adc_sequence ecg_seq;
static volatile int ecg_sample_index = 0;

volatile bool led2_enabled = true;
volatile bool error_state_flag = false;
static bool bluetooth_initialized = false;

float battery_pct = 0.0f;

static struct k_poll_signal adc_signal;

extern enum bt_data_notifications_enabled notifications_enabled;
extern struct bt_gatt_service remote_srv;


static enum adc_action ecg_adc_callback(const struct device *dev,
    const struct adc_sequence *sequence,
    void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(sequence);
    ARG_UNUSED(user_data);

    ecg_sample_index++;
    return ADC_ACTION_FINISH;
}

static struct adc_sequence_options ecg_seq_opts = {
    .callback = ecg_adc_callback,
    .user_data = NULL,
    .extra_samplings = 0,
};

// function declarations

const struct device *const temp_sensor = DEVICE_DT_GET_ONE(jedec_jc_42_4_temp);

float temperature_degC;
volatile float measured_bpm = 0.0f;

// Define Device Events
K_EVENT_DEFINE(app_events);
// Represent Button with Mask
#define MEASURE_DATA BIT(0)
#define CLEAR_LED BIT(1)
#define RESET_DEVICE BIT(2)
#define BATTERY_TIMER_EVENT BIT(3)

// Define Error Events
K_EVENT_DEFINE(errors);
// Represent Errors with Mask
#define MEASURE_ERROR BIT(0)
#define LED_BUTTON_ERROR BIT(1)
#define ADC_ERROR BIT(2)
#define TEMP_SENSOR_ERROR BIT(3)
#define BLE_ERROR BIT(4)


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
static const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_ALIAS(erroronly), gpios);

static const struct gpio_dt_spec measure_button = GPIO_DT_SPEC_GET(DT_ALIAS(hrmeasure), gpios);
static const struct gpio_dt_spec clear_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

static struct gpio_callback measure_button_cb;
void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_event_post(&app_events, MEASURE_DATA);
}

static struct gpio_callback clear_button_cb;
void clear_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (error_state_flag) {
        return;
    }

    if (led2_enabled) {
        LOG_INF("Clearing LED2");
        led2_enabled = false;
        k_event_post(&app_events, CLEAR_LED);
    } else {
        LOG_WRN("LED2 is already off. No heart rate measurement taken yet.");
    }
}


static struct gpio_callback reset_button_cb;
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    LOG_INF("Reset Button Pressed");
    k_event_post(&app_events, RESET_DEVICE);
}

// State Framework
enum states { INIT, IDLE, MEASURE, BATTERY, ERROR };

int state = INIT;

static const struct smf_state states[];
struct s_object {
    struct smf_ctx ctx;
} s_obj;

// Initialize Threads
void heartbeat_thread(void *, void *, void *) {
    while (1) {
        if (!error_state_flag) {
            gpio_pin_set_dt(&heartbeat_led, 1);
            k_msleep(500);
            gpio_pin_set_dt(&heartbeat_led, 0);
            k_msleep(500);
        } else {
            k_msleep(100);
        }
    }
}
K_THREAD_DEFINE(heartbeat_thread_id, 1024, heartbeat_thread, NULL, NULL, NULL, 5, 0, 0);

void average_hr_led_thread(void *, void *, void *) {
    while (1) {
        if (error_state_flag) {
            k_msleep(100);
            continue;
        }

        if (!led2_enabled) {
            gpio_pin_set_dt(&average_hr_led, 0);
            k_msleep(200);
            continue;
        }

        float bpm = measured_bpm;
        if (bpm >= 40.0f && bpm <= 200.0f) {
            float hz = bpm / 60.0f;
            int period_ms = (int)(1000.0f / hz);
            int on_time = period_ms / 4;
            int off_time = period_ms - on_time;

            gpio_pin_set_dt(&average_hr_led, 1);
            k_msleep(on_time);
            gpio_pin_set_dt(&average_hr_led, 0);
            k_msleep(off_time);
        } else {
            gpio_pin_set_dt(&average_hr_led, 0);
            k_msleep(200);
        }
    }
}


K_THREAD_DEFINE(average_hr_led_thread_id, 1024, average_hr_led_thread,
                NULL, NULL, NULL, 5, 0, 0);

void error_leds_thread(void *, void *, void *) {
    while (1) {
        if (error_state_flag) {
            gpio_pin_set_dt(&heartbeat_led, 1);
            pwm_set_dt(&pwm1, ERROR_LED_PERIOD_USEC, ERROR_LED_PERIOD_USEC);
            gpio_pin_set_dt(&average_hr_led, 1);
            gpio_pin_set_dt(&error_led, 1);
            k_msleep(ERROR_LED_ON_TIME_MS);
            gpio_pin_set_dt(&heartbeat_led, 0);
            pwm_set_pulse_dt(&pwm1, 0);
            gpio_pin_set_dt(&average_hr_led, 0);
            gpio_pin_set_dt(&error_led, 0);
            k_msleep(ERROR_LED_ON_TIME_MS);
        } else {
            k_msleep(100);
        }
    }
}

K_THREAD_DEFINE(error_leds_thread_id, 1024, error_leds_thread,
                NULL, NULL, NULL, 5, 0, 0);

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
            battery_pct = val_mv / 3000.0f;
            uint32_t pulse_width = (uint32_t)(battery_pct * PWM_PERIOD_USEC);

            int ret = pwm_set_pulse_dt(&pwm1, pulse_width * 1000);
            if (ret < 0) {
                LOG_ERR("PWM set failed: %d", ret);
            }

            LOG_INF("Battery level: %d mV → %.1f%% brightness", val_mv, battery_pct * 100);
            return (float)val_mv;
        } else {
            LOG_ERR("ADC to millivolts conversion failed");
        }
    } else {
        LOG_ERR("ADC read failed");
    }
    return -1.0f;
}


void ecg_sample_work_handler(struct k_work *work);
void ecg_sample_timer_handler(struct k_timer *timer);

K_WORK_DEFINE(ecg_sample_work, ecg_sample_work_handler);
K_TIMER_DEFINE(ecg_sample_timer, ecg_sample_timer_handler, NULL);

void ecg_sample_work_handler(struct k_work *work) {
    if (ecg_sample_index >= ECG_BUFFER_SIZE) {
        k_timer_stop(&ecg_sample_timer);
        return;
    }

    ecg_seq.buffer = &ecg_buffer[ecg_sample_index];
    ecg_seq.buffer_size = sizeof(int16_t);
    ecg_seq.options = &ecg_seq_opts;  // <--- CRITICAL FIX

    int err = adc_read_async(adc_diff.dev, &ecg_seq, &adc_signal);
    if (err < 0) {
        LOG_ERR("ADC async read failed at index %d (%d)", ecg_sample_index, err);  
        k_timer_stop(&ecg_sample_timer);
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
    }
}


void ecg_sample_timer_handler(struct k_timer *timer) {
    k_work_submit(&ecg_sample_work);
}

float measure_average_heart_rate(void) {
    if (!device_is_ready(adc_diff.dev)) {
        LOG_ERR("Differential ADC not ready");
        return -1.0f;
    }

    if (adc_channel_setup_dt(&adc_diff) < 0) {
        LOG_ERR("Failed to setup ADC diff channel");
        return -1.0f;
    }

    ecg_sample_index = 0;

    ecg_seq = (struct adc_sequence){
        .buffer = &ecg_buffer[0],
        .buffer_size = sizeof(int16_t),
        .resolution = 12,
        .oversampling = 4,
        .channels = BIT(adc_diff.channel_id),
    };
    adc_sequence_init_dt(&adc_diff, &ecg_seq);

    k_timer_start(&ecg_sample_timer, K_NO_WAIT, K_MSEC(1));

    while (ecg_sample_index < ECG_BUFFER_SIZE) {
        uint32_t events = k_event_wait(&app_events, MEASURE_DATA, false, K_NO_WAIT);
        if (events & MEASURE_DATA) {
            LOG_ERR("ERROR: Measurement button pressed again during ECG sampling.");
            k_event_post(&errors, MEASURE_ERROR);
            k_timer_stop(&ecg_sample_timer);
            return -1.0f;
        }
        k_sleep(K_MSEC(1));
    }

    LOG_HEXDUMP_INF(ecg_buffer, sizeof(ecg_buffer), "ECG Buffer (HEX)");
    float bpm = compute_bpm(ecg_buffer, ECG_BUFFER_SIZE, ECG_SAMPLE_RATE_HZ, ECG_DURATION_SEC);
    return bpm;
}

float read_temperature(void) {

    float temp;
    int ret = read_temperature_sensor(temp_sensor, &temperature_degC);
        if (ret != 0) {
            LOG_ERR("There was a problem reading the temperature sensor (%d)", ret);
            return ret;
        }
    return temperature_degC;
}


// State Framework Functions

static void init_run(void *o) {
    if (!device_is_ready(heartbeat_led.port) ||
        !device_is_ready(average_hr_led.port) ||
        !device_is_ready(error_led.port) ||
        !device_is_ready(measure_button.port) ||
        !device_is_ready(clear_button.port) ||
        !device_is_ready(reset_button.port)) {
        LOG_ERR("GPIO0 device not ready.");
        k_event_post(&errors, LED_BUTTON_ERROR);
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);

        return;
    }

    gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&average_hr_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&measure_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&measure_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_configure_dt(&clear_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&clear_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&measure_button_cb, measure_button_callback, BIT(measure_button.pin));
    gpio_add_callback_dt(&measure_button, &measure_button_cb);

    gpio_init_callback(&clear_button_cb, clear_button_callback, BIT(clear_button.pin));
    gpio_add_callback_dt(&clear_button, &clear_button_cb);

    gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin));
    gpio_add_callback_dt(&reset_button, &reset_button_cb);

    if (!device_is_ready(adc_vadc.dev)) {
        LOG_ERR("Battery ADC not ready in INIT");
        k_event_post(&errors, ADC_ERROR);
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    if (adc_channel_setup_dt(&adc_vadc) < 0) {
        LOG_ERR("Failed to setup battery ADC channel in INIT");
        k_event_post(&errors, ADC_ERROR);
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    if (!bluetooth_initialized) {
        int ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
        if (ret < 0) {
            LOG_ERR("Bluetooth init failed (%d)", ret);
            k_event_post(&errors, BLE_ERROR);
            smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
            return;
        }
        bluetooth_initialized = true;
    }

    LOG_INF("Initial battery check on boot...");

    uint16_t battery_mv = (uint16_t)check_battery_and_update_pwm();
    bluetooth_set_battery_level(battery_mv);

    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}


static void idle_entry(void *o) {
    LOG_INF("Entering IDLE State");
    k_timer_start(&battery_timer, K_MSEC(BATTERY_MEASURE_INTERVAL_MS), K_MSEC(BATTERY_MEASURE_INTERVAL_MS));
}

static void idle_run(void *o) {
    LOG_INF("WAITING FOR BUTTON");

    uint32_t error_flags = k_event_wait(&errors,
        MEASURE_ERROR | LED_BUTTON_ERROR | ADC_ERROR | TEMP_SENSOR_ERROR | BLE_ERROR,
        false, K_NO_WAIT);
    
    if (error_flags) {
        LOG_ERR("Error detected: 0x%X", error_flags);
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

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
        k_event_post(&errors, ADC_ERROR);
        return;
    }

    if (adc_channel_setup_dt(&adc_vadc) < 0) {
        LOG_ERR("Failed to setup battery ADC channel");
        k_event_post(&errors, ADC_ERROR);
        return;
    }
}

static void battery_run(void *o) {
    LOG_INF("BATTERY RUN: Checking battery level");

    uint32_t error_flags = k_event_wait(&errors,
        MEASURE_ERROR | LED_BUTTON_ERROR | ADC_ERROR | TEMP_SENSOR_ERROR | BLE_ERROR,
        false, K_NO_WAIT);
    
    if (error_flags) {
        LOG_ERR("Error detected: 0x%X", error_flags);
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    uint16_t battery_mv = (uint16_t)check_battery_and_update_pwm();
    bluetooth_set_battery_level(battery_mv);
    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}

static void measure_entry(void *o) {
    LOG_INF("Measure ENTRY");
    (void)k_event_wait(&app_events, MEASURE_DATA, true, K_NO_WAIT);

    if (!device_is_ready(adc_diff.dev)) {
        LOG_ERR("ADC not ready");
        k_event_post(&errors, ADC_ERROR);
        return;
    }
    if (!device_is_ready(temp_sensor)) {
        LOG_ERR("Temperature sensor %s is not ready", temp_sensor->name);
        k_event_post(&errors, TEMP_SENSOR_ERROR);
        return;
    }
    else {
        LOG_INF("Temperature sensor %s is ready", temp_sensor->name);
    }
}

static void measure_run(void *o) {
    LOG_INF("Measure RUN: Starting ECG measurement");

    uint32_t error_flags = k_event_wait(&errors,
        MEASURE_ERROR | LED_BUTTON_ERROR | ADC_ERROR | TEMP_SENSOR_ERROR | BLE_ERROR,
        false, K_NO_WAIT);
    
    if (error_flags) {
        LOG_ERR("Error detected: 0x%X", error_flags);
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    float bpm = measure_average_heart_rate();
    float temp = read_temperature();
    if (bpm < 0) {
        LOG_ERR("Error measuring heart rate");
        k_event_post(&errors, MEASURE_ERROR);
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    LOG_INF("Computed Average Heart Rate: %.1f BPM", bpm);

    measured_bpm = bpm;
    uint16_t bluetooth_bpm = (int)bpm;

    // Send hr and temperature data to Bluetooth
    bluetooth_set_heart_rate(bluetooth_bpm);
    if (notifications_enabled == BT_DATA_NOTIFICATIONS_ENABLED) {
        bt_gatt_notify(NULL, &remote_srv.attrs[1], &temperature_degC, sizeof(temperature_degC));
    }

    led2_enabled = true;

    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}

static void error_entry(void *o) {
    LOG_ERR("Entering ERROR state");

    k_timer_stop(&battery_timer);
    k_timer_stop(&ecg_sample_timer);


    error_state_flag = true;
    k_event_clear(&app_events, MEASURE_DATA | BATTERY_TIMER_EVENT);

    if (notifications_enabled == BT_DATA_NOTIFICATIONS_ENABLED) {
        bt_gatt_notify(NULL, &remote_srv.attrs[3], &errors.events, sizeof(errors.events));
    }
}


static void error_run(void *o) {
    LOG_INF("In ERROR state... waiting for reset");

    uint32_t events = k_event_wait(&app_events, RESET_DEVICE, true, K_FOREVER);

    if (events & RESET_DEVICE) {
        LOG_INF("Reset event received. Clearing error and reinitializing.");
        error_state_flag = false;

        k_event_clear(&errors, MEASURE_ERROR | ADC_ERROR | TEMP_SENSOR_ERROR | BLE_ERROR);
        
        smf_set_state(SMF_CTX(&s_obj), &states[INIT]);
    }
}


static const struct smf_state states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL, NULL),
    [MEASURE] = SMF_CREATE_STATE(measure_entry, measure_run, NULL, NULL, NULL),
    [BATTERY] = SMF_CREATE_STATE(battery_entry, battery_run, NULL, NULL, NULL),
    [ERROR] = SMF_CREATE_STATE(error_entry, error_run, NULL, NULL, NULL),
};

int main(void) {

    k_poll_signal_init(&adc_signal);
   
    smf_set_initial(SMF_CTX(&s_obj), &states[INIT]);
    // read the temperature every MEASUREMENT_DELAY_MS
    while (1) {

        smf_run_state(SMF_CTX(&s_obj));
        k_msleep(100);
    }

    return 0;
}