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

static struct adc_sequence ecg_seq;
static volatile int ecg_sample_index = 0;

volatile bool led2_enabled = true;


// function declarations

const struct device *const temp_sensor = DEVICE_DT_GET_ONE(jedec_jc_42_4_temp);

float temperature_degC;
volatile float measured_bpm = 0.0f;

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

static const struct gpio_dt_spec measure_button = GPIO_DT_SPEC_GET(DT_ALIAS(hrmeasure), gpios);
static const struct gpio_dt_spec clear_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

static struct gpio_callback measure_button_cb;
void measure_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_event_post(&app_events, MEASURE_DATA);
}
static struct gpio_callback clear_button_cb;
void clear_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    LOG_INF("Clearing LED2");
    led2_enabled = false;
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

void average_hr_led_thread(void *, void *, void *) {
    while (1) {
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
    int ret = adc_read(adc_diff.dev, &ecg_seq);
    if (ret < 0) {
        LOG_ERR("ADC read failed at index %d (%d)", ecg_sample_index, ret);
    } else {
        ecg_sample_index++;
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
        k_sleep(K_MSEC(1));
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
        return;
    }

    gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&average_hr_led, GPIO_OUTPUT_INACTIVE);

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

    // 🛠️ Setup ADC channel here BEFORE first battery check
    if (!device_is_ready(adc_vadc.dev)) {
        LOG_ERR("Battery ADC not ready in INIT");
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    if (adc_channel_setup_dt(&adc_vadc) < 0) {
        LOG_ERR("Failed to setup battery ADC channel in INIT");
        smf_set_state(SMF_CTX(&s_obj), &states[ERROR]);
        return;
    }

    LOG_INF("Initial battery check on boot...");
    check_battery_and_update_pwm();

    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}


static void idle_entry(void *o) {
    LOG_INF("Entering IDLE State");
    k_timer_start(&battery_timer, K_MSEC(BATTERY_MEASURE_INTERVAL_MS), K_MSEC(BATTERY_MEASURE_INTERVAL_MS));
}

static void idle_run(void *o) {
    LOG_INF("WAITING FOR BUTTON");
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
    measured_bpm = bpm;
    led2_enabled = true;

    smf_set_state(SMF_CTX(&s_obj), &states[IDLE]);
}


static const struct smf_state states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL, NULL),
    [MEASURE] = SMF_CREATE_STATE(measure_entry, measure_run, NULL, NULL, NULL),
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
        k_msleep(100);
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