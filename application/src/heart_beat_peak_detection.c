#include "heart_beat_peak_detection.h"
#include <stdint.h>
#include <stdlib.h>

float compute_bpm(const int16_t *ecg_buffer, int buffer_size, int sample_rate, int duration_sec) {
    const int16_t threshold = 100;
    int peak_count = 0;

    for (int i = 1; i < buffer_size - 1; i++) {
        if (ecg_buffer[i] > threshold &&
            ecg_buffer[i] > ecg_buffer[i - 1] &&
            ecg_buffer[i] > ecg_buffer[i + 1]) {
            peak_count++;
            i += sample_rate / 4;
        }
    }

    return ((float)peak_count / duration_sec) * 60.0f;
}