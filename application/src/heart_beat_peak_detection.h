#ifndef HEART_BEAT_PEAK_DETECTION_H
#define HEART_BEAT_PEAK_DETECTION_H

// Function prototype goes here
#include <stdint.h>
#include <stddef.h>

float compute_bpm(const int16_t *ecg_buffer, int buffer_size, int sample_rate, int duration_sec);

#endif