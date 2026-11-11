#pragma once

#include <stdint.h>

void encoders_init(void);
void encoders_sample(float vit_tps[3], float omega_rad_s[3], float dt_s);
void encoders_get_raw_counts(int64_t counts[3]);
