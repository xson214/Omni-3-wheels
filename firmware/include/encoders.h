#pragma once

void encoders_init(void);
void encoders_reset(void);
void encoders_sample(float vit_tps[3], float omega_rad_s[3], const float consigne_tps[3]);
