#pragma once

void pid_reset(void);
void pid_update(const float omega_ref[3], const float vit_mes_tps[3], float dt_s);
