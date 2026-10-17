#pragma once

void invkin(float xd, float yd, float phid, float phi, float w[3]);
void invkin_body(float vx_b, float vy_b, float wz, float w[3]);
void fwdkin(const float w[3], float phi, float* xd, float* yd, float* phid);
void saturate_omega(float w[3]);
float wrap_pi(float a);
