#include "kinematics.h"
#include "config.h"

#include <math.h>

float wrap_pi(float a) {
  a = fmodf(a, 2.0f * PI);
  if (a > PI) a -= 2.0f * PI;
  if (a < -PI) a += 2.0f * PI;
  return a;
}

void invkin(float xd, float yd, float phid, float phi, float w[3]) {
  const float alpha[3] = {ALPHA1_RAD, ALPHA2_RAD, ALPHA3_RAD};
  for (int i = 0; i < 3; i++) {
    float a = phi + alpha[i];
    float vi = -sinf(a) * xd + cosf(a) * yd + BASE_RADIUS_M * phid;
    w[i] = vi / WHEEL_RADIUS_M;
  }
}

void invkin_body(float vx_b, float vy_b, float wz, float w[3]) {
  // Jb = [[0, 1, L], [-√3/2, -1/2, L], [√3/2, -1/2, L]]
  const float s3 = 0.86602540378f;  // sqrt(3)/2
  float v[3];
  v[0] = vy_b + BASE_RADIUS_M * wz;
  v[1] = -s3 * vx_b - 0.5f * vy_b + BASE_RADIUS_M * wz;
  v[2] = s3 * vx_b - 0.5f * vy_b + BASE_RADIUS_M * wz;
  for (int i = 0; i < 3; i++) {
    w[i] = v[i] / WHEEL_RADIUS_M;
  }
}

void fwdkin(const float w[3], float phi, float* xd, float* yd, float* phid) {
  const float alpha[3] = {ALPHA1_RAD, ALPHA2_RAD, ALPHA3_RAD};
  float sx = 0.0f;
  float sy = 0.0f;
  float sw = 0.0f;
  for (int i = 0; i < 3; i++) {
    float a = phi + alpha[i];
    sx += -sinf(a) * w[i];
    sy += cosf(a) * w[i];
    sw += w[i];
  }
  *xd = (2.0f * WHEEL_RADIUS_M / 3.0f) * sx;
  *yd = (2.0f * WHEEL_RADIUS_M / 3.0f) * sy;
  *phid = (WHEEL_RADIUS_M / (3.0f * BASE_RADIUS_M)) * sw;
}

void saturate_omega(float w[3]) {
  float m = fmaxf(fmaxf(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (m > OMEGA_MAX_RAD_S) {
    float s = OMEGA_MAX_RAD_S / m;
    w[0] *= s;
    w[1] *= s;
    w[2] *= s;
  }
}
