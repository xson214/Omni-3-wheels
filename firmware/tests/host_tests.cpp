#include "config.h"
#include "kinematics.h"
#include "motors.h"
#include "pid_speed.h"
#include "tracking.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

static int checks = 0;
static int pwm_output[3] = {};
static int direction_output[3] = {};

static void require(bool condition, const char* message) {
  ++checks;
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

static void near(float actual, float expected, float tolerance, const char* message) {
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
    std::fprintf(stderr, "%s: actual %.9g expected %.9g tolerance %.9g\n",
                 message, actual, expected, tolerance);
    require(false, message);
  }
  ++checks;
}

void motors_init() {}
void set_motor(int index, int pwm, int direction) {
  require(index >= 0 && index < 3, "valid motor index");
  require(pwm >= 0 && pwm <= PWM_MAX, "PWM always bounded");
  require(direction >= -1 && direction <= 1, "motor direction valid");
  pwm_output[index] = pwm;
  direction_output[index] = direction;
}
void motors_stop_all() {
  for (int i = 0; i < 3; ++i) set_motor(i, 0, 0);
}

static void test_kinematics() {
  float vx, vy, wz;
  const float slide14_w[3] = {4, 2, 3};
  fwdkin(slide14_w, 0, &vx, &vy, &wz);
  near(vx, 0.0167431578f, 1e-7f, "slide 14 vx");
  near(vy, 0.029f, 1e-7f, "slide 14 vy");
  near(wz, 0.725f, 1e-6f, "slide 14 yaw rate");

  float x = 0, y = 0, yaw = 0;
  for (int i = 0; i < 5000; ++i) {
    // Midpoint integration with the real forward model.
    float midpoint = yaw + wz * 0.0005f;
    fwdkin(slide14_w, midpoint, &vx, &vy, &wz);
    x += vx * 0.001f; y += vy * 0.001f; yaw += wz * 0.001f;
  }
  near(x, -0.0861507216f, 2e-5f, "slide 15 integrated x");
  near(y, 0.0249498736f, 2e-5f, "slide 15 integrated y");

  float w[3];
  invkin(0.1f, 0.05f, 0.5f, 0, w);
  near(w[0], 3.7931034483f, 1e-5f, "slide 17 wheel 1");
  near(w[1], -1.7793979441f, 1e-5f, "slide 17 wheel 2");
  near(w[2], 4.1931910475f, 1e-5f, "slide 17 wheel 3");

  for (float angle : {-3.13f, -1.2f, 0.0f, 0.7f, 3.13f}) {
    for (const auto& velocity : {std::array<float,3>{0.1f,0,0},
                                {0,-0.1f,0}, {0,0,0.4f}, {-0.08f,0.05f,-0.2f}}) {
      invkin(velocity[0], velocity[1], velocity[2], angle, w);
      fwdkin(w, angle, &vx, &vy, &wz);
      near(vx, velocity[0], 1e-6f, "world inverse/forward vx roundtrip");
      near(vy, velocity[1], 1e-6f, "world inverse/forward vy roundtrip");
      near(wz, velocity[2], 1e-6f, "world inverse/forward yaw roundtrip");
      float body_w[3];
      invkin_body(std::cos(angle)*velocity[0]+std::sin(angle)*velocity[1],
                  -std::sin(angle)*velocity[0]+std::cos(angle)*velocity[1],
                  velocity[2], body_w);
      for (int i=0; i<3; ++i) near(body_w[i], w[i], 2e-6f, "body/world inverse agree");
    }
  }
  float high[3] = {12,-6,3};
  saturate_omega(high);
  near(high[0], OMEGA_MAX_RAD_S, 1e-6f, "wheel limit");
  near(high[1]/high[0], -0.5f, 1e-6f, "saturation preserves negative ratio");
  near(high[2]/high[0], 0.25f, 1e-6f, "saturation preserves positive ratio");
  float low[3] = {0,-1,2};
  saturate_omega(low);
  near(low[1], -1, 1e-7f, "unsaturated speeds unchanged");
  near(wrap_pi(2*PI+0.2f), 0.2f, 1e-6f, "angle wrapping");
  std::puts("PASS kinematics: slide fixtures, frame transforms, roundtrip, limits");
}

static void test_speed_pid() {
  const float target[3] = {0.2f*2*PI,0.2f*2*PI,0.2f*2*PI};
  const float aligned[3] = {0.1f,0.1f,0.1f};
  const float opposed[3] = {-0.1f,-0.1f,-0.1f};
  pid_reset(); pid_update(target, aligned, 0.01f);
  int aligned_pwm = pwm_output[0];
  pid_reset(); pid_update(target, opposed, 0.01f);
  require(pwm_output[0] > aligned_pwm, "wrong-direction encoder increases corrective PWM");
  require(direction_output[0] == 1, "positive setpoint drives positive direction");

  const float negative[3] = {-target[0],-target[1],-target[2]};
  const float stopped[3] = {};
  for (int i=0; i<500; ++i) pid_update(target, stopped, 0.01f);
  pid_update(negative, stopped, 0.01f);
  int reversal_pwm = pwm_output[0];
  require(direction_output[0] == -1, "reversal changes motor direction");
  pid_reset(); pid_update(negative, stopped, 0.01f);
  require(pwm_output[0] == reversal_pwm, "reversal clears old integral and derivative");

  for (int i=0; i<500; ++i) pid_update(target, stopped, 0.01f);
  pid_update(stopped, stopped, 0.01f);
  for (int i=0; i<3; ++i) require(pwm_output[i] == 0 && direction_output[i] == 0, "zero target stops motor");
  pid_update(target, stopped, 0.01f);
  int after_stop = pwm_output[0];
  pid_reset(); pid_update(target, stopped, 0.01f);
  require(pwm_output[0] == after_stop, "stop clears PID memory");

  const float saturated[3] = {10*2*PI,10*2*PI,10*2*PI};
  pid_reset();
  for (int i=0; i<10000; ++i) pid_update(saturated, stopped, 0.01f);
  require(pwm_output[0] == PWM_MAX, "large error saturates");
  pid_update(target, stopped, 0.01f);
  int after_saturation = pwm_output[0];
  pid_reset(); pid_update(target, stopped, 0.01f);
  require(pwm_output[0] == after_saturation, "saturated error does not accumulate integral");

  for (float invalid_dt : {0.0f,-0.01f,1.0f,std::numeric_limits<float>::quiet_NaN()}) {
    pid_update(target, stopped, 0.01f);
    pid_update(target, stopped, invalid_dt);
    for (int i=0; i<3; ++i) require(pwm_output[i] == 0, "invalid sample time stops every motor");
  }
  const float bad_measurement[3] = {std::numeric_limits<float>::quiet_NaN(),0,0};
  pid_update(target, bad_measurement, 0.01f);
  require(pwm_output[0] == 0, "invalid wheel measurement stops affected motor");

  // Closed-loop plant deliberately differs from the feedforward estimate:
  // deadzone, lower maximum speed and motor inertia must be corrected by PID.
  pid_reset();
  float plant_tps = 0;
  for (float desired_tps : {0.35f,-0.25f,0.1f,0.0f}) {
    float plant_target[3] = {desired_tps*2*PI,desired_tps*2*PI,desired_tps*2*PI};
    for (int i=0; i<2000; ++i) {
      float measurement[3] = {plant_tps,plant_tps,plant_tps};
      pid_update(plant_target,measurement,0.01f);
      float equilibrium = direction_output[0] * std::fmax(0.0f,pwm_output[0]-40.0f)/215.0f*0.65f;
      plant_tps += (1-std::exp(-0.01f/0.12f))*(equilibrium-plant_tps);
    }
    near(plant_tps,desired_tps,0.01f,"speed PID converges with deadzone and model mismatch");
  }
  std::puts("PASS speed PID: signed feedback, reset, reversal, antiwindup, invalid samples");
}

struct Plant { float x=0, y=0, yaw=0, vx=0, vy=0, wz=0; };

static void simulate_segment(Plant& p, float tx, float ty, float target_yaw, float tau, bool disturb) {
  TrackingState state;
  tracking_begin(state, p.x, p.y, tx, ty, target_yaw);
  bool done = false, injected = false;
  for (int i=0; i<20000; ++i) {
    float out[3];
    done = tracking_step(state,p.x,p.y,p.yaw,p.vx,p.vy,p.wz,0.01f,out);
    for (float value : out) require(std::isfinite(value), "tracking output finite");
    require(std::hypot(out[0],out[1]) <= V_BODY_MAX_M_S+1e-6f, "tracking translation limited");
    require(std::fabs(out[2]) <= YAW_RATE_MAX_RAD_S+1e-6f, "tracking yaw rate limited");
    float wheel[3]; invkin(out[0],out[1],out[2],p.yaw,wheel);
    for (float value : wheel) require(std::fabs(value) <= OMEGA_MAX_RAD_S+1e-5f, "tracking respects all wheel limits");
    if (done) break;
    require(!tracking_timed_out(state), "moving plant converges before timeout");
    float factor = tau > 0 ? 1-std::exp(-0.01f/tau) : 1;
    p.vx += factor*(out[0]-p.vx); p.vy += factor*(out[1]-p.vy); p.wz += factor*(out[2]-p.wz);
    p.x += p.vx*0.01f; p.y += p.vy*0.01f; p.yaw = wrap_pi(p.yaw+p.wz*0.01f);
    if (disturb && !injected && state.progress > 0.5f*state.length) {
      p.y += 0.03f; injected = true;
    }
  }
  require(done, "segment finishes");
  require(std::hypot(p.x-tx,p.y-ty) <= POSE_XY_TOL_M, "final position inside tolerance");
  require(std::fabs(wrap_pi(p.yaw-target_yaw)) <= POSE_YAW_TOL_RAD, "final heading inside tolerance");
  require(std::hypot(p.vx,p.vy) <= SETTLE_SPEED_M_S, "finished plant translation settled");
  require(std::fabs(p.wz) <= SETTLE_YAW_RAD_S, "finished plant yaw settled");
}

static void test_tracking() {
  Plant ideal;
  simulate_segment(ideal,0.3f,0,0,0,false);
  Plant lagged;
  simulate_segment(lagged,-0.3f,-0.2f,0,0.18f,true);
  simulate_segment(lagged,0.2f,-0.2f,0,0.18f,false);
  simulate_segment(lagged,0.2f,0.2f,0,0.18f,false);
  simulate_segment(lagged,0,0,0,0.18f,false);
  Plant rotate;
  rotate.yaw=PI-0.02f;
  simulate_segment(rotate,0,0,-PI+0.02f,0.15f,false);
  simulate_segment(rotate,0.25f,0.2f,1.0f,0.15f,false);

  TrackingState wrap_state;
  tracking_begin(wrap_state,0,0,0,0,-PI+0.05f);
  float out[3];
  tracking_step(wrap_state,0,0,PI-0.05f,0,0,0,0.01f,out);
  require(out[2] > 0 && out[2] < 0.3f, "yaw controller uses shortest wrapped error");

  TrackingState settling;
  tracking_begin(settling,0,0,0,0,0);
  for (int i=0; i<200; ++i) require(!tracking_step(settling,0,0,0,0.05f,0,0,0.01f,out), "position alone cannot finish while moving");
  for (int i=0; i<10; ++i) require(!tracking_step(settling,0,0,0,0,0,0,0.01f,out), "requires sustained settling");
  require(!tracking_step(settling,0,0,0,0,0,0.1f,0.01f,out), "yaw movement resets settling");
  bool done=false;
  for (int i=0; i<50; ++i) done = tracking_step(settling,0,0,0,0,0,0,0.01f,out);
  require(done, "stationary endpoint completes after settling");

  TrackingState blocked;
  tracking_begin(blocked,0,0,0.2f,0,0);
  bool expired=false;
  for (int i=0; i<6000; ++i) {
    require(!tracking_step(blocked,0,0,0,0,0,0,0.01f,out), "immobile robot never reports destination reached");
    if (tracking_timed_out(blocked)) { expired=true; break; }
  }
  require(expired, "immobile robot reaches finite timeout");
  tracking_begin(blocked,1,2,1.1f,2.2f,0.5f);
  near(blocked.elapsed_s,0,1e-7f,"new segment clears timeout");
  near(blocked.progress,0,1e-7f,"new segment clears progress");
  std::puts("PASS tracking: ideal/lagged plants, disturbances, segments, wrap, settling, timeout");
}

int main() {
  test_kinematics();
  test_speed_pid();
  test_tracking();
  std::printf("All host tests passed (%d checks). No physical robot was commanded.\n",checks);
}
