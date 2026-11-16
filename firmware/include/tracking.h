#pragma once

struct TrackingState {
  float start_x = 0, start_y = 0, target_x = 0, target_y = 0, target_yaw = 0;
  float length = 0, progress = 0, reference_speed = 0;
  float integral_x = 0, integral_y = 0, integral_yaw = 0;
  float settled_s = 0, elapsed_s = 0;
};

void tracking_begin(TrackingState& s, float x, float y, float tx, float ty, float yaw);
// World-frame output [vx, vy, wz]. Returns true after position AND speed settle.
bool tracking_step(TrackingState& s, float x, float y, float yaw,
                   float vx, float vy, float wz, float dt, float output[3]);
bool tracking_timed_out(const TrackingState& s);
