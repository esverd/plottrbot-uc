#pragma once

#include <stdint.h>
#include <math.h>
#include <limits.h>

namespace plottrbot {

// Drawing coordinates are millimetres from the top-left canvas origin: +X is
// right and +Y is down.  Step positions are absolute belt-length steps.
struct MachineConfig {
  float anchor_span_mm;
  float home_x_mm;
  float home_y_mm;
  float canvas_width_mm;
  float canvas_height_mm;
  float left_mm_per_microstep;
  float right_mm_per_microstep;
};

struct CartesianPoint {
  float x_mm;
  float y_mm;
};

struct StepPosition {
  int32_t left;
  int32_t right;
};

inline bool isFinite(float value) {
  return isfinite(value);
}

inline bool isValidConfig(const MachineConfig &config) {
  return isFinite(config.anchor_span_mm) && isFinite(config.canvas_width_mm) &&
         isFinite(config.canvas_height_mm) && isFinite(config.left_mm_per_microstep) &&
         isFinite(config.right_mm_per_microstep) && config.anchor_span_mm > 0.0f &&
         config.canvas_width_mm > 0.0f && config.canvas_height_mm > 0.0f &&
         config.left_mm_per_microstep > 0.0f && config.right_mm_per_microstep > 0.0f;
}

inline bool isInBounds(const MachineConfig &config, const CartesianPoint &point) {
  return isValidConfig(config) && isFinite(point.x_mm) && isFinite(point.y_mm) &&
         point.x_mm >= 0.0f && point.x_mm <= config.canvas_width_mm &&
         point.y_mm >= 0.0f && point.y_mm <= config.canvas_height_mm;
}

inline bool roundToInt32(float value, int32_t *out) {
  if(!isFinite(value) || value < (float)INT32_MIN || value > (float)INT32_MAX)
    return false;
  *out = (int32_t)(value >= 0.0f ? floorf(value + 0.5f) : ceilf(value - 0.5f));
  return true;
}

inline bool cartesianToSteps(const MachineConfig &config, const CartesianPoint &point, StepPosition *out) {
  if(!isInBounds(config, point))
    return false;

  const float left_length = sqrtf(point.x_mm * point.x_mm + point.y_mm * point.y_mm);
  const float right_x = config.anchor_span_mm - point.x_mm;
  const float right_length = sqrtf(right_x * right_x + point.y_mm * point.y_mm);
  return roundToInt32(left_length / config.left_mm_per_microstep, &out->left) &&
         roundToInt32(right_length / config.right_mm_per_microstep, &out->right);
}

inline bool stepsToCartesian(const MachineConfig &config, const StepPosition &steps, CartesianPoint *out) {
  if(!isValidConfig(config) || steps.left < 0 || steps.right < 0)
    return false;

  const float left_length = steps.left * config.left_mm_per_microstep;
  const float right_length = steps.right * config.right_mm_per_microstep;
  const float span = config.anchor_span_mm;
  const float x = (span * span + left_length * left_length - right_length * right_length) / (2.0f * span);
  const float y_squared = left_length * left_length - x * x;
  // Integer quantisation can make a value just below zero at an anchor.
  if(!isFinite(x) || y_squared < -0.01f)
    return false;
  const float y = sqrtf(y_squared < 0.0f ? 0.0f : y_squared);
  if(!isFinite(y))
    return false;
  out->x_mm = x;
  out->y_mm = y;
  return isInBounds(config, *out);
}

inline int32_t stepDelta(int32_t from, int32_t to) {
  // Configured belt lengths are bounded well inside int32.  Do not evaluate
  // signed overflow if a malformed configuration somehow reaches this layer.
  const int64_t delta = (int64_t)to - (int64_t)from;
  return delta > INT32_MAX ? INT32_MAX : (delta < INT32_MIN ? INT32_MIN : (int32_t)delta);
}

inline uint32_t absoluteStepCount(int32_t value) {
  return value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

struct DdaTick {
  bool pulse_left;
  bool pulse_right;
};

// A deterministic two-axis DDA.  One call to next() is one shared planner
// tick; an axis is marked only when it must receive a physical pulse.
class TwoAxisDda {
public:
  TwoAxisDda(const StepPosition &from, const StepPosition &to)
      : left_delta_(stepDelta(from.left, to.left)), right_delta_(stepDelta(from.right, to.right)),
        left_total_(absoluteStepCount(left_delta_)), right_total_(absoluteStepCount(right_delta_)),
        ticks_(left_total_ > right_total_ ? left_total_ : right_total_), tick_(0), left_error_(0), right_error_(0) {}

  uint32_t totalTicks() const { return ticks_; }
  int32_t leftDelta() const { return left_delta_; }
  int32_t rightDelta() const { return right_delta_; }

  bool next(DdaTick *out) {
    if(tick_ >= ticks_)
      return false;
    ++tick_;
    left_error_ += left_total_;
    right_error_ += right_total_;
    out->pulse_left = left_error_ >= ticks_;
    out->pulse_right = right_error_ >= ticks_;
    if(out->pulse_left)
      left_error_ -= ticks_;
    if(out->pulse_right)
      right_error_ -= ticks_;
    return true;
  }

private:
  int32_t left_delta_;
  int32_t right_delta_;
  uint32_t left_total_;
  uint32_t right_total_;
  uint32_t ticks_;
  uint32_t tick_;
  uint32_t left_error_;
  uint32_t right_error_;
};

}  // namespace plottrbot
