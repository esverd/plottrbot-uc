#pragma once

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "kinematics.h"

namespace plottrbot {

enum AxisValueResult { AXIS_ABSENT, AXIS_VALID, AXIS_MALFORMED };

inline AxisValueResult extractAxisFloat(const char *command, char axis, float *out) {
  const char *position = strchr(command, axis);
  if(position == nullptr)
    return AXIS_ABSENT;
  ++position;
  while(*position == ' ' || *position == '\t')
    ++position;
  if(*position == '\0')
    return AXIS_MALFORMED;
  char *end = nullptr;
  // avr-libc exposes strtod but not strtof on this Nano toolchain.
  const float value = (float)strtod(position, &end);
  if(end == position || !isFinite(value) || (*end != '\0' && *end != ' ' && *end != '\t'))
    return AXIS_MALFORMED;
  *out = value;
  return AXIS_VALID;
}

inline void formatM115(char *out, size_t out_size) {
  snprintf(out, out_size,
      "M115 FIRMWARE_NAME:PlottrBotUC FIRMWARE_VERSION:2026.09.03 PROTOCOL_VERSION:2 CAPABILITIES:ABS_STEP_MOTION,G1_XY,G92_HOME,HOME_CONFIRM_REQUIRED,M114_STEPS,SAFE_DIAGNOSTIC,G5_REJECTED");
}

inline void formatFixedMillimetres(char *out, size_t out_size, float value) {
  int32_t thousandths = 0;
  if(!roundToInt32(value * 1000.0f, &thousandths)) {
    snprintf(out, out_size, "UNKNOWN");
    return;
  }
  const bool negative = thousandths < 0;
  // Avoid negating INT32_MIN while keeping this tiny formatting helper free
  // of the Nano's comparatively expensive int64 division routines.
  const uint32_t magnitude = negative ? (uint32_t)(-(thousandths + 1)) + 1U : (uint32_t)thousandths;
  snprintf(out, out_size, "%s%lu.%03lu", negative ? "-" : "", (unsigned long)(magnitude / 1000U), (unsigned long)(magnitude % 1000U));
}

inline void formatM114(char *out, size_t out_size, const MachineConfig &config, const StepPosition &steps, bool position_known) {
  CartesianPoint point;
  if(position_known && stepsToCartesian(config, steps, &point)) {
    char x[16];
    char y[16];
    formatFixedMillimetres(x, sizeof(x), point.x_mm);
    formatFixedMillimetres(y, sizeof(y), point.y_mm);
    snprintf(out, out_size, "M114 X:%s Y:%s L:%ld R:%ld", x, y, (long)steps.left, (long)steps.right);
  } else {
    snprintf(out, out_size, "M114 X:UNKNOWN Y:UNKNOWN L:UNKNOWN R:UNKNOWN");
  }
}

}  // namespace plottrbot
