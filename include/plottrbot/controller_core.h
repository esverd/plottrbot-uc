#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kinematics.h"

namespace plottrbot {

struct ControllerState {
  bool position_known;
  StepPosition steps;
};

enum CommandKind {
  COMMAND_NONE,
  COMMAND_M115,
  COMMAND_M114,
  COMMAND_M503,
  COMMAND_SAFE_DIAGNOSTIC,
  COMMAND_G92_HOME,
  COMMAND_G1,
  COMMAND_G28,
  COMMAND_M17,
  COMMAND_M18
};

struct CommandPlan {
  CommandKind kind;
  bool change_pen;
  bool pen_draw;
  bool enable_drivers;
  bool disable_drivers;
  CartesianMovePlan motion;
  StepPosition home_steps;
};

struct ParsedG1 {
  bool has_x;
  bool has_y;
  bool has_z;
  float x;
  float y;
  float z;
};

inline void resetPlan(CommandPlan *plan) {
  memset(plan, 0, sizeof(CommandPlan));
  plan->kind = COMMAND_NONE;
}

inline const char *skipSpaces(const char *cursor) {
  while(*cursor == ' ' || *cursor == '\t')
    ++cursor;
  return cursor;
}

inline bool commandStartsWith(const char *command, const char *code, const char **after_code) {
  const size_t length = strlen(code);
  if(strncmp(command, code, length) != 0)
    return false;
  const char next = command[length];
  if(next != '\0' && next != ' ' && next != '\t')
    return false;
  *after_code = command + length;
  return true;
}

inline bool commandHasNoArguments(const char *command, const char *code) {
  const char *after_code = nullptr;
  return commandStartsWith(command, code, &after_code) && *skipSpaces(after_code) == '\0';
}

inline bool commandHasExactArgument(const char *command, const char *code, const char *argument) {
  const char *after_code = nullptr;
  if(!commandStartsWith(command, code, &after_code))
    return false;
  const char *cursor = skipSpaces(after_code);
  const size_t argument_length = strlen(argument);
  if(strncmp(cursor, argument, argument_length) != 0)
    return false;
  cursor += argument_length;
  return *cursor == '\0' || ((*cursor == ' ' || *cursor == '\t') && *skipSpaces(cursor) == '\0');
}

inline bool parseFloatToken(const char *start, const char *end, float *out) {
  char *parsed_end = nullptr;
  const float value = (float)strtod(start, &parsed_end);
  if(parsed_end == start || parsed_end != end || !isFinite(value))
    return false;
  *out = value;
  return true;
}

inline bool parseG1(const char *command, ParsedG1 *out) {
  const char *cursor = nullptr;
  if(!(commandStartsWith(command, "G1", &cursor) || commandStartsWith(command, "G01", &cursor)))
    return false;
  memset(out, 0, sizeof(ParsedG1));
  cursor = skipSpaces(cursor);
  if(*cursor == '\0')
    return false;
  while(*cursor != '\0') {
    const char *token_start = cursor;
    while(*cursor != '\0' && *cursor != ' ' && *cursor != '\t')
      ++cursor;
    const char *token_end = cursor;
    if(token_end - token_start < 2)
      return false;
    float value = 0.0f;
    if(!parseFloatToken(token_start + 1, token_end, &value))
      return false;
    if(*token_start == 'X' && !out->has_x) {
      out->has_x = true; out->x = value;
    } else if(*token_start == 'Y' && !out->has_y) {
      out->has_y = true; out->y = value;
    } else if(*token_start == 'Z' && !out->has_z) {
      out->has_z = true; out->z = value;
    } else {
      return false;
    }
    cursor = skipSpaces(cursor);
  }
  return out->has_x || out->has_y || out->has_z;
}

inline bool planCommand(const char *command, const MachineConfig &config,
                        const ControllerState &state, float max_chord_mm,
                        CommandPlan *plan, const char **error) {
  resetPlan(plan);
  *error = nullptr;
  if(commandHasNoArguments(command, "M115")) { plan->kind = COMMAND_M115; return true; }
  if(commandStartsWith(command, "M115", error)) { *error = "malformed:M115"; return false; }
  if(commandHasNoArguments(command, "M114")) { plan->kind = COMMAND_M114; return true; }
  if(commandStartsWith(command, "M114", error)) { *error = "malformed:M114"; return false; }
  if(commandHasNoArguments(command, "M503")) { plan->kind = COMMAND_M503; return true; }
  if(commandStartsWith(command, "M503", error)) { *error = "malformed:M503"; return false; }
  if(commandHasExactArgument(command, "M950", "SAFE")) { plan->kind = COMMAND_SAFE_DIAGNOSTIC; return true; }
  if(commandStartsWith(command, "M950", error)) { *error = "malformed:M950"; return false; }

  const char *ignored = nullptr;
  if(commandStartsWith(command, "G5", &ignored)) {
    *error = commandHasNoArguments(command, "G5") ? "unsupported:G5_flatten_on_host" : "malformed:G5";
    return false;
  }
  if(commandStartsWith(command, "G92", &ignored)) {
    if(!commandHasExactArgument(command, "G92", "H")) { *error = "malformed:G92"; return false; }
    if(!isValidConfig(config) || !cartesianToSteps(config, CartesianPoint{config.home_x_mm, config.home_y_mm}, &plan->home_steps)) {
      *error = "config:invalid"; return false;
    }
    plan->kind = COMMAND_G92_HOME;
    return true;
  }
  if(commandStartsWith(command, "G1", &ignored) || commandStartsWith(command, "G01", &ignored)) {
    ParsedG1 parsed;
    if(!parseG1(command, &parsed)) { *error = "malformed:G1"; return false; }
    if(!state.position_known) { *error = "state:home_required"; return false; }
    if(parsed.has_z && parsed.z != 0.0f && parsed.z != 1.0f) { *error = "bounds:Z"; return false; }
    if(parsed.has_x || parsed.has_y) {
      CartesianPoint target;
      if(!stepsToCartesian(config, state.steps, &target)) { *error = "state:unresolvable"; return false; }
      if(parsed.has_x) target.x_mm = parsed.x;
      if(parsed.has_y) target.y_mm = parsed.y;
      if(!preflightCartesianMove(config, state.steps, target, max_chord_mm, &plan->motion)) {
        *error = "motion:unsafe"; return false;
      }
    }
    plan->kind = COMMAND_G1;
    plan->change_pen = parsed.has_z;
    plan->pen_draw = parsed.has_z && parsed.z == 0.0f;
    return true;
  }
  if(commandStartsWith(command, "G28", &ignored)) {
    if(!commandHasNoArguments(command, "G28")) { *error = "malformed:G28"; return false; }
    if(!state.position_known) { *error = "state:home_required"; return false; }
    const CartesianPoint home = {config.home_x_mm, config.home_y_mm};
    if(!preflightCartesianMove(config, state.steps, home, max_chord_mm, &plan->motion)) {
      *error = "motion:unsafe"; return false;
    }
    plan->kind = COMMAND_G28;
    return true;
  }
  if(commandStartsWith(command, "M17", &ignored)) {
    if(!commandHasNoArguments(command, "M17")) { *error = "malformed:M17"; return false; }
    plan->kind = COMMAND_M17; plan->enable_drivers = true; return true;
  }
  if(commandStartsWith(command, "M18", &ignored)) {
    if(!commandHasNoArguments(command, "M18")) { *error = "malformed:M18"; return false; }
    plan->kind = COMMAND_M18; plan->disable_drivers = true; return true;
  }
  *error = "unsupported:command";
  return false;
}

inline void formatSafeDiagnostic(char *out, size_t out_size, const MachineConfig &config, float max_chord_mm) {
  ControllerState unknown = {false, {0, 0}};
  CommandPlan motion_plan;
  CommandPlan home_plan;
  const char *error = nullptr;
  const bool rejected = !planCommand("G1 X0", config, unknown, max_chord_mm, &motion_plan, &error) &&
      error != nullptr && strcmp(error, "state:home_required") == 0 && motion_plan.kind == COMMAND_NONE &&
      !motion_plan.change_pen && !motion_plan.enable_drivers && !motion_plan.disable_drivers;
  error = nullptr;
  const bool logical_home = planCommand("G92 H", config, unknown, max_chord_mm, &home_plan, &error) &&
      home_plan.kind == COMMAND_G92_HOME && !home_plan.change_pen && !home_plan.enable_drivers &&
      !home_plan.disable_drivers && home_plan.motion.chord_count == 0 && unknown.position_known == false;
  const bool safe = rejected && logical_home;
  snprintf(out, out_size, "M950 SAFE PREHOME_G1:%s G92_HOME_LOGICAL:%s NO_ACTUATION:%s",
      rejected ? "ERR_state_home_required" : "FAIL", logical_home ? "PASS" : "FAIL", safe ? "PASS" : "FAIL");
}

}  // namespace plottrbot
