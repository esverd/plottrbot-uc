#include <assert.h>
#include <string.h>
#include <math.h>
#include <plottrbot/kinematics.h>
#include <plottrbot/protocol.h>
#include <plottrbot/controller_core.h>

using namespace plottrbot;

static MachineConfig config() {
  MachineConfig value = {1162.0f, 581.0f, 240.0f, 1162.0f, 1000.0f, 0.018f, 0.019f};
  return value;
}

static ControllerState homedState(const MachineConfig &machine) {
  StepPosition home;
  assert(cartesianToSteps(machine, CartesianPoint{machine.home_x_mm, machine.home_y_mm}, &home));
  ControllerState state = {true, home};
  return state;
}

static bool plan(const char *command, const MachineConfig &machine, const ControllerState &state,
                 CommandPlan *command_plan, const char **error) {
  return planCommand(command, machine, state, 2.0f, command_plan, error);
}

static void moveWithoutHardware(StepPosition *state, const StepPosition &target) {
  TwoAxisDda dda(*state, target);
  DdaTick tick;
  while(dda.next(&tick)) {
    if(tick.pulse_left)
      state->left += dda.leftDelta() < 0 ? -1 : 1;
    if(tick.pulse_right)
      state->right += dda.rightDelta() < 0 ? -1 : 1;
  }
}

void test_unknown_on_boot_and_explicit_g92_home_transition() {
  const MachineConfig machine = config();
  ControllerState state = {false, {0, 0}};
  char response[230];
  formatM114(response, sizeof(response), machine, state.steps, state.position_known);
  assert(strcmp(response, "M114 X:UNKNOWN Y:UNKNOWN L:UNKNOWN R:UNKNOWN") == 0);

  CommandPlan command_plan;
  const char *error = nullptr;
  assert(plan("G92 H", machine, state, &command_plan, &error));
  assert(command_plan.kind == COMMAND_G92_HOME);
  // G92 H is a logical state plan, not an actuator plan.
  assert(!command_plan.change_pen && !command_plan.enable_drivers && !command_plan.disable_drivers);
  assert(command_plan.motion.chord_count == 0 && command_plan.motion.total_pulses == 0);
  assert(!state.position_known && state.steps.left == 0 && state.steps.right == 0);

  state.steps = command_plan.home_steps;
  state.position_known = true;
  formatM114(response, sizeof(response), machine, state.steps, state.position_known);
  assert(strstr(response, "M114 X:") == response);
  assert(strstr(response, "UNKNOWN") == nullptr);
}

void test_motion_is_rejected_before_home_without_effects() {
  const MachineConfig machine = config();
  const ControllerState state = {false, {0, 0}};
  const char *commands[] = {"G1 X600", "G1 Z0", "G28"};
  for(unsigned int index = 0; index < sizeof(commands) / sizeof(commands[0]); ++index) {
    CommandPlan command_plan;
    const char *error = nullptr;
    assert(!plan(commands[index], machine, state, &command_plan, &error));
    assert(strcmp(error, "state:home_required") == 0);
    assert(command_plan.kind == COMMAND_NONE);
    assert(!command_plan.change_pen && !command_plan.enable_drivers && !command_plan.disable_drivers);
    assert(state.steps.left == 0 && state.steps.right == 0 && !state.position_known);
  }
}

void test_rejected_mixed_g1_is_transactional() {
  const MachineConfig machine = config();
  const ControllerState state = homedState(machine);
  CommandPlan command_plan;
  const char *error = nullptr;
  assert(!plan("G1 X2000 Z0", machine, state, &command_plan, &error));
  assert(strcmp(error, "motion:unsafe") == 0);
  assert(command_plan.kind == COMMAND_NONE);
  assert(!command_plan.change_pen && !command_plan.enable_drivers && !command_plan.disable_drivers);
  assert(state.position_known);
  assert(state.steps.left == homedState(machine).steps.left && state.steps.right == homedState(machine).steps.right);
}

void test_strict_command_parser() {
  const MachineConfig machine = config();
  const ControllerState state = homedState(machine);
  const char *commands[] = {"M115 junk", "M114 X0", "M503 now", "G28 X0", "G92 H X0", "G92 X0", "M17 now", "G1 X1 Y2 F100", "G1 X1 X2", "G1 Xgarbage"};
  for(unsigned int index = 0; index < sizeof(commands) / sizeof(commands[0]); ++index) {
    CommandPlan command_plan;
    const char *error = nullptr;
    assert(!plan(commands[index], machine, state, &command_plan, &error));
    assert(strncmp(error, "malformed:", 10) == 0);
    assert(command_plan.kind == COMMAND_NONE);
  }
  CommandPlan command_plan;
  const char *error = nullptr;
  assert(plan("M115   ", machine, state, &command_plan, &error));
  assert(command_plan.kind == COMMAND_M115);
}

void test_endpoint_is_invariant_to_chord_count() {
  const MachineConfig machine = config();
  const CartesianPoint start = {581.0f, 240.0f};
  const CartesianPoint end = {960.0f, 700.0f};
  StepPosition initial, direct_target;
  assert(cartesianToSteps(machine, start, &initial));
  assert(cartesianToSteps(machine, end, &direct_target));
  StepPosition direct = initial;
  moveWithoutHardware(&direct, direct_target);

  StepPosition segmented = initial;
  for(int part = 1; part <= 17; ++part) {
    const float t = part / 17.0f;
    const CartesianPoint chord = {start.x_mm + (end.x_mm - start.x_mm) * t, start.y_mm + (end.y_mm - start.y_mm) * t};
    StepPosition target;
    assert(cartesianToSteps(machine, chord, &target));
    moveWithoutHardware(&segmented, target);
  }
  assert(direct.left == direct_target.left && direct.right == direct_target.right);
  assert(segmented.left == direct.left && segmented.right == direct.right);
}

void test_closed_path_and_dda_pulse_counts() {
  const MachineConfig machine = config();
  const CartesianPoint points[] = {{300.0f, 300.0f}, {800.0f, 300.0f}, {800.0f, 700.0f}, {300.0f, 700.0f}, {300.0f, 300.0f}};
  StepPosition start;
  assert(cartesianToSteps(machine, points[0], &start));
  StepPosition state = start;
  for(int repetition = 0; repetition < 10; ++repetition) {
    for(unsigned int index = 1; index < sizeof(points) / sizeof(points[0]); ++index) {
      StepPosition target;
      assert(cartesianToSteps(machine, points[index], &target));
      moveWithoutHardware(&state, target);
    }
    assert(state.left == start.left && state.right == start.right);
  }
  const StepPosition small_target = {91, 116};
  state = StepPosition{100, 100};
  moveWithoutHardware(&state, small_target);
  assert(state.left == small_target.left && state.right == small_target.right);
}

void test_independent_scales_and_configuration_limits() {
  MachineConfig machine = config();
  const CartesianPoint point = {732.5f, 432.25f};
  StepPosition steps;
  CartesianPoint restored;
  assert(cartesianToSteps(machine, point, &steps));
  assert(stepsToCartesian(machine, steps, &restored));
  assert(fabsf(restored.x_mm - point.x_mm) < 0.1f);
  assert(fabsf(restored.y_mm - point.y_mm) < 0.1f);
  machine.left_mm_per_microstep = 0.0f;
  assert(!isValidConfig(machine));
  machine = config();
  machine.canvas_width_mm = machine.anchor_span_mm + 1.0f;
  assert(!isValidConfig(machine));
  machine = config();
  machine.home_y_mm = machine.canvas_height_mm + 1.0f;
  assert(!isValidConfig(machine));
}

void test_protocol_capabilities_and_safe_diagnostic() {
  const MachineConfig machine = config();
  char response[230];
  formatM115(response, sizeof(response));
  assert(strstr(response, "PROTOCOL_VERSION:2") != nullptr);
  assert(strstr(response, "HOME_CONFIRM_REQUIRED") != nullptr);
  formatSafeDiagnostic(response, sizeof(response), machine, 2.0f);
  assert(strcmp(response, "M950 SAFE PREHOME_G1:ERR_state_home_required G92_HOME_LOGICAL:PASS NO_ACTUATION:PASS") == 0);
}

int main() {
  test_unknown_on_boot_and_explicit_g92_home_transition();
  test_motion_is_rejected_before_home_without_effects();
  test_rejected_mixed_g1_is_transactional();
  test_strict_command_parser();
  test_endpoint_is_invariant_to_chord_count();
  test_closed_path_and_dda_pulse_counts();
  test_independent_scales_and_configuration_limits();
  test_protocol_capabilities_and_safe_diagnostic();
  return 0;
}
