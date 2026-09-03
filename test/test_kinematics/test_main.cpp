#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <plottrbot/kinematics.h>
#include <plottrbot/protocol.h>

using namespace plottrbot;

static MachineConfig config() {
  MachineConfig value = {1162.0f, 581.0f, 240.0f, 1162.0f, 1000.0f, 0.018f, 0.019f};
  return value;
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

void test_endpoint_is_invariant_to_chord_count() {
  const MachineConfig machine = config();
  const CartesianPoint start = {581.0f, 240.0f};
  const CartesianPoint end = {960.0f, 700.0f};
  StepPosition initial, direct_target;
  assert(cartesianToSteps(machine, start, &initial));
  assert(cartesianToSteps(machine, end, &direct_target));

  StepPosition direct = initial;
  moveWithoutHardware(&direct, direct_target);
  assert(direct.left == direct_target.left && direct.right == direct_target.right);

  StepPosition segmented = initial;
  for(int part = 1; part <= 17; ++part) {
    const float t = part / 17.0f;
    const CartesianPoint chord = {start.x_mm + (end.x_mm - start.x_mm) * t, start.y_mm + (end.y_mm - start.y_mm) * t};
    StepPosition target;
    assert(cartesianToSteps(machine, chord, &target));
    moveWithoutHardware(&segmented, target);
  }
  assert(segmented.left == direct.left && segmented.right == direct.right);
}

void test_closed_path_and_return_have_no_step_drift() {
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
}

void test_bounds_and_bad_commands_do_not_change_state() {
  const MachineConfig machine = config();
  StepPosition state;
  assert(cartesianToSteps(machine, CartesianPoint{581.0f, 240.0f}, &state));
  const StepPosition original = state;
  StepPosition ignored;
  assert(!cartesianToSteps(machine, CartesianPoint{-0.01f, 2.0f}, &ignored));
  assert(!cartesianToSteps(machine, CartesianPoint{NAN, 2.0f}, &ignored));
  float parsed = 0.0f;
  assert(extractAxisFloat("G1 Xgarbage Y2", 'X', &parsed) == AXIS_MALFORMED);
  assert(extractAxisFloat("G1 X1e999 Y2", 'X', &parsed) == AXIS_MALFORMED);
  assert(state.left == original.left && state.right == original.right);
}

void test_dda_emits_exact_pulse_counts() {
  StepPosition state = {100, 100};
  const StepPosition target = {91, 116};
  moveWithoutHardware(&state, target);
  assert(state.left == target.left && state.right == target.right);
}

void test_structured_protocol_responses() {
  char response[220];
  formatM115(response, sizeof(response));
  assert(strstr(response, "FIRMWARE_NAME:PlottrBotUC") != nullptr);
  assert(strstr(response, "CAPABILITIES:ABS_STEP_MOTION") != nullptr);
  MachineConfig machine = config();
  StepPosition home;
  assert(cartesianToSteps(machine, CartesianPoint{machine.home_x_mm, machine.home_y_mm}, &home));
  formatM114(response, sizeof(response), machine, home);
  assert(strstr(response, "M114 X:") == response);
  assert(strstr(response, "L:") != nullptr && strstr(response, "R:") != nullptr);
}

int main() {
  test_endpoint_is_invariant_to_chord_count();
  test_closed_path_and_return_have_no_step_drift();
  test_bounds_and_bad_commands_do_not_change_state();
  test_dda_emits_exact_pulse_counts();
  test_structured_protocol_responses();
  return 0;
}
