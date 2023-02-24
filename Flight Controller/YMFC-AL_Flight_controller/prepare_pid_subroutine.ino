void prepare_pid_subroutine() {
  // Roll
  // Calculate the PID set point in degrees per second using the roll receiver input
  // A dead band of 16us is needed for better results
  int roll_input = receiver_input_channel_1;
  if (roll_input > 1508) {
    pid_roll_setpoint = (roll_input - 1508 - roll_level_adjust) / 3.0;
  } else if (roll_input < 1492) {
    pid_roll_setpoint = (roll_input - 1492 - roll_level_adjust) / 3.0;
  } else {
    pid_roll_setpoint = 0;
  }

  // Pitch
  // Calculate the PID set point in degrees per second using the pitch receiver input
  // A dead band of 16us is needed for better results
  int pitch_input = receiver_input_channel_2;
  if (pitch_input > 1508) {
    pid_pitch_setpoint = (pitch_input - 1508 - pitch_level_adjust) / 3.0;
  } else if (pitch_input < 1492) {
    pid_pitch_setpoint = (pitch_input - 1492 - pitch_level_adjust) / 3.0;
  } else {
    pid_pitch_setpoint = 0;
  }

  // Yaw
  // Calculate the PID set point in degrees per second using the yaw receiver input
  // A dead band of 16us is needed for better results
  int yaw_input = receiver_input_channel_3;
  if (yaw_input > 1050 && receiver_input_channel_4 > 1508) {
    pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
  } else if (yaw_input > 1050 && receiver_input_channel_4 < 1492) {
    pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  } else {
    pid_yaw_setpoint = 0;
  }
}
