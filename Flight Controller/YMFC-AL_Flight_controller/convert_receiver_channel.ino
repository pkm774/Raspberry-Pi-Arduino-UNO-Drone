// This part converts the actual receiver signals to
// a standardized 1000 – 1500 – 2000 microsecond value.
// The stored data in the EEPROM is used.

int convert_receiver_channel(byte function) {
  static const int SCALE_FACTOR = 500;
  static const int NEUTRAL_VALUE = 1500;

  byte channel = eeprom_data[function + 23] & 0b00000111;                            //What channel corresponds with the specific function
  bool reverse = (eeprom_data[function + 23] & 0b10000000) != 0;                     //Reverse channel when most significant bit is set
                                                                                     //If the most significant is not set there is no reverse

  int actual = receiver_input[channel];                                              //Read the actual receiver value for the corresponding function
  int low = (eeprom_data[channel * 2 + 14] << 8) | eeprom_data[channel * 2 + 15];    //Store the low value for the specific receiver input channel
  int center = (eeprom_data[channel * 2 - 2] << 8) | eeprom_data[channel * 2 - 1];   //Store the center value for the specific receiver input channel
  int high = (eeprom_data[channel * 2 + 6] << 8) | eeprom_data[channel * 2 + 7];     //Store the high value for the specific receiver input channel

  int difference;
  if (actual < center) {                                                             //The actual receiver value is lower than the center value
    actual = max(actual, low);                                                       //Limit the lowest value to the value that was detected during setup
    difference = ((center - actual) * SCALE_FACTOR) >> 9; // equivalent to division by (center - low) / 2^9
    return reverse ? (NEUTRAL_VALUE + difference) : (NEUTRAL_VALUE - difference);
  } else if (actual > center) {                                                      //The actual receiver value is higher than the center value
    actual = min(actual, high);                                                      //Limit the lowest value to the value that was detected during setup
    difference = ((actual - center) * SCALE_FACTOR) >> 9; // equivalent to division by (high - center) / 2^9
    return reverse ? (NEUTRAL_VALUE - difference) : (NEUTRAL_VALUE + difference);
  } else {
    return NEUTRAL_VALUE;
  }
}
