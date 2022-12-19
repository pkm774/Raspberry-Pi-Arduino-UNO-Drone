///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void callibrate_gyro(void) {
  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                        //Take 2000 readings for calibration.
      if (cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));             //Change the led status to indicate calibration.
      read_gyro_data();                                                     //Read the gyro output.
      gyro_roll_cal += gyro_axis[1];                                        //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_axis[3];                                         //Ad yaw value to gyro_yaw_cal.

      // Avoid ESC Beep sound while callibrating
      PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
      delayMicroseconds(1000);                                                //Wait 1000us.
      PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.

      delay(4);                                                             //Small delay to simulate a 250Hz loop during calibration.
    }

    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                  //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                 //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                   //Divide the yaw total by 2000.

#if defined(DEBUG_ANGLE)
  Serial.print(F(" gyro_roll_cal = "));Serial.print(gyro_roll_cal);
  Serial.print(F(" gyro_pitch_cal = "));Serial.print(gyro_pitch_cal);
  Serial.print(F(" gyro_yaw_cal = "));Serial.println(gyro_yaw_cal);
#endif
  }
}
