///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//For debugging and testing purpose
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define DEBUG                              //Enable for serial Monitoring at 57600 baud.
// Uncomment according to purpose.
//#define DEBUG_BATTERY
//#define DEBUG_ANGLE
//#define DEBUG_ESC
//#define DEBUG_RCSIGNAL

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Maximum allowed signal and motor pulse
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int allowed_limit = 1850;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const float battery_compensation = 40.00;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 2.2;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.00;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
const int pid_max_roll = 400;              //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
const int pid_max_pitch = pid_max_roll;    //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
const int pid_max_yaw = 400;               //Maximum output of the PID-controller (+/-)

const boolean auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t eeprom_data[36];
uint8_t highByte, lowByte;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, cal_int, start, temperature;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int receiver_input[5];
int acc_axis[4], gyro_axis[4];
int gyro_address;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;

int32_t receiver_input_channel_1, counter_channel_1;
int32_t receiver_input_channel_2, counter_channel_2;
int32_t receiver_input_channel_3, counter_channel_3;
int32_t receiver_input_channel_4, counter_channel_4;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
#if defined(DEBUG)
  Serial.begin(57600);
#endif

  //Copy the EEPROM data for fast access data.
  for (int16_t data = 0; data <= 35; data++)eeprom_data[data] = EEPROM.read(data);

  start = 0;                                                                //Set start to zero.
  cal_int = 0;                                                              //Set cal_int to zero.

  gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.

  Wire.begin();                                                             //Start the I2C as master.
  Wire.setClock(400000);                                                    //Set the SCL clock speed to 400kHz

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                                        //Configure digital poort 12 and 13 as output.

  //Use the led on the Arduino for startup indication.
  digitalWrite(12, HIGH);                                                   //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while (eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program
  if (eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();                                                     //Set the specific gyro registers.

  for (int16_t pulse = 0; pulse < 1250 ; pulse ++) {                        //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }

  callibrate_gyro();                                                        //Callibrate the gyro.

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throtle is set to the lower position.
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if (start == 125) {                                                     //Every 125 loops (500ms).
      digitalWrite(12, !digitalRead(12));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.

  //Load the battery voltage to the battery_voltage variable.zz
  //12.6V equals ~4.2V @ Analog 0 with 1K ohm and 2K ohm.
  //5V Equals to 1023 analog input.
  //4.2V  = 4.2 * (1023/5) = 859.32 analog input.
  //12.6V equals 859.32 analogRead(0).
  //1260 / 859.32 = 1.466.
  //Compensation +10 due to internal circuit.
  battery_voltage = ((float)analogRead(0) * 1.466) + 10; // /100 = Voltage
  //The variable battery_voltage holds 1265 if the battery voltage is 12.65V.
#if defined(DEBUG_BATTERY)
  Serial.print(F("AnalogRead(0) = ")); Serial.print((float)analogRead(0));
  Serial.print(F(" Battery_voltage = ")); Serial.println(battery_voltage);
#endif

  loop_timer = micros();                                                    //Set the timer for the next loop.

  //When everything is done, turn off the led.
  digitalWrite(12, LOW);                                                    //Turn off the warning led.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;       //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;        //Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

#if defined(DEBUG_ANGLE)
  Serial.print(F(" angle_pitch = ")); Serial.print(angle_pitch);
  Serial.print(F(" angle_roll = ")); Serial.println(angle_roll);
#endif

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if (!auto_level) {                                                        //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }

#if defined(DEBUG_ANGLE)
  Serial.print(F(" pitch_level_adjust = ")); Serial.print(pitch_level_adjust);
  Serial.print(F(" roll_level_adjust = ")); Serial.println(roll_level_adjust);
#endif

  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450) {
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

  calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.117 = 0.08 * 1.466.
  // +1 for error(+-1) compensation.
  battery_voltage = (battery_voltage * 0.92) + ((float)analogRead(0) * 0.117) + 1;
#if defined(DEBUG_BATTERY)
  Serial.print(F("AnalogRead(0) = ")); Serial.print((float)analogRead(0));
  Serial.print(F(" Battery_voltage = ")); Serial.println(battery_voltage);
#endif

  //Turn on the led if battery voltage is to low.
  if (battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12, HIGH);

  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

#if defined(DEBUG_RCSIGNAL)
  Serial.print(F(" Throttle signal = ")); Serial.println(throttle);
#endif

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/fqEkVcqxtU8
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  if (start == 2) {                                                         //The motors are started.

    if (throttle > allowed_limit) throttle = allowed_limit;                 //We need some room to keep full control at full throttle.

    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 900) {                  //Is the battery connected?
      esc_1 += (1240 - battery_voltage) / battery_compensation;             //Compensate the esc-1 pulse for voltage drop.
      esc_2 += (1240 - battery_voltage) / battery_compensation;             //Compensate the esc-2 pulse for voltage drop.
      esc_3 += (1240 - battery_voltage) / battery_compensation;             //Compensate the esc-3 pulse for voltage drop.
      esc_4 += (1240 - battery_voltage) / battery_compensation;             //Compensate the esc-4 pulse for voltage drop.
    }

    //These values can vary according to the ESCs.
    //So it is necessary to recalculate errors and compensate
    //values after changing ESCs.
    esc_1 -= 3;                                                            // Correct the esc-1 pulse error.
    esc_2 += 20;                                                           // Correct the esc-2 pulse error.
    esc_3 += 10;                                                           // Correct the esc-3 pulse error.
    esc_4 -= 7;                                                            // Correct the esc-4 pulse error.

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if (esc_1 > allowed_limit)esc_1 = allowed_limit;                        //Limit the esc-1 pulse to 1950us for stability.
    if (esc_2 > allowed_limit)esc_2 = allowed_limit;                        //Limit the esc-2 pulse to 1950us for stability.
    if (esc_3 > allowed_limit)esc_3 = allowed_limit;                        //Limit the esc-3 pulse to 1950us for stability.
    if (esc_4 > allowed_limit)esc_4 = allowed_limit;                        //Limit the esc-4 pulse to 1950us for stability.
  }

  else {
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

#if defined(DEBUG_ESC)
  Serial.print(F("esc_1 = ")); Serial.print(esc_1);
  Serial.print(F(" esc_2 = ")); Serial.print(esc_2);
  Serial.print(F(" esc_3 = ")); Serial.print(esc_3);
  Serial.print(F(" esc_4 = ")); Serial.println(esc_4);
#endif

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer! More information can be found on
  //the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  if (micros() - loop_timer > 4050)digitalWrite(12, HIGH);                  //Turn on the LED if the loop time exceeds 4050us.

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while (micros() - loop_timer < 4000);                                     //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.

  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  read_gyro_data();

  while (PORTD >= 16) {                                                     //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;               //Set digital output 4 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;               //Set digital output 5 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;               //Set digital output 6 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;               //Set digital output 7 to low if the time is expired.
  }
}
