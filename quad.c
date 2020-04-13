#include <Wire.h>   //Include the Wire.h library so we can communicate with the gyro.
TwoWire HWire(2, I2C_FAST_MODE);          //Initiate I2C port 2 at 400kHz.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.45; //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.001;            //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 3.15;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

boolean auto_level = true;                 //Auto level on (true) or off (false).

//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 38;
int16_t manual_acc_roll_cal_value = -33;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false;    // Set to false or true;
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;

uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t highByte, lowByte, flip32, start;
uint8_t error, error_counter, error_led;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;


int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3,k;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

uint32_t loop_timer, error_timer;

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
  pinMode(4, INPUT_ANALOG);                        //This is needed for reading the analog value of port A4.
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.
  //The following function connects PB3 and PB4 to the
  //alternate output function.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                    //Connects PB3 and PB4 to output function.

  //On the Flip32 the LEDs are connected differently. A check is needed for controlling the LEDs.
  pinMode(PB3, INPUT);                                         //Set PB3 as input.
  pinMode(PB4, INPUT);                                         //Set PB4 as input.
  if (digitalRead(PB3) || digitalRead(PB3))flip32 = 1;         //Input PB3 and PB4 are high on the Flip32
  else flip32 = 0;

  pinMode(PB3, OUTPUT);                                         //Set PB3 as output.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output.

  green_led(LOW);                                               //Set output PB3 low.
  red_led(HIGH);                                                //Set output PB4 high.

  //Serial.begin(57600);                          //Set the serial output to 57600 kbps. (for debugging only)
  //delay(250);                                //Give the serial port some time to start to prevent data loss.

  timer_setup();                                  //Setup the timers for the receiver inputs and ESC's output.
  delay(50);                                                    //Give the timers some time to start.

  HWire.begin();                                                //Start the I2C as master
  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  error = HWire.endTransmission();                        //End the transmission and register the exit status.
  while (error != 0) {                                //Stay in this loop because the MPU-6050 did not responde.
    error = 2;                                                  //Set the error status to 2.
    error_signal();                                             //Show the error via the red LED.
    delay(4);
  }

  gyro_setup();                                           //Initiallize the gyro and set the correct registers.

  if (!use_manual_calibration) {
    //Create a 5 second delay before calibration.
    for (count_var = 0; count_var < 1250; count_var++) {        //1250 loops of 4 microseconds = 5 seconds
      if (count_var % 125 == 0) {                               //Every 125 loops (500ms).
        digitalWrite(PB4, !digitalRead(PB4));                   //Change the led status.
      }
      delay(4);                                                 //Delay 4 microseconds
    }
    count_var = 0;                                              //Set start back to 0.
  }

  calibrate_gyro();                                             //Calibrate the gyro offset.

  //Wait until the receiver is active.

  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    error = 3;                                                  //Set the error status to 3.
    error_signal();                                             //Show the error via the red LED.
    delay(4);
  }
  error = 0;                                                    //Reset the error status to 0.

  //Wait until the throtle is set to the lower position.
  while (channel_3 < 990 || channel_3 > 1050)  {
    error = 4;                                                  //Set the error status to 4.
    error_signal();                                             //Show the error via the red LED.
    delay(4);
  }
  error = 0;                                                    //Reset the error status to 0.

  //When everything is done, turn off the led.
  red_led(LOW);                                                 //Set output PB4 low.

  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  battery_voltage = (float)analogRead(4) / 112.81;

  loop_timer = micros();                                        //Set the timer for the first loop.

  green_led(HIGH);                                              //Turn on the green led.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  error_signal();                                                     //Show the errors via the red LED.
  gyro_signalen();                                                     //Read the gyro and accelerometer data.

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

 

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611; //Calculate the traveled pitch angle and add this to the         angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;        //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = (angle_pitch - 0.1)* 15;                                           //Calculate the pitch angle correction.
  roll_level_adjust = (angle_roll - 0.2) * 15;                                             //Calculate the roll angle correction.

  if (!auto_level) {                                                               //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                        //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                         //Set the roll angle correcion to zero.
  }


  //For starting the motors: throttle low and yaw left (step 1).
  if (channel_3 < 1050 && channel_4 < 1070)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1425) {
    start = 2;

    green_led(LOW);                                                                //Turn off the green led.

    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1850) {
    start = 0;
    green_led(HIGH);                                                               //Turn on the green led.
  }

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.

  if (channel_1 > 1508)pid_roll_setpoint = channel_1 - 1508;
  else if (channel_1 < 1492)pid_roll_setpoint = channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
  else if (channel_2 < 1492)pid_pitch_setpoint = channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
    else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
  }

  calculate_pid();                                                                 //PID inputs are known. So we can calculate the pid output.

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);

  //Turn on the led if battery voltage is to low. In this case under 10.0V
  if (battery_voltage < 10.0 && error == 0)error = 1;
  

  throttle = map(channel_3,1000, 1950,1000,1650);                                                           //We need the throttle signal as a base signal.
   

  if (start == 2) {                                                                //The motors are started.
    if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

    if (esc_1 < 1100) esc_1 = 1100;                                                //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                                //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                                //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                                //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
  }

  TIMER4_BASE->CCR1 = esc_1;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
  TIMER4_BASE->CCR2 = esc_2;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
  TIMER4_BASE->CCR3 = esc_3;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
  TIMER4_BASE->CCR4 = esc_4;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.
  TIMER4_BASE->CNT = 5000;                                                         //This will reset timer 4 and the ESC pulses are directly created.


//>>> LED_CONTROL
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//These functions handle the red and green LEDs. The LEDs on the flip 32 are inverted. That is why a Flip32 test is needed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void red_led(int8_t level) {
  if (flip32)digitalWrite(PB4, !level);    //If a Flip32 is detected invert the output.
  else digitalWrite(PB4, level);           //When using the BluePill the output should not be inverted.
}
void green_led(int8_t level) {
  if (flip32)digitalWrite(PB3, !level);    //If a Flip32 is detected invert the output.
  else digitalWrite(PB3, level);           //When using the BluePill the output should not be inverted.
}


//>>> CALCULATE_PID
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(void){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

//>>> CALIBRATE_GYR0
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  if (use_manual_calibration)cal_int = 2000;                                          //If manual calibration is used set cal_int to 2000 to skip the calibration.
  else {
    cal_int = 0;                                                                      //If manual calibration is not used.
    manual_gyro_pitch_cal_value = 0;                                                  //Set the manual pitch calibration variable to 0.
    manual_gyro_roll_cal_value = 0;                                                   //Set the manual roll calibration variable to 0.
    manual_gyro_yaw_cal_value = 0;                                                    //Set the manual yaw calibration variable to 0.
  }

  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
      if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4));                    //Change the led status every 125 readings to indicate calibration.
      gyro_signalen();                                                                //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
      delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
    }
    red_led(HIGH);                                                                     //Set output PB3 low.
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
    manual_gyro_pitch_cal_value = gyro_pitch_cal;                                     //Set the manual pitch calibration variable to the detected value.
    manual_gyro_roll_cal_value = gyro_roll_cal;                                       //Set the manual roll calibration variable to the detected value.
    manual_gyro_yaw_cal_value = gyro_yaw_cal;                                         //Set the manual yaw calibration variable to the detected value.
  }
}

//>>> GYRO_SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the various registers of the MPU-6050 are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void){
  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  HWire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  HWire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  HWire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  HWire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  HWire.endTransmission();                                      //End the transmission with the gyro.
}
//>>> INPUT_CAPTURE_MODE_HANDLERS
void handler_channel_1(void) {                           //This function is called when channel 1 is captured.
  if (0b1 & GPIOA_BASE->IDR  >> 0) {                     //If the receiver channel 1 input pulse on A0 is high.
    channel_1_start = TIMER2_BASE->CCR1;                 //Record the start time of the pulse.
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 1 input pulse on A0 is low.
    channel_1 = TIMER2_BASE->CCR1 - channel_1_start;     //Calculate the total pulse time.
    if (channel_1 < 0)channel_1 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;               //Change the input capture mode to the rising edge of the pulse.
  }
}

void handler_channel_2(void) {                           //This function is called when channel 2 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 1) {                      //If the receiver channel 2 input pulse on A1 is high.
    channel_2_start = TIMER2_BASE->CCR2;                 //Record the start time of the pulse.
    TIMER2_BASE->CCER |= TIMER_CCER_CC2P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 2 input pulse on A1 is low.
    channel_2 = TIMER2_BASE->CCR2 - channel_2_start;     //Calculate the total pulse time.
    if (channel_2 < 0)channel_2 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC2P;               //Change the input capture mode to the rising edge of the pulse.
  }
}

void handler_channel_3(void) {                           //This function is called when channel 3 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 2) {                      //If the receiver channel 3 input pulse on A2 is high.
    channel_3_start = TIMER2_BASE->CCR3;                 //Record the start time of the pulse.
    TIMER2_BASE->CCER |= TIMER_CCER_CC3P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 3 input pulse on A2 is low.
    channel_3 = TIMER2_BASE->CCR3 - channel_3_start;     //Calculate the total pulse time.
    if (channel_3 < 0)channel_3 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC3P;               //Change the input capture mode to the rising edge of the pulse.
  }
}

void handler_channel_4(void) {                           //This function is called when channel 4 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 3) {                      //If the receiver channel 4 input pulse on A3 is high.
    channel_4_start = TIMER2_BASE->CCR4;                 //Record the start time of the pulse.
    TIMER2_BASE->CCER |= TIMER_CCER_CC4P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 4 input pulse on A3 is low.
    channel_4 = TIMER2_BASE->CCR4 - channel_4_start;     //Calculate the total pulse time.
    if (channel_4 < 0)channel_4 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC4P;               //Change the input capture mode to the rising edge of the pulse.
  }
}

void handler_channel_5(void) {                           //This function is called when channel 5 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 6) {                      //If the receiver channel 5 input pulse on A6 is high.
    channel_5_start = TIMER3_BASE->CCR1;                 //Record the start time of the pulse.
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 5 input pulse on A6 is low.
    channel_5 = TIMER3_BASE->CCR1 - channel_5_start;     //Calculate the total pulse time.
    if (channel_5 < 0)channel_5 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;               //Change the input capture mode to the rising edge of the pulse.
  }
}

void handler_channel_6(void) {                           //This function is called when channel 6 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 7) {                      //If the receiver channel 6 input pulse on A7 is high.
    channel_6_start = TIMER3_BASE->CCR2;                 //Record the start time of the pulse.
    TIMER3_BASE->CCER |= TIMER_CCER_CC2P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 6 input pulse on A7 is low.
    channel_6 = TIMER3_BASE->CCR2 - channel_6_start;     //Calculate the total pulse time.
    if (channel_6 < 0)channel_6 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC2P;               //Change the input capture mode to the rising edge of the pulse.
  }
}

//>>> READ_GYRO_DATA
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
  HWire.beginTransmission(gyro_address);                       //Start communication with the gyro.
  HWire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  HWire.endTransmission();                                     //End the transmission.
  HWire.requestFrom(gyro_address, 14);                         //Request 14 bytes from the MPU 6050.
  acc_y = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_x variable.
  acc_x = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_y variable.
  acc_z = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_z variable.
  temperature = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the temperature variable.
  gyro_roll = HWire.read() << 8 | HWire.read();                //Read high and low part of the angular data.
  gyro_pitch = HWire.read() << 8 | HWire.read();               //Read high and low part of the angular data.
  gyro_yaw = HWire.read() << 8 | HWire.read();                 //Read high and low part of the angular data.
  gyro_pitch *= -1;                                            //Invert the direction of the axis.
  gyro_yaw *= -1;                                              //Invert the direction of the axis.

  acc_y -= manual_acc_pitch_cal_value;                         //Subtact the manual accelerometer pitch calibration value.
  acc_x -= manual_acc_roll_cal_value;                          //Subtact the manual accelerometer roll calibration value.
  gyro_roll -= manual_gyro_roll_cal_value - 1 ;                     //Subtact the manual gyro roll calibration value.
  gyro_pitch -= manual_gyro_pitch_cal_value;                   //Subtact the manual gyro pitch calibration value.
  gyro_yaw -= manual_gyro_yaw_cal_value;                       //Subtact the manual gyro yaw calibration value.
}
//>>> TIMER_SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.

void timer_setup(void) {
  Timer2.attachCompare1Interrupt(handler_channel_1);
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCMR2 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;

  Timer3.attachCompare1Interrupt(handler_channel_5);
  Timer3.attachCompare2Interrupt(handler_channel_6);
  TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR = 0;
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE;
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER3_BASE->CCMR2 = 0;
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E;
  TIMER3_BASE->PSC = 71;
  TIMER3_BASE->ARR = 0xFFFF;
  TIMER3_BASE->DCR = 0;

  TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;
  TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 5000;
  TIMER4_BASE->DCR = 0;
  TIMER4_BASE->CCR1 = 1000;

  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR2 = 1000;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
  pinMode(PB6, PWM);
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
}
