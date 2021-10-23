#include <Wire.h>
#include <math.h>

const float pi = 3.141592653589793238;

// const float tau = 0.0318; // 5 Hz                                  // tau variable for complementary filter
// const float tau = 0.0796; // 2 Hz 
// const float tau = 0.1592; // 1 Hz 
// const float tau = 0.3183; // 0.5 Hz 
const float tau = 0.6365; // 0.25 Hz 

float p = 0.000 ,q = 0.000, r = 0.000;                                // Data from gyro sensor (angular rates)
// p = roll rate, q = pitch rate, r = yaw rate <-- all in body frame                                      

float raw_acc_x = 0.0, raw_acc_y = 0.0, raw_acc_z = 0.0;              // Unfiltered data from accelerometer sensor
float acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;                          // Filtered data from accelerometer sensor

float p_cal = 0.0, q_cal = 0.0, r_cal = 0.0;                          // Gyro biases (offsets)
float gyro_phi = 0.0, gyro_theta = 0.0, gyro_psi = 0.0;               // Euler Angles based only on gyro data
float raw_acc_theta = 0.0, raw_acc_phi = 0.0;                         // Euler Angles based only on unfiltered accelerometer data
float acc_phi = 0.0, acc_theta = 0.0;                                 // Euler Angles based only on filtered accelerometer data

float raw_comp_theta = 0.0, raw_comp_phi = 0.0;                       // Euler Angles based on unfiltered complementary data (used in discrete low-pass filter)
float comp_theta = 0.0, comp_phi = 0.0;                               // Euler Angles based on filtered complementary data
float t_prev = 0.0, delta_t = 0.0;                                    // Time change used in integrals and derivatives

void IMU_Setup()
{
  Serial.begin(115200);                                              
  
  Wire.begin();                                                        // Start I2C as master
  pinMode(13, OUTPUT);                                                 // Set output 13 (LED) as output
  
  IMU_Register_Setup();                                                // Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  
  digitalWrite(13, HIGH);                                              // Set digital output 13 high to indicate startup

  //Serial.println("Calibrating gyroscope...");
  float sample_size = 1000.0;
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){
  
    Read_IMU();
                                                      
    q_cal += q;                                         
    p_cal += p;                                         
    r_cal += r;                                                                                       
  }
  
  q_cal /= sample_size;                                                // Calculate average offsets
  p_cal /= sample_size;                                           
  r_cal /= sample_size;                                                 

  // Optional data
  /* 
  Serial.print("Average pitch_rate offset = "); Serial.println(q);
  Serial.print("Average roll_rate offset = "); Serial.println(p);
  Serial.print("Average yaw_rate offset = "); Serial.println(r);
  Serial.print("Average acc_pitch offset = "); Serial.println(acc_theta_cal);
  Serial.print("Average acc_roll offset = "); Serial.println(acc_phi_cal);
  */
  
  gyro_phi   = acc_phi;                                                 // Initial orientation                                                       
  gyro_theta = acc_theta;
  gyro_psi   = 0;
  
  digitalWrite(13, LOW);                                                // All done, turn the LED off
  //Serial.println("Gyroscope and accelerometer calibration complete.");
}

/*
// Brokking methods for approximating conversion to vehicle frame

  gyro_phi   += p*delta_t;                                       // Roll (vehicle frame)
  gyro_theta += q*delta_t;                                     // Pitch (vehicle frame)
  gyro_psi   += r*delta_t;                                       // Yaw (vehicle frame)

  gyro_phi -= gyro_theta*sin(r*delta_t*pi/180);
  gyro_theta += gyro_phi*sin(r*delta_t*pi/180);


  float acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
  acc_theta = asin((float)acc_y/acc_total_vector)*180/pi;              // Calculate pitch and roll angles from accelerometer
  acc_phi   = -asin((float)acc_x/acc_total_vector)*180/pi;

*/

void Read_IMU()
{  
  // Measure change in time since last IMU reading
  float t_cur = micros()/1000000.0;                                  
  delta_t = t_cur - t_prev;
  t_prev = t_cur;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Update "prev" values before collecting new data from sensors
  
  float raw_prev_acc_x = raw_acc_x;                                     // Accelerometer unfiltered previous sensor reading u(n-1)
  float raw_prev_acc_y = raw_acc_y;
  float raw_prev_acc_z = raw_acc_z;

  float acc_x_prev = acc_x;                                             // Accelerometer filtered previous sensor reading y(n-1)
  float acc_y_prev = acc_y;
  float acc_z_prev = acc_z; 

  float raw_prev_comp_theta = raw_comp_theta;                           // Unfiltered previous complementary data u(n-1)
  float raw_prev_comp_phi   = raw_comp_phi; 

  float prev_comp_theta = comp_theta;                                   // Filtered previous complementary data y(n-1)
  float prev_comp_phi   = comp_phi; 
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C transmitting/recieving
                                                                       
  Wire.beginTransmission(0x68);                                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                                     // Send the requested starting register
  Wire.endTransmission();                                               // End the transmission
  Wire.requestFrom(0x68,14);                                            // Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                         // Wait until all the bytes are received
  raw_acc_x = Wire.read()<<8|Wire.read();                               // Add the low and high byte to the acc_x variable
  raw_acc_y = Wire.read()<<8|Wire.read();                               // Add the low and high byte to the acc_y variable
  raw_acc_z = Wire.read()<<8|Wire.read();                               // Add the low and high byte to the acc_z variable
  int temperature = Wire.read()<<8|Wire.read();                         // Add the low and high byte to the temperature variable
  p = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_x variable
  q = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_y variable
  r = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_z variable

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Convert and integrate gyro data, filter accelerometer data
  
  p -= p_cal;                                                           //Subtract offset calibration value
  q -= q_cal;                                                                                                    
  r -= r_cal;  

  q /= 65.5;                                                            // Convert gyro raw data to deg/s
  p /= 65.5;                                            
  r /= 65.5;

  // ^^These values are all in the body-frame
  // --> Convert to vehicle frame with transformation matrix
/*
  float c_phi   = cos(gyro_phi*pi/180.0);
  float s_phi   = sin(gyro_phi*pi/180.0);
  float c_theta = cos(gyro_theta*pi/180.0);
  float s_theta = sin(gyro_theta*pi/180.0);
  float c_psi   = cos(gyro_psi*pi/180.0);
  float s_psi   = sin(gyro_psi*pi/180.0);

  // gyro_roll_rate
  float phi_dot =  p + q*s_phi*s_theta/c_theta + r*c_phi*s_theta/c_theta;

  // gyro_pitch_rate
  float theta_dot =  q*c_phi - r*s_phi;

  // gyro_yaw_rate
  float psi_dot = q*s_phi/c_theta + r*c_phi/c_theta;                
*/
                                                                  
  gyro_phi   += p*delta_t; // Roll                                      // Integrate angular rates from gyro to get Euler angles   
  gyro_theta += q*delta_t; // Pitch 
  gyro_psi   += r*delta_t; // Yaw 

  acc_x = acc_lpf(acc_x_prev, raw_prev_acc_x);                          // Low pass filter accelerometer data
  acc_y = acc_lpf(acc_y_prev, raw_prev_acc_y);
  acc_z = acc_lpf(acc_z_prev, raw_prev_acc_z);

  acc_theta = atan2(-acc_x, sqrt(acc_y*acc_y + acc_z*acc_z))*180/pi;    // Calculate pitch and roll angles from accelerometer
  acc_phi   = atan2(acc_y, sqrt(acc_x*acc_x + acc_z*acc_z))*180/pi;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Complementary Filter for gyro and (raw) accelerometer data
  
  raw_acc_theta = atan2(-raw_acc_x, sqrt(raw_acc_y*raw_acc_y + raw_acc_z*raw_acc_z))*180/pi;
  raw_acc_phi   = atan2( raw_acc_y, sqrt(raw_acc_x*raw_acc_x + raw_acc_z*raw_acc_z))*180/pi;
  raw_comp_theta = tau*q + raw_acc_theta;
  raw_comp_phi = tau*p + raw_acc_phi;
  comp_theta = comp_lpf(prev_comp_theta, raw_prev_comp_theta);
  comp_phi   = comp_lpf(prev_comp_phi,   raw_prev_comp_phi);
  
}

void IMU_Register_Setup()
{
  //Serial.println("Configuring IMU registers...");
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


float acc_lpf(float y_prev, float u_prev)
{
// MATLAB Low-Pass Filter Design
//   y(n) = filtered current sensor reading
//   y(n-1) = filtered previous sensor reading
//   u(n-1) = unfiltered previous sensor reading

   //return 0.7304*y_prev + 0.2696*u_prev; // 5 Hz cutoff frequency
   //return 0.8819*y_prev + 0.1181*u_prev; // 2 Hz
     return 0.9391*y_prev + 0.0609*u_prev; // 1 Hz
}

float comp_lpf(float y_prev, float u_prev)
{
// MATLAB Low-Pass Filter Design
//   y(n) = filtered current sensor reading
//   y(n-1) = filtered previous sensor reading
//   u(n-1) = unfiltered previous sensor reading

   //return 0.7304*y_prev + 0.2696*u_prev; // 5 Hz cutoff frequency
   //return 0.8819*y_prev + 0.1181*u_prev; // 2 Hz
   //return 0.9391*y_prev + 0.0609*u_prev; // 1 Hz
   //return 0.9691*y_prev + 0.03093*u_prev; // 0.5 Hz
   return 0.9844*y_prev + 0.01559*u_prev; // 0.25 Hz
}
