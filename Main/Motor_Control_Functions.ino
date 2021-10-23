#include <Servo.h>

// Global variables used for printing/plotting
int FL_motor_command = 0, FR_motor_command = 0, BL_motor_command = 0, BR_motor_command = 0;

Servo FL, FR, BR, BL; // Front-left, Front-right, Back-right, Back-left
#define FL_PIN 5
#define FR_PIN 2
#define BR_PIN 3
#define BL_PIN 4

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
int DELAY = 1000;

void motor_setup()
{
  FL.attach(FL_PIN);
  FR.attach(FR_PIN);
  BL.attach(BL_PIN);
  BR.attach(BR_PIN);
  Serial.begin(115200);
  ESC_cal();
}

void serial_motor_control()
{
  if (Serial.available() > 0)
  {
    int SPEED = Serial.parseInt();
    Motors(SPEED, SPEED, SPEED, SPEED);
    Serial.print("\n");
    Serial.println("Motor speed:"); Serial.print("  "); Serial.print(FL_motor_command); Serial.print("%");      
  }
}

void motor_test()
{
  // Run motors at very low speed
  int test_speed = 10;
  int test_delay = 1000;
  Motors(test_speed, 0, 0, 0);
  delay(test_delay);
  Motors(0, test_speed, 0, 0);
  delay(test_delay);
  Motors(0, 0, test_speed, 0);
  delay(test_delay);
  Motors(0, 0, 0, test_speed);
  delay(test_delay);
  Motors(test_speed, test_speed, 0, 0);
  delay(test_delay);
  Motors(0, 0, test_speed, test_speed);
  delay(test_delay);
  Motors(0, 0, 0, 0);
  delay(500);
  Motors(test_speed, 0, 0, test_speed);
  delay(test_delay);
  Motors(0, test_speed, test_speed, 0);
  delay(test_delay);
  Motors(0, 0, 0, 0);
}

// Calibration for RC Electronic Speed Controllers (ESCs)
void ESC_cal()
{
  FL.writeMicroseconds(MAX_SIGNAL);
  FR.writeMicroseconds(MAX_SIGNAL);
  BL.writeMicroseconds(MAX_SIGNAL);
  BR.writeMicroseconds(MAX_SIGNAL);

  pinMode(6, OUTPUT); 
  digitalWrite(6, HIGH); 
  delay(5000);

  // Send min output
  FL.writeMicroseconds(MIN_SIGNAL);
  FR.writeMicroseconds(MIN_SIGNAL);
  BL.writeMicroseconds(MIN_SIGNAL);
  BR.writeMicroseconds(MIN_SIGNAL);
  delay(8000);
}

void Motors(int FL_speed, int FR_speed, int BR_speed, int BL_speed)
{

  // Check that all inputs are valid motor commands
  if (FL_speed > 100)
  {
    FL_speed = 100;
  }
  else if(FL_speed < 0)
  {
    FL_speed = 0;
  }

  if (FR_speed > 100)
  {
    FR_speed = 100;
  }
  else if(FR_speed < 0)
  {
    FR_speed = 0;
  }

  if (BR_speed > 100)
  {
    BR_speed = 100;
  }
  else if(BR_speed < 0)
  {
    BR_speed = 0;
  }

  if (BL_speed > 100)
  {
    BL_speed = 100;
  }
  else if(BL_speed < 0)
  {
    BL_speed = 0;
  }

  // ESCs take commands with periods of 1000 to 2000 microseconds
  FL.writeMicroseconds(1000 + FL_speed*10);
  FR.writeMicroseconds(1000 + FR_speed*10);
  BL.writeMicroseconds(1000 + BL_speed*10);
  BR.writeMicroseconds(1000 + BR_speed*10);

  // Save commands to global variables for printing/plotting
  FL_motor_command = FL_speed;
  FR_motor_command = FR_speed;
  BL_motor_command = BL_speed;
  BR_motor_command = BR_speed;
}
