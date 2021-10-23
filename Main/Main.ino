// TO DO
  // 1. Fix coordinate system transformation (body frame to vehicle frame)
  // 2. Account for gyro data roll-over
  // 3. Implement RC receiver PWM interpreter for commanded throttle, pitch, roll, and yaw

int throttle = 30;

void setup() 
{
  motor_setup();
  IMU_Setup();
  plot_labeler();

  //motor_test();
}

void loop() 
{
  Read_IMU();
  theta_control(0);
  loop_plotter_printer();

  //serial_motor_control();
}
