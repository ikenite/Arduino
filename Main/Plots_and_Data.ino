// Global variables for printing/plotting
float cmd_theta = 0.0;
int theta_P_comp = 0;
int theta_I_comp = 0;
int theta_D_comp = 0;

// "Serial Monitor" Data selector
// (d1) 
bool data1 = false;

// "Serial Plotter" Plot selector
// (p1) Low-Pass Filtered Accelerometer (x-axis) vs Raw Accelerometer
bool plot1 = false; 
// (p2) Gyro Angle vs Accelerometer Angle vs Complementary Filter Angle (pitch = theta)
bool plot2 = false;
// (p3) Commanded Angle vs Measured Angle (pitch = theta)
bool plot3 = true;
// (p4) Individual Motor commands (0-100)
bool plot4 = false;
// (p5) Pitch (theta) controller: P_compononent vs I_Compoonent vs D_component 
bool plot5 = false;

void plot_labeler()
{
  if (plot1 == true)
  {
  Serial.println("Filtered_Acc_x Raw_Acc_x");
  }
  
  if (plot2 == true)
  { 
  Serial.println("Gyro_theta Acc_theta Complementary_filter_theta");
  }

  if (plot3 == true)
  {
  Serial.println("Commanded_theta Measured_theta");
  }

  if (plot4 == true)
  {
  Serial.println("FL_speed FR_speed BR_speed BL_speed");
  }

  if (plot5 == true)
  {
  Serial.println("P_component I_component D_component");
  }
}

void loop_plotter_printer()
{
  // (d1)
  if (data1 == true)
  {
    Serial.print("delta_t [s] = "); Serial.print(delta_t,9);
    //Serial.print(" || Yaw = "); Serial.print(gyro_psi);
    Serial.print(" || Gyro Pitch = "); Serial.print(gyro_theta);
    Serial.print(" || Acc Pitch = "); Serial.print(acc_theta);
    //Serial.print(" || Gyro Roll = "); Serial.print(gyro_phi);
    //Serial.print(" || Acc Roll = "); Serial.println(acc_phi);
    Serial.print(" || Comp Pitch = "); Serial.println(comp_theta);
  }
  
  // (p1) 
  if (plot1 == true)
  {
    Serial.print(acc_x);
    Serial.print(" ");
    Serial.println(raw_acc_x);
  }

  // (p2) 
  if (plot2 == true)
  {  
    Serial.print(gyro_theta);
    Serial.print(" ");
    Serial.print(raw_acc_theta);
    Serial.print(" ");
    Serial.println(comp_theta);
  }

  // (p3)
  if (plot3 == true)
  {
    Serial.print(cmd_theta);
    Serial.print(" ");
    Serial.println(comp_theta);
  }

  // (p4)
  if (plot4 == true)
  {
    Serial.print(FL_motor_command);
    Serial.print(" ");
    Serial.print(FR_motor_command);
    Serial.print(" ");
    Serial.print(BR_motor_command);
    Serial.print(" ");
    Serial.println(BL_motor_command);
  }

  // (p5)
  if (plot5 == true)
  {
    Serial.print(theta_P_comp);
    Serial.print(" ");
    Serial.print(theta_I_comp);
    Serial.print(" ");
    Serial.println(theta_D_comp);
  }
}
