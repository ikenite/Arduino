float theta_error_integrated = 0.0;
float theta_error = 0.0;
float prev_theta_error_derivative[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float prev_theta_error = 0.0;

// I_clamp tuning
//float P_theta = 0.0, I_theta = .150, D_theta = 0.00;
// Max value for theta_error_integrated
float I_clamp = 20.0;

// Tuned P controller
//float P_theta = 0.35, I_theta = 0.0, D_theta = 0.0;

// Tuned PI controller
float P_theta = 0.2, I_theta = 0.15, D_theta = 0.0;

// Tuned PID Controller
// float P_theta = 1.2, I_theta = .1, D_theta = 0.001;

void theta_control(float commanded_theta)
{ 
  cmd_theta = commanded_theta;

  prev_theta_error = theta_error;
  theta_error = commanded_theta - comp_theta;   // Calculate difference between commanded and measured theta
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // D_THETA AVERAGING
  for (int i = 9; i > 0; i--)                                               // shift prev_theta_error_derivative data down array by one
  {
    prev_theta_error_derivative[i] = prev_theta_error_derivative[i-1];
  }
  
  prev_theta_error_derivative[0] = (theta_error - prev_theta_error)/delta_t;// add new data to start of array
  
  float sum = 0.0;
  for(int i = 0; i < 10; i++)                                               // Average data to smooth over time
  {
    sum += prev_theta_error_derivative[i];
  }
  float theta_error_derivative_avg = sum/10.0;
  theta_D_comp = D_theta*theta_error_derivative_avg;
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Theta PID Controller

  // Only engage P-component if error is greater than threshold
  if (abs(theta_error) > 3)
  {
    theta_P_comp = P_theta*theta_error;
  }
  else
  {
    theta_P_comp = 0;
  }

  // Hold integrator if pitch is stable
  if (abs(theta_error) < 0.1 && theta_error_derivative_avg < 30.0)
  {
    theta_error_integrated = theta_error_integrated;
  }
  // Reset integrator if angular rate is high and threshold is passed
  else if (abs(theta_error) < 0.01 && theta_error_derivative_avg > 1.0)
  {
    theta_error_integrated = 0;
  }
  else
  {
    theta_error_integrated += theta_error*delta_t;
  }
  
  theta_I_comp = I_theta*theta_error_integrated;
  
  // Clamping method for pitch error integrator
  if (theta_I_comp > I_clamp)
  {
    theta_error_integrated = I_clamp/I_theta;
  }
  else if (theta_I_comp < -I_clamp)
  {
    theta_error_integrated = -I_clamp/I_theta;
  }

  int theta_PID = theta_P_comp  + theta_I_comp - theta_D_comp;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Motor commands

  int Front_motor_command = throttle - theta_PID;
  int Back_motor_command  = throttle + theta_PID;

  Motors(Front_motor_command, Front_motor_command, Back_motor_command, Back_motor_command); 

}
