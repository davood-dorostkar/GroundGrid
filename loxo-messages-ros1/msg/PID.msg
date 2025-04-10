# Message to debug the output of a PID
Header header 

float32 kp                   # Current value of the proportional gain
float32 ki                   # Current value of the integral gain
float32 kd                   # Current value of the derivative gain
float32 kf                   # Current value of the feedforward gain

float32 error                # Current error 
float32 output_proportional  # Current P output 
float32 output_integral      # Current I output  
float32 output_derivative    # Current D output 
float32 output_pid           # Current PID output 
float32 output_feedforward   # Current feedfoward output 
float32 output_total         # Current total output 