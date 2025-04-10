Header header
float32 steering_angle      # desired steering angle (radians)
float32 speed               # desired forward speed (m/s)
float32 acceleration        # desired deceleration (m/s2) to help de feedforward controller
float32 brake               # desired brake force (0-1)
bool is_switch_safe         # true if it's possible to switch to autonomous mode safely
bool is_autonomous_engaged  # true the vehicle accepted the request to go in autonomous mode