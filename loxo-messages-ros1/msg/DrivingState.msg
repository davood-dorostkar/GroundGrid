# State of the driving system
Header header                   # Header info

FloatCommanded throttle
FloatCommanded brake
FloatCommanded speed
FloatCommanded steeringWheel
FloatCommanded frontWheelsAngle
FloatCommanded rearWheelsAngle  # Angle of rear wheels, if they can be steered
GearCommanded gear              # Gear
bool isBrakeWorking
BoolCommanded isParkingBrake

WheelModeCommanded wheelMode    # Steering mode of wheels
string extraInfo                # String for extra info if needed