# Documentation
1. ik.py
- Kinematics(length1,length2,length3,length4)
input: link lengths
Kinematics object for robot arm.
- Kinematics.move_to(target):
target = array(target_x, target_y, target_z)
returns: time_elapsed (in seconds)
Move to a target coordinate. If out of range, it will scale the target vector to a vector in the same direction with magnitude same as its length.
- Kinematics.grip(bool):
bool = 1 (for enable) or 0 (for disable)
returns: 1 (after successfuly run) or 0 (fail)
Grip enable/disable.
- Kinematics.dispense():
returns: 1 (after successfuly run) or 0 (fail)
Dispense card. Need to implement sensor in the future.