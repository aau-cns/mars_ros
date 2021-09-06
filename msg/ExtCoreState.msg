# Stamped External core state estimate message
#
# For feeding state estimate to FCU from external state estimation module.
#
# Notes:
# vectors express quantity in "w" frame
# Quaternions express rotation from "i" to "w"
# FRAME_TYPE express if the "w" frame is UNKNOWN (0), ENU (1) or NED (2)
# QUATERNION_TYPE express if quaternion convention is UNKNOWN (0), HAMILTONIAN (1) or JPL (2)

std_msgs/Header header
geometry_msgs/Vector3 p_wi
geometry_msgs/Vector3 v_wi
geometry_msgs/Quaternion q_wi
geometry_msgs/Vector3 b_w
geometry_msgs/Vector3 b_a
float64[225] cov
uint8 FRAME_TYPE
uint8 QUATERNION_TYPE
