# Ported from ROS 1 by scripts/port_msgs_to_ros2.py:
#   type time -> builtin_interfaces/Time (timestamp)
#   renamed field isInputOnly -> is_input_only
builtin_interfaces/Time timestamp
float64 value
bool is_input_only
