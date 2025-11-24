# AudioLevel.msg
# Simulated microphone sensor reading

# Header with timestamp
std_msgs/Header header

# Audio intensity level (0.0 = silence, 1.0 = max)
float32 level

# Frequency band (Hz) - for future use
float32 frequency

# Sensor location on robot
geometry_msgs/Point sensor_position
