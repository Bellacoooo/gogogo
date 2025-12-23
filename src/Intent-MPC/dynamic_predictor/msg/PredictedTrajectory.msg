# Probability weight of this mode
float64 probability

# Mean trajectory for this mode (length = horizon)
geometry_msgs/Point[] mean

# Variance per time step (same length as mean)
# Variance is ordered as var(x), var(y), var(z)
geometry_msgs/Vector3[] variance

