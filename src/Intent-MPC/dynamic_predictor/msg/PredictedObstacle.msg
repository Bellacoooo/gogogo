# Obstacle identifier from tracker
int32 id

geometry_msgs/Point current_position
geometry_msgs/Vector3 current_velocity

# All modal trajectories for this obstacle
dynamic_predictor/PredictedTrajectory[] modes

