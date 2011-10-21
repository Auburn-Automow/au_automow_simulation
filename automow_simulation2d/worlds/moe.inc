include "hokuyo_r311.inc"

# Moe
define moe position (
  color "blue"
  drive "diff"
  
  size [ 1.14 0.78 0.75 ]
  origin [ 0.2 0.0 0.0 0.0 ]
  
  gui_nose 1
  
  mass 113 # Approximate (250 lbs)
  
  localization "gps"
  localization_origin [ 0 0 0 0 ]
  
#  localization "odom"
#  Odometry Error in x, y, and theta (Normally distributed)
#  odom_error [ 0.1 0.1 0.2 ]
  
  # Add laser to moe
  hokuyo_lrf(pose [ 0.75 0.39 -0.65 45.0 ])
)
