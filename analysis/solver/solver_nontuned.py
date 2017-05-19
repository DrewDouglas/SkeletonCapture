# - Load Dynamics Matrices 
	# (read text file as string??)
# Request directory of subject
# - Load Parameters (hand entered OR from JSON)
	# check directory for params.json, else Ask input format
	# Hand entered
		# request each parameter in list
		# Save all as JSON to file in user directory after prompt
# - Request multiple sensors: 
# How many sensors are being used?
# For each one input:
	# - sensor type (Kinect2 (all joints) or Opt Marker Array or IMU or w/e)
	# sensor name for identification
	# - Associated stiffness & damping factors
	# - if Kinect: pull in all joints
		# - what body part is it attached to
		# - transformation matrix in that body part's frame
		# - CSV timeseries of relevant data (pos & detectedflag for Kinect Joints)
	# Assemble Jacobian Matrix
# - ??Flesh Out Initial State of the body??
	# Init velocities are 0?
# - For each timestep (fixed 60Hz in paper)
