# request directory of the base body model
# - Load Dynamics Matrices  & jacobians & transformations
# request directory of associated sensor data format
# Reuse Old sensor loadout?
	# if yes:
	# enter number of sensors
	# for each sensor,
		# ask what body part it's on
		# ask for offset in terms of that body part's frame
	# if no:
	# request directory of sensor loadout & Load Sensor Loadout matrices
		# verify count of sensors & set of free variables vs body model?
# Request directory of subject
	# - Load Parameters (hand entered OR from JSON)
		# check directory for params.json, else Ask input format
		# Hand entered
			# request each parameter in list
			# Save all as JSON to file in user directory after prompt
	# - Request multiple sensors: 
		# How many sensors are being used?
		# begin constructing sensor jacobian string???
		# For each one input:
			# - sensor type (Kinect2 (all joints) or Opt Marker Array or IMU or w/e)
			# sensor name for identification
			# - Associated stiffness & damping factors
			# - if Kinect: pull in all joints
				# - what body part is it attached to
				# o_i offset in that body part's frame
				#  CSV timeseries of relevant data (pos & detectedflag for Kinect Joints)
		# Also request constraint stiffness/damping for nullspace pose guessing (restoration to Flying-V-Pose)
		# calculate entire Sigma_inverse & Gamma matrices ((3n+m-6)x(3n+m-6) diagonals)
# request frequency of eval, define h = 1/f
# asteval starts here~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	# Assemble Sensor Jacobian Matrix 
	# 
	# - ??Flesh Out Initial State of the body??
		# Init velocities are 0?
	# - For each timestep (fixed 60Hz in paper)
		# PHASE ONE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		# Eval current M (mxm), C (mx1), Js (3nxm), Ts num_parts*(3x3)
		# JsCross (mx3n) = pseudoinverse of current Js
		# NsPLUS (mxm) =  mxm identity - JsCross * Js
		# Ns ((m-6)xm) = drop top rows from NsPLUS (for root DOF)
		# phi_qtilde ((m-6)x1) = -q (ignore root DOFs)
		# for every sensor:
			# eval T_i (3x3) of local part 
			# xbar_i (3x1) = most recent sensor datapoint
			# phi_x_i (3x1) = xbar_i - T_i * o_i
		# phi ((3n+m-6)x1) = phi_x on top of phi_qtilde
		# J ((3n+m-6)xm) = Js on top of Ns
		# PHASE TWO ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		# A (mxm) = M + J_transpose * Sigma_inverse * J
		# find A_inv (use the magical Lin Alg trick)
		# b (mx1) = M * u - h * C + h^-1*J_transpose * Sigma_inv * Gamma * phi
		# PHASE THREE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		# u_next (mx1) = A_inv * b
		# lambda ((3n+m-6)x1) = Sigma_inv * (Gamma*phi - J*u_next) / h
		# PHASE FOUR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		# torques = J_transpose * lambda
		# q_next = q + h * u_next
		# OUTPUTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		# output current q, u, torques, and frame location/orientation of each part
		# q = q_next; u = u_next

