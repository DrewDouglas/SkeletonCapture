import sys
import re
import os
dir_root = sys.argv[1] if(len(sys.argv) >= 2) else None

reg_rob_qstr = r"\[q(\d+)\]"
# def repl_rob_q(m):
# 	n = m.group(1)
# 	print(n)
# 	return r"[q["+str(int(n)-1)+r"]]"

def polish_TRob(t_rob):
	t = 'np.matrix(['+','.join(t_rob)+'])'
	t = re.sub(reg_rob_qstr,r"[q[\g<1>]]",t)
	t = t.replace('{','[').replace('}',']').replace('\n','')
	return t


def polish_Jacobian(count_speeds, jac_rob, joint_mappings,stop_index):
	if(len(jac_rob)!=6):
		raise ValueError("Jacobian is not 6 rows!")
	jmat = []
	for r in range(0,6):
		row = jac_rob[r].strip().strip('{}').split(', ')
		jmat.append(row)
	n_col =len(jmat[0])
	print(jmat)

def trimJacobian(count_speeds,jmat,joint_mappings,stop_index,root_DOFs):
	print("\ttrimming irrelevant dimensions & Building Complete Local Jacobian:")
	jac = []
	for r in range(0,6):
		row = []
		for c_minus in range(root_DOFs,count_speeds):
			c = c_minus+1
			take_col = (c in joint_mappings) and (joint_mappings[c]<stop_index)
			term = (jmat[r][joint_mappings[c]] if take_col else "0")
			row.append(term.strip())
		rowstr = ", ".join(row)
		print("\t"+rowstr)
		jac.append("["+rowstr+"]")
	jacstr = "np.matrix(["+", ".join(jac)+"])"
	print(jacstr)
	return jacstr



# accept a directory
if(dir_root is None):
	print("Please enter the root directory of the model: ")
	dir_root = raw_input()

# read in Mass Matrix & polish it

# Read in Coriolis matrix & polish it

# read in root jacobian and T



count_speeds = raw_input("How many generalized speed DOFs are there in the complete model? ")
count_speeds = int(count_speeds)
root_DOFs = raw_input("How many generalized speed DOFs are there in the model's root? Default:6")
if(root_DOFs==""):
	root_DOFs=6
root_DOFs = int(root_DOFs)
# for each branch
done_bodywide = False

while not done_bodywide:
	dir_branch = raw_input("Enter the directory of a body branch defined relative to the model root, or hit enter to complete: ")
	if(dir_branch==""):
		break

	print("Searching for index_remappings.txt...")
	try:
		fmappings = open(dir_root+"/"+dir_branch + "/index_remappings.txt").read()
	except Exception as e:
		raise FileNotFoundError("Failed to find index remapping file.")
	fmappings = fmappings.strip('<|>Association[]').replace("->",":").split(',')
	# read in joint remappings:
	joint_mappings = {}
	for i in range(0,len(fmappings)):
		cur = fmappings[i].strip()
		m = cur.split(":")
		joint_mappings[int(m[1])] = int(m[0])

	print("\tMappings aquired: (listed Global: Local) \n\t "+str(joint_mappings))

	# check for jacobian fr robotica
  	try:
  		jac_rob = open(dir_root+"/"+dir_branch+"/jacobian.txt").readlines()
  	except Exception as e:
  		raise FileNotFoundError("Failed to find " + dir_branch + "'s jacobian from Robotica!")
	print("Found Jacobian of "+dir_branch)


	# Polish Body Parts List 
	print("Finding Body Parts List in branch "+dir_branch)
	try:
		fBodies = open(dir_root+"/"+dir_branch + "/bodies.txt").read()
	except Exception as e:
		raise FileNotFoundError("Failed to find body parts list.")

	fBodies = fBodies.strip('<|>Association[]').replace("->",":").replace('\"',"").split(',')
	n_parts = len(fBodies)
	bodies = {}
	for i in range(n_parts):
		c = fBodies[i].split(':')
		bodies[c[0].strip()]=int(c[1]);

	print("\tFound "+str(n_parts)+" parts.")
	print(bodies)
	print("\nIterating through body parts: ")
	# For each body part
	for i in range(0,n_parts):
		partName = 
		[i]
		print("Part #"+str(i)+": "+partName)
	  	# check for associated Robotics transformation
	  	try:
	  		t_rob = open(dir_root+"/"+dir_branch+"/t_robotica/"+partName+".txt").readlines()
	  	except Exception as e:
	  		raise FileNotFoundError("Failed to find " + partName + "'s T matrix from Robotica!")

	  	if(len(t_rob)!= 4 and len(t_rob[0].split(',')) != 4):
	  		raise ValueError("Robotica Transformation matrix of "+partName + " not a 4x4 matrix!")
		print("\tFound T of "+partName+":")
		# polish t_rob of part
		t = polish_TRob(t_rob)

		# output canonical t
		filename = dir_root+"/polished/t/"+partName+".txt"
		if not os.path.exists(os.path.dirname(filename)):
			print("making directory for polished outputs")
			os.makedirs(os.path.dirname(filename))

		with open(filename,'w') as dest:
			dest.write(t)

		jac = polish_Jacobian(count_speeds, jac_rob, joint_mappings,bodies[partName])
		
		# output canonical jacobian of part
		filename = dir_root+"/polished/jac_local/"+partName+".txt"
		if not os.path.exists(os.path.dirname(filename)):
		    os.makedirs(os.path.dirname(filename))
		with open(filename, 'w') as dest:
			dest.write(jac)