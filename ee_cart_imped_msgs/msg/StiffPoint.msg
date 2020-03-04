Header header
#The pose to achieve in the stiffness directions
geometry_msgs/Pose pose
#Wrench or stiffness for each dimension
geometry_msgs/Wrench wrench_or_stiffness
#The following are True if a force/torque should
#be exerted and False if a stiffness should be used.
bool isForceX
bool isForceY
bool isForceZ
bool isTorqueX
bool isTorqueY
bool isTorqueZ
# There's one extra degree of freedom in our 7dof arm
# We use it to keep kinova's elbow as high in the air as possible
# by applying an "elbow" force to it with the following magnitude
float32 elbow_z_force
#The time from the start of the trajectory that this
#point should be achieved.
duration time_from_start
