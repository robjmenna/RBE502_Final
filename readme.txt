This project contains two top level functions. 
The first is executeContinuousWorkspaceMove. This command will allow the 
user to enter position and orientation waypoints, and then produce a figure 
that traces the motion defined by feeding the waypoints to the trajectory 
generator. Once the function starts, the user will be prompted for input in
the MATLAB command window. An example of this function in use is provided below.

>> executeContinuousWorkspaceMove
Please enter the number of positions that will be taught: 5
Please enter the XYZ position of the end-effector in meters: [0.45 0.25 0.573]
Please enter the orientation of the end-effector as ZYX euler angles in radians: [pi -pi/2 0]
Please enter the time in seconds when the robot should arrive at this pose: 0
Please enter the XYZ position of the end-effector in meters: [0.45 0.25 0.2]
Please enter the orientation of the end-effector as ZYX euler angles in radians: [pi -pi/2 0]
Please enter the time in seconds when the robot should arrive at this pose: 1
Please enter the XYZ position of the end-effector in meters: [0.45 -0.25 0.2]
Please enter the orientation of the end-effector as ZYX euler angles in radians: [pi -pi/2 0]
Please enter the time in seconds when the robot should arrive at this pose: 2
Please enter the XYZ position of the end-effector in meters: [0.45 -0.25 0.573]
Please enter the orientation of the end-effector as ZYX euler angles in radians: [pi -pi/2 0]
Please enter the time in seconds when the robot should arrive at this pose: 3
Please enter the XYZ position of the end-effector in meters: [0.45 0.25 0.573]
Please enter the orientation of the end-effector as ZYX euler angles in radians: [pi -pi/2 0]
Please enter the time in seconds when the robot should arrive at this pose: 4

The second function is executeContinuousJointMove. This function is similar 
to the first, except the operator will be prompted for the joint positions
instead of workspace position and orientation. An example of the function in 
action is provided below.

>> executeContinuousJointMove
Please enter the number of positions that will be taught: 3
Please enter the joint positions in radians for position #1: [0 0 0 0 0 0]
Please enter the time in seconds when the robot should arrive at this pose: 0
Please enter the joint positions in radians for position #2: [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2]
Please enter the time in seconds when the robot should arrive at this pose: 1
Please enter the joint positions in radians for position #3: [0 0 0 0 0 0]
Please enter the time in seconds when the robot should arrive at this pose: 2