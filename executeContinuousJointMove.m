function executeContinuousJointMove()
numberOfPoints = input('Please enterthe number of points that will be taught: ');

q = [[]];
t = [];
for i = 1:numberOfPoints
    q = [q; input(['Please enter the joint positions in radians for position #'...
        num2str(i) ': '])];
    t = [t input('Please enter the time in seconds when the robot should arrive at this pose: ')];
end

lastIndex = size(t,2);
robot = createRigidTreeModel();
trajectory = planContinuousTrajectory(t,q);
Gd = robot.gravityTorque(q(lastIndex,:));
controlLaw = @(xd,x)calculatePdControlLaw(xd, x, 10, 100, Gd.');
moveRobot(t, q, robot, trajectory, controlLaw);
end

