function executeContinuousJointMove()
close all
clear

numberOfPoints = input('Please enter the number of positions that will be taught: ');

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
controlLaw = @(xd,x)calculatePdControlWithGd(xd, x, 10, 10000, Gd.');
[T, X, ~] = moveRobot(t(1), t(lastIndex), q(1,:), robot, trajectory, controlLaw);
plotTrajectory(T, X, trajectory,t, q, robot);
showMotion(T, X(:,1:6), robot);
end

