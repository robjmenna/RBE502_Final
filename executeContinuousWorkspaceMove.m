function executeContinuousWorkspaceMove()
close all
clear

% numberOfPoints = input('Please enter the number of positions that will be taught: ');
% 
% q = [[]];
% t = [];
% for i = 1:numberOfPoints
%     trans = input('Please enter the XYZ position of the end-effector in meters: ');
%     rot = input('Please enter the orientation of the end-effector as ZYX euler angles in radians: ');
%     q = [q; trans rot];
%     t = [t input('Please enter the time in seconds when the robot should arrive at this pose: ')];
% end

numberOfPoints = 5;
q = [[]];
q = [q; 0.45 0.25 0.573 pi -pi/2 0];
q = [q; 0.45 0.25 0.2 pi -pi/2 0];
q = [q; 0.45 -0.25 0.2 pi -pi/2 0];
q = [q; 0.45 -0.25 0.573 pi -pi/2 0];
q = [q; 0.45 0.25 0.573 pi -pi/2 0];
t = [0 2 4 6 8];
ik = inverseKinematics;
robot = createRigidTreeModel();
ik.RigidBodyTree = robot;

for i=1:size(q,1)
    temp = trvec2tform(q(i,1:3))*eul2tform(q(i,4:6));
    q(i,:) = ik('body6',temp,[1 1 1 1 1 1], [0 0 0 0 0 0]);
end


lastIndex = size(t,2);
trajectory = planContinuousTrajectory(t, q);

Gd = robot.gravityTorque(q(lastIndex,:));
controlLaw = @(xd,x)calculatePdControlWithGd(xd, x, 10, 10000, Gd.');
[T, X, ~] = moveRobot(t(1), t(5), q(1,:), robot, trajectory, controlLaw);
plotTrajectory(T, X, trajectory, t, q, robot);
showMotion(T, X(:,1:6), robot);
end
