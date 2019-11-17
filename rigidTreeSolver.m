clear all
close all

robot = createRigidTreeModel();
x0 = [robot.homeConfiguration 0 0 0 0 0 0];
xf = [pi/2 0 pi/4 0 -pi/4 0 0 0 0 0 0 0];
Gd = robot.gravityTorque(xf(1:6));
controlLaw = @(xd,x)calculatePdControl(xd,x,10,100,Gd.');
reference = getCubicTrajectory(x0(1:6),x0(7:12),xf(1:6),xf(7:12),0, 5);
[T,X,torque] = solver(0,5,x0,robot,controlLaw,reference);
robot.show(X(1,1:6));
axis([0 1 0 1 0 1])

for i = 2:size(T,1)
    robot.show(X(i,1:6));
    axis([0 1 0 1 0 1])
    pause(T(i) - T(i-1));
end

%%
function [T, X, torque] = solver(t0, tf, x0, robot, controlLaw, reference)
    torque = [];
    options = odeset('RelTol',1e-6,'AbsTol',[1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]);
    [T,X] = ode45(@(t,x)solver_(t,x),[t0 tf], x0, options);
    
    function dx = solver_(t,x)
        M = robot.massMatrix(x(1:6).');
        a = x(1:6).';
        b = x(7:12).';
        V = robot.velocityProduct(a,b).';
        G = robot.gravityTorque(x(1:6).').';
        invM = inv(M);
        refTrajectory = reference(t);
        xd = [refTrajectory(1,:) refTrajectory(2,:)].';
        tau = controlLaw(xd,x);
        torque = [torque, tau];
        dx = zeros(12,1);
        dx(1:6) = x(7:12);
        dx(7:12) = -invM*V + invM*tau - invM*G;
    end
end

function tau = calculatePdControl(xd, x, Kv, Kp, Gd)
a = Kp*eye(6)*(xd(1:6) - x(1:6));
b = Kv*eye(6)*x(7:12);

tau = a - b + Gd;
end