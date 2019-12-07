function moveRobot(t, q, robot, trajectory, controlLaw)

lastIndex = size(t,2);
numberOfJoints = size(q,2);
x0 = [q(1,:) zeros(1, numberOfJoints)];
xf = [q(lastIndex,:) zeros(1, numberOfJoints)];
[T, X, torque] = solver(t(1), t(lastIndex), x0, robot, controlLaw, trajectory);
% robot.show(X(1,1:6));
% axis([0 1 0 1 0 1]);

% tim = timer();
% tim.ExecutionMode = 'fixedRate';
% tim.TimerFcn = @(x,y)foo(x,y);
% tim.Period = T(2) - T(1);
% index = 0;
% start(tim);
% 
% function foo(x,y)
%     index = index + 1;
%     if (index == size(T))
%         x.Stop
%     end
%     
%     robot.show(X(index,1:6));
%     axis([-1 1 -1 1 0 1]);    
% end

%%
function [T, X, torque] = solver(t0, tf, x0, robot, controlLaw, reference)
    torque = [];
    tol = 1e-6.*ones(1, numberOfJoints*2);
    options = odeset('RelTol',1e-6,'AbsTol',tol);
    [T,X] = ode45(@(t,x)solver_(t,x),[t0 tf], x0, options);
    
    function dx = solver_(t,x)
        M = robot.massMatrix(x(1:numberOfJoints).');
        a = x(1:numberOfJoints).';
        b = x((numberOfJoints+1):(numberOfJoints*2)).';
        V = robot.velocityProduct(a,b).';
        G = robot.gravityTorque(x(1:numberOfJoints).').';
        invM = inv(M);
        refTrajectory = reference(t);
        xd = [refTrajectory(1,:) refTrajectory(2,:)].';
        tau = controlLaw(xd,x);
        torque = [torque, tau];
        dx = zeros(numberOfJoints*2,1);
        dx(1:numberOfJoints) = x((numberOfJoints+1):(numberOfJoints*2));
        dx((numberOfJoints+1):(numberOfJoints*2)) = -invM*V + invM*tau - invM*G;
    end
end
end