function [T, X] = moveRobot(t0, tf, q0, robot, trajectory, controlLaw)

%lastIndex = size(t,2);
%numberOfJoints = size(q0,2);
x0 = [q0 zeros(1, 6)];
%xf = [q(lastIndex,:) zeros(1, 6)];
[T, X] = solver(t0, tf, x0, robot, controlLaw, trajectory);


%%
function [T, X] = solver(t0, tf, x0, robot, controlLaw, reference)
    %path = [];
    tol = 1e-6.*ones(1, 12);
    options = odeset('RelTol',1e-6,'AbsTol',tol);
    [T,X] = ode45(@(t,x)solver_(t,x),[t0 tf], x0, options);
    
    function dx = solver_(t,x)
        fprintf([num2str(t) '\n']);
        M = robot.massMatrix(x(1:6).');
        a = x(1:6).';
        b = x(7:12).';
        V = robot.velocityProduct(a,b).';
        G = robot.gravityTorque(x(1:6).').';
        invM = inv(M);
        refTrajectory = reference(x(1:6).',t);
        tau = controlLaw(refTrajectory(1,:).',refTrajectory(2,:).',x);
        %torque = [torque, tau];
        dx = zeros(12,1);
        dx(1:6) = x(7:12);
        dx(7:12) = -invM*V + invM*tau - invM*G;
    end
end
end