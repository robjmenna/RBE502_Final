%% Homework #4
%% Problem #1
clc
clear all;
close all;

% initial condition - Format:[theta1,theta2,dtheta1,dtheta2]
x0= [0,0,-pi,-pi/3,0,0]; %You can change the initial condition here.
tf=2;
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);

[T,X,~] = ode45PlanarArmPIDSolver(tf,x0,options);

figure('Name','Theta_1 Error under PID Control');
plot(T, -X(:,3),'r-');
title('Theta_1 Error under PID Control');
hold on
plot(T, zeros(size(T,1),1),'b--');
figure('Name','Theta_2 Error under PID Control');
plot(T, -X(:,4),'r-');
title('Theta_2 Error under PID Control');
hold on
plot(T, zeros(size(T,1)),'b--');

%% Problem #2
% This plot can be recreated by using the code in the PlanarArm.m example,
% but modifying the trajectory component to represent equations 5.7, 5.8,
% and 5.9.

clc
clear all;
close all;
syms t

I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

% initial condition - Format:[theta1,theta2,dtheta1,dtheta2]
x0= [0,0,0,0]; %You can change the initial condition here.
tf=10;

% the options for ode - Optional!
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
h = 1 - exp(-2*t^3);
b1 = pi/4;
c1 = pi/9;
b2 = pi/3;
c2 = pi/6;

qd = [b1*h+c1*h*sin(t*4); b2*h+c2*h*sin(t*3)];
dqd = diff(qd,t);
ddqd = diff(dqd,t);

getTheta = matlabFunction(qd);
getVelocity = matlabFunction(dqd);
getAcceleration = matlabFunction(ddqd);

[T,X] = ode45PlanarArmSolver(tf,x0,options,...
    @(theta_d, dtheta_d, ddtheta_d, x) planarArmTorqueControl(theta_d, dtheta_d, ddtheta_d, x),...
    @(t)getTheta(t),@(t)getVelocity(t),@(t)getAcceleration(t));

X_d = zeros(size(T,1),2);
error = zeros(size(T,1),2);
for i = 1:size(T)
    X_d(i,1:2) = getTheta(T(i));
    error(i,1:2) = X_d(i,1:2) - X(i,1:2);
end

figure('Name','Positon Error Against Time');
plot(T, error(:,1),'r-');
title('Positon Error Against Time');
hold on
plot(T, error(:,2),'b-');

%% Problem #3

clc
clear all;
close all;
syms t;

% initial condition - Format:[theta1,theta2,dtheta1,dtheta2]
x0= [0,0,0,0]; %You can change the initial condition here.
tf=10;

% the options for ode - Optional!
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
h = 1 - exp(-2*t^3);
b1 = pi/4;
c1 = pi/9;
b2 = pi/3;
c2 = pi/6;

qd = [b1*h+c1*h*sin(t*4); b2*h+c2*h*sin(t*3)];
dqd = diff(qd,t);
ddqd = diff(dqd,t);

getTheta = matlabFunction(qd);
getVelocity = matlabFunction(dqd);
getAcceleration = matlabFunction(ddqd);

[T,X] = ode45PlanarArmSolver(tf,x0,options,...
    @(theta_d, dtheta_d, ddtheta_d, x) planarArmPDWithFeedforwardControl(theta_d, dtheta_d, ddtheta_d, x),...
    @(t)getTheta(t),@(t)getVelocity(t),@(t)getAcceleration(t));

X_d = zeros(size(T,1),2);
error = zeros(size(T,1),2);
for i = 1:size(T)
    X_d(i,1:2) = getTheta(T(i));
    error(i,1:2) = X_d(i,1:2) - X(i,1:2);
end

figure('Name','Positon Error Against Time');
plot(T, error(:,1),'r-');
title('Positon Error Against Time');
hold on
plot(T, error(:,2),'b-');

%% Problem #4
clc
clear all;
close all;

w=0.2;
tf=10;
x0= [-0.5,0.2,0.1,0.1];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);

[T,X, torque] = ode45PlanarArmSolver(tf,x0,options,...
    @(theta_d, dtheta_d, ddtheta_d, x) planarArmFeedforwardControl(theta_d, dtheta_d, ddtheta_d, x),...
    @(t)[w;sin(2*t)],@(t)[0; 2*cos(2*t)],@(t)[0; -4*sin(2*t)]);

figure('Name','Theta_1 under Feedforward Control');
plot(T, X(:,1),'r-');
title('Theta_1 under Feedforward Control');
hold on
plot(T, w*ones(size(T,1),1),'b-');
figure('Name','Theta_2 under Feedforward Control');
plot(T, X(:,2),'r--');
title('Theta_2 under Feedforward Control');
hold on
plot(T, sin(2*T),'b-');
hold on

figure('Name','Input_ Feedforward Control');
plot(T, torque(1,1:size(T,1)),'-' );
title('Input under Feedforward Control');
hold on
plot(T, torque(2,1:size(T,1)),'r--');
hold on

%%%
% From the plots it can be observed that this control implementation cannot
% follow the desired trajectory. Theta 2 fails to follow the sine wave
% accurately, and theta 1 exhibits a large overshoot and is unable to
% settle in the alotted time.

%% Problem #5

%%
% Part a)
clc
clear all;
close all;

t0 = 0;
tf = 15;
x0 = [inverseKinematics3DOF(0,0.2,0.15),0,0,0];
xf = [inverseKinematics3DOF(0.2,0,0.2),0,0,0];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4,1e-4,1e-4]);

[T,X, torque] = ode453DOFSolver(tf,x0,options,...
    @(theta_d, dtheta_d, ddtheta_d, x) threeDOFTorqueControl(theta_d, dtheta_d, ddtheta_d, x),...
    @(t)get3DOFPosition(x0,xf,t0,10,t),...
    @(t)get3DOFVelocity(x0,xf,t0,10,t),...
    @(t)get3DOFAcceleration(x0,xf,t0,10,t));

a = 0.3;
b = 0.1;
c = b;
syms q1 q2 q3;
T01 = DH(0,0,a,0);
T12 = DH(-pi/2,0,0,q1);
T23 = DH(-pi/2,0,b+q2,0);
T3f = DH(0,0,c+q3,0);
H = T01*T12*T23*T3f;
forwardKinematics = matlabFunction(H(1:3,4));
xd = zeros(size(T,1),3);
for i = 1:size(T,1)
    xd(i,1:3) = get3DOFPosition(x0,xf,t0,10,T(i,1));
    xd(i,1:3) = forwardKinematics(xd(i,1),xd(i,2),xd(i,3));
end
plotResults(torque,T,X,xd(:,1),xd(:,2),xd(:,3),'Torque Control');
%plotStickModel(X,'trajectory_with_torque_control');

%%
% Part b)

close all

[T,X, torque] = ode453DOFSolver(tf,x0,options,...
    @(theta_d, dtheta_d, ddtheta_d, x) threeDOFPDWithFeedforwardControl(theta_d, dtheta_d, ddtheta_d, x),...
    @(t)get3DOFPosition(x0,xf,t0,10,t),...
    @(t)get3DOFVelocity(x0,xf,t0,10,t),...
    @(t)get3DOFAcceleration(x0,xf,t0,10,t));

xd = zeros(size(T,1),3);
for i = 1:size(T,1)
    xd(i,1:3) = get3DOFPosition(x0,xf,t0,10,T(i,1));
    xd(i,1:3) = forwardKinematics(xd(i,1),xd(i,2),xd(i,3));
end
plotResults(torque,T,X,xd(:,1),xd(:,2),xd(:,3),'PD Plus Feedforward Control');
%plotStickModel(X,'trajectory_with_PD_plus_feedforward_control');

%%
% Part c)

close all;

% a = 0.3;
% b = 0.1;
% c = b;
% syms q1 q2 q3 dq1 dq2 dq3;
% T01 = DH(0,0,a,0);
% T12 = DH(-pi/2,0,0,q1);
% T23 = DH(-pi/2,0,b+q2,0);
% T3f = DH(0,0,c+q3,0);
% H = T01*T12*T23*T3f;
syms dq1 dq2 dq3
J = [diff(H(1:3,4),q1), diff(H(1:3,4),q2), diff(H(1:3,4),q3)];
invJ = matlabFunction(inv(J)*[dq1;dq2;dq3], 'Vars', [q1 q2 q3 dq1 dq2 dq3]);

t0 = 0;
tf = 15;
x0 = [inverseKinematics3DOF(0,0.2,0.15),0,0,0];
xf = [inverseKinematics3DOF(0.2,0,0.2),0,0,0];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4,1e-4,1e-4]);

[T,X, torque] = ode453DOFSolver(tf,x0,options,...
    @(theta_d, dtheta_d, ddtheta_d, x) threeDOFTorqueControl(theta_d, dtheta_d, ddtheta_d, x),...
    @(t)getStraightLinePosition([0,0.2,0.15,0,0,0],[0.2,0,0.2,0,0,0],t0,10,t),...
    @(t)getStraightLineVelocity([0,0.2,0.15,0,0,0],[0.2,0,0.2,0,0,0],t0,10,invJ,t),...
    @(t)getStraightLineAcceleration([0,0.2,0.15,0,0,0],[0.2,0,0.2,0,0,0],t0,10,t));

xd = zeros(size(T,1),3);
for i = 1:size(T,1)
    xd(i,1:3) = getStraightLinePosition([0,0.2,0.15,0,0,0],[0.2,0,0.2,0,0,0],t0,10,T(i,1));
    xd(i,1:3) = forwardKinematics(xd(i,1),xd(i,2),xd(i,3));
end
plotResults(torque,T,X,xd(:,1),xd(:,2),xd(:,3),'Torque Control With Straight Line Motion');
%plotStickModel(X,'straight_line_trajectory_with_torque_control');

%%
% Part d)

close all

[T,X, torque] = ode453DOFSolver(tf,x0,options,...
    @(theta_d, dtheta_d, ddtheta_d, x) threeDOFPDWithFeedforwardControl(theta_d, dtheta_d, ddtheta_d, x),...
    @(t)getStraightLinePosition([0,0.2,0.15,0,0,0],[0.2,0,0.2,0,0,0],t0,10,t),...
    @(t)getStraightLineVelocity([0,0.2,0.15,0,0,0],[0.2,0,0.2,0,0,0],t0,10,invJ,t),...
    @(t)getStraightLineAcceleration([0,0.2,0.15,0,0,0],[0.2,0,0.2,0,0,0],t0,10,t));

xd = zeros(size(T,1),3);
for i = 1:size(T,1)
    xd(i,1:3) = getStraightLinePosition([0,0.2,0.15,0,0,0],[0.2,0,0.2,0,0,0],t0,10,T(i,1));
    xd(i,1:3) = forwardKinematics(xd(i,1),xd(i,2),xd(i,3));
end
plotResults(torque,T,X,xd(:,1),xd(:,2),xd(:,3),'PD Plus Feedforward With Straight Line Motion');
%plotStickModel(X,'straight_line_trajectory_with_PD_plus_feedforward_control');

%% Functions
function plotStickModel(X, fileName)
figure('Name', ['Robot Animation']);
a = 0.3;
b = 0.1;
c = b;

increment = floor(size(X,1)/80);
myVideo = VideoWriter(fileName);
myVideo.FrameRate = 10;
open(myVideo);

for q = 1:increment:size(X,1)
    T01 = DH(0,0,a,0);
    T12 = DH(-pi/2,0,0,X(q,1));
    T23 = DH(-pi/2,0,b+X(q,2),0);
    T3f = DH(0,0,c+X(q,3),0);
    
    T02 = T01*T12;
    T03 = T01*T12*T23;
    T0f = T01*T12*T23*T3f;
    
    plot3([0 T01(1,4) T02(1,4) T03(1,4) T0f(1,4)],...
        [0 T01(2,4) T02(2,4) T03(2,4) T0f(2,4)],...
        [0 T01(3,4) T02(3,4) T03(3,4) T0f(3,4)]);
    axis([0 0.5 0 0.5 0 0.5]);    
    pause(0.05);
    frame = getframe(gcf);
    writeVideo(myVideo, frame);
end
close(myVideo);
end

function plotResults(torque, T, X, xd, yd, zd, controlLaw)

    figure('Name', ['Joint Position Under ', controlLaw]);    
    subplot(3,1,1)    
    plot(T, X(:,1),'r-');
    title(['Joint Position Under ', controlLaw]);
    ylabel('Joint #1 (rad)');
    subplot(3,1,2)
    plot(T, X(:,2),'r-');
    ylabel('Joint #2 (m)');
    subplot(3,1,3)
    plot(T, X(:,3),'r-');
    ylabel('Joint #3 (m)');
    xlabel('Time (s)');
    hold on;
    
    figure('Name',['Joint Velocity Under ', controlLaw]);
    subplot(3,1,1)
    plot(T, X(:,4),'r-');
    title(['Joint Velocity Under ', controlLaw]);
    ylabel('Joint #1 (rad/s)');
    subplot(3,1,2)
    plot(T, X(:,5),'r-');
    ylabel('Joint #2 (m/s)');
    subplot(3,1,3)
    plot(T, X(:,6),'r-');
    ylabel('Joint #3 (m/s)');
    xlabel('Time (s)');
    hold on;
    
    a = 0.3;
    b = 0.1;
    c = b;
    syms q1 q2 q3;
    T01 = DH(0,0,a,0);
    T12 = DH(-pi/2,0,0,q1);
    T23 = DH(-pi/2,0,b+q2,0);
    T3f = DH(0,0,c+q3,0);
    H = T01*T12*T23*T3f;
    forwardKinematics = matlabFunction(H(1:3,4));
    P = zeros(size(X,1),3);
    for i = 1:size(X,1)
        P(i,1:3) = forwardKinematics(X(i,1),X(i,2),X(i,3));
    end
    
    figure('Name',['End-effector Position Under ', controlLaw]);
    subplot(3,1,1)
    plot(T, P(:,1),'r-');
    title(['End-effector Position Under ', controlLaw]);
    ylabel('X-Axis Position (m)');
    subplot(3,1,2)
    plot(T, P(:,2),'r-');
    ylabel('Y-Axis Position (m)');
    subplot(3,1,3)
    plot(T, P(:,3),'r-');
    ylabel('Z-Axis Position (m)');
    xlabel('Time (s)');
    hold on;    
    
    figure('Name',['End-effector Error Under ', controlLaw]);
    subplot(3,1,1)
    plot(T, xd - P(:,1),'r-');
    ylabel('X-Axis Error (m)');
    title(['End-effector Error Under ', controlLaw]);
    subplot(3,1,2)
    plot(T, yd - P(:,2),'r-');
    ylabel('X-Axis Error (m)');
    subplot(3,1,3)
    plot(T, zd - P(:,3),'r-');
    ylabel('X-Axis Error (m)');
    xlabel('Time (s)');
    hold on;
    
    figure('Name',['Input under ', controlLaw]);
    subplot(3,1,1)
    plot(T, torque(1,1:size(T,1)),'-' );
    title(['Input under ', controlLaw]);
    ylabel('Joint #1 (N*m)');
    subplot(3,1,2)
    plot(T, torque(2,1:size(T,1)),'r--' );
    ylabel('Joint #2 (N)');
    subplot(3,1,3)
    plot(T, torque(3,1:size(T,1)),'g-' );
    ylabel('Joint #3 (N)');
    xlabel('Time (s)');
    hold on;
end

function T = DH(alpha, a, d, theta)
T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta);
     sin(theta) cos(theta)*cos(alpha)  -cos(theta)*sin(alpha) a*sin(theta);
     0          sin(alpha)             cos(alpha)             d;
     0          0                      0                      1];
end

function q = get3DOFPosition(x0,xf,t0,tf,t)
    if (t>tf) 
        q = transpose(xf(1:3));        
    else
        temp(1,:) = calculate_cubic_traj(x0(1), x0(4), xf(1), xf(4),t0,tf, t);
        temp(2,:) = calculate_cubic_traj(x0(2), x0(5), xf(2), xf(5),t0,tf, t);
        temp(3,:) = calculate_cubic_traj(x0(3), x0(6), xf(3), xf(6),t0,tf, t);
        q = temp(:,1);
    end
end

function dq = get3DOFVelocity(x0,xf,t0,tf,t)
if t > tf
    dq = transpose(xf(4:6));
else
    temp(1,:) = calculate_cubic_traj(x0(1), x0(4), xf(1), xf(4),t0,tf, t);
    temp(2,:) = calculate_cubic_traj(x0(2), x0(5), xf(2), xf(5),t0,tf, t);
    temp(3,:) = calculate_cubic_traj(x0(3), x0(6), xf(3), xf(6),t0,tf, t);
    dq = temp(:,2);
end
end

function ddq = get3DOFAcceleration(x0,xf,t0,tf,t)
if t > tf
    ddq = zeros(3,1);
else
    temp(1,:) = calculate_cubic_traj(x0(1), x0(4), xf(1), xf(4),t0,tf, t);
    temp(2,:) = calculate_cubic_traj(x0(2), x0(5), xf(2), xf(5),t0,tf, t);
    temp(3,:) = calculate_cubic_traj(x0(3), x0(6), xf(3), xf(6),t0,tf, t);
    ddq = temp(:,3);
end
end

function q = getStraightLinePosition(x0,xf,t0,tf,t)
    if (t>tf) 
        q = transpose(inverseKinematics3DOF(xf(1),xf(2),xf(3)));        
    else
        temp(1,:) = calculate_cubic_traj(x0(1), x0(4), xf(1), xf(4),t0,tf, t);
        temp(2,:) = calculate_cubic_traj(x0(2), x0(5), xf(2), xf(5),t0,tf, t);
        temp(3,:) = calculate_cubic_traj(x0(3), x0(6), xf(3), xf(6),t0,tf, t);
        q = transpose(inverseKinematics3DOF(temp(1,1),temp(2,1),temp(3,1)));
    end
end

function dq = getStraightLineVelocity(x0,xf,t0,tf,invJ,t)
    if (t>tf) 
        dq = invJ(xf(1),xf(2),xf(3),xf(4),xf(5),xf(6));
    else      
        temp(1,:) = calculate_cubic_traj(x0(1), x0(4), xf(1), xf(4),t0,tf, t);
        temp(2,:) = calculate_cubic_traj(x0(2), x0(5), xf(2), xf(5),t0,tf, t);
        temp(3,:) = calculate_cubic_traj(x0(3), x0(6), xf(3), xf(6),t0,tf, t);
        dq = invJ(temp(1,1),temp(2,1),temp(3,1),temp(1,2),temp(2,2),temp(3,2));
    end
end

function ddq = getStraightLineAcceleration(x0,xf,t0,tf,t)
    ddq = zeros(3,1);
end

function d = calculate_cubic_traj(q0, v0, qf, vf, t0, tf, t)
b = [q0; v0; qf; vf;];
M = [ 1 t0 t0^2 t0^3;
0 1 2*t0 3*t0^2;
1 tf tf^2 tf^3;
0 1 2*tf 3*tf^2];
  
a = inv(M)*b;
d(1) = a(1) + a(2)*t + a(3)*t^2 + a(4)*t^3;
d(2) = a(2) + 2*a(3)*t + 3*a(4)*t^2;
d(3) = 2*a(3) + 6*a(4)*t;
end

function q = inverseKinematics3DOF(x,y,z)
    q = [-atan2(x,y), (x^2 + y^2)^0.5-0.1, 0.2 - z];
end

function [Mmat, Cmat, Gmat] = compute3DOFMatrices(x)
    b = 0.1;
    m1 = 0.05;
    m2 = m1;
    g = 9.8;
    
    Mmat = [m2*(b + x(2))^2 0 0;
            0 m2 0;
            0 0 m2];
    Cmat = [m2*x(5)*(b + x(2)), m2*x(4)*(b + x(2)), 0;
            -m2*x(4)*(b + x(2)), 0, 0;
            0, 0, 0];
    Gmat = [0; 0; -g*m2];
end

function [T,X,torque] = ode45PlanarArmSolver(tf, x0, options, controlLaw, position, velocity, acceleration)
    torque = [];
    [T,X] = ode45(@(t,x)plannarArmODE(t,x),[0 tf],x0,options);
    
    function [dx ] = plannarArmODE(t, x)
        theta_d = position(t);
        dtheta_d = velocity(t);
        ddtheta_d = acceleration(t);


        [Mmat, Cmat, Gmat] = computeArmMatrices(x);
        invM = inv(Mmat);
        invMC = invM*Cmat;
        invMG = invM*Gmat;

        tau = controlLaw(theta_d, dtheta_d, ddtheta_d, x); 
        torque =[torque, tau];
        dx=zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3:4) = -invMC* x(3:4) + invM*tau - invMG; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end    
end

function [T,X,torque] = ode45PlanarArmPIDSolver(tf, x0, options)
    torque = [];
    [T,X] = ode45(@(t,x)plannarArmODE(t,x),[0 tf],x0,options);
    
    function [dx ] = plannarArmODE(t, x)
        theta_d = [0,0];
        dtheta_d = [0,0];

        %[~,~,Gmatd] = computeArmMatrices([0 0 0 0 0 0]);
        [Mmat, Cmat, Gmat] = computeArmMatrices(x);
        invM = inv(Mmat);
        invMC = invM*Cmat;
        %invMG = invM*Gmat;
        %invMGd = invM*Gmatd;

        tau = planarArmPIDControl(x); 
        torque =[torque, tau];
        dx=zeros(6,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3) = -x(5);
        dx(4) = -x(6);
        dx(5:6) = invM*tau - invMC*x(5:6); % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end    
end

function tau = planarArmPIDControl(x)
Kp = 30*eye(2);
Kv = [7,0;0,3];
Ki = [70,0;0,100];

tau = Kp*x(3:4)-Kv*x(5:6)+Ki*x(1:2);
end

function [T,X,torque] = ode453DOFSolver(tf, x0, options, controlLaw, position, velocity, acceleration)
    torque = [];
    [T,X] = ode45(@(t,x)threeDOFODE(t,x),[0 tf],x0,options);
    
    function [dx ] = threeDOFODE(t, x)
        theta_d = position(t);
        dtheta_d = velocity(t);
        ddtheta_d = acceleration(t);


        [Mmat, Cmat, Gmat] = compute3DOFMatrices(x);
        invM = inv(Mmat);
        invMC = invM*Cmat;
        invMG = invM*Gmat;

        tau = controlLaw(theta_d, dtheta_d, ddtheta_d, x); 
        torque =[torque, tau];
        dx=zeros(6,1);
        dx(1) = x(4);
        dx(2) = x(5);
        dx(3) = x(6);
        dx(4:6) = -invMC* x(4:6) + invM*tau - invMG; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end    
end

function tau = planarArmTorqueControl(theta_d, dtheta_d, ddtheta_d, x)
    theta= x(1:2,1);
    dtheta= x(3:4,1);    
    [Mmat, Cmat, Gmat] = computeArmMatrices(x);
    w = [38.7, 0; 0, 118.3];
    Kp=w.^2;
    Kv=2*w;
    e=theta_d-theta; % position error
    de = dtheta_d - dtheta; % velocity error
    tau= Mmat*(Kp*e + Kv*de) + Cmat*dtheta + Mmat*ddtheta_d + Gmat;
end

function tau = threeDOFTorqueControl(theta_d, dtheta_d, ddtheta_d, x)
    theta= x(1:3,1);
    dtheta= x(4:6,1);    
    [Mmat, Cmat, Gmat] = compute3DOFMatrices(x);
    Kp=100*eye(3);
    Kv=100*eye(3);
    e=theta_d-theta; % position error
    de = dtheta_d - dtheta; % velocity error
    tau= Mmat*(Kp*e + Kv*de) + Cmat*dtheta + Mmat*ddtheta_d + Gmat;
end

function tau = threeDOFPDWithFeedforwardControl(theta_d, dtheta_d, ddtheta_d, x)
    theta= x(1:3,1);
    dtheta= x(4:6,1);   
    [Mmatd, Cmatd, Gmatd] = compute3DOFMatrices([transpose(theta_d), transpose(dtheta_d)]);
    Kp=200*eye(3);
    Kv=3*eye(3);

    e=theta_d-theta; % position error
    de = dtheta_d - dtheta; % velocity error
    tau= (Kp*e + Kv*de) + Cmatd*dtheta_d + Mmatd*ddtheta_d + Gmatd;
end

function tau = planarArmPDWithFeedforwardControl(theta_d, dtheta_d, ddtheta_d, x)
    theta= x(1:2,1);
    dtheta= x(3:4,1);    
    [Mmatd, Cmatd, Gmatd] = computeArmMatrices([transpose(theta_d), transpose(dtheta_d)]);
    Kp=[200 0; 0 150];
    Kv=[3 0; 0 3];

    e=theta_d-theta; % position error
    de = dtheta_d - dtheta; % velocity error
    tau= (Kp*e + Kv*de) + Cmatd*dtheta_d + Mmatd*ddtheta_d + Gmatd;
end

function [Mmat, Cmat, Gmat] = computeArmMatrices(x)
    I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
    g = 9.8;
    a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
    b = m2*l1*r2;
    d = I2+ m2*r2^2;
    Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
    Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
    Gmat = [
        (m1*r1+m2*l1)*g*sin(x(1))+m2*r2*g*sin(x(1)+x(2));
        m2*r2*g*sin(x(1)+x(2))];
end

function tau = planarArmFeedforwardControl(theta_d, dtheta_d, ddtheta_d, x)
    [Mmatd, Cmatd, Gmatd] = computeArmMatrices([transpose(theta_d), transpose(dtheta_d)]);
    tau= Cmatd*dtheta_d + Mmatd*ddtheta_d + Gmatd;
end
