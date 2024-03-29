function plotTrajectory(T, X, trajectory, t, q, robot)
numberOfJoints = 6;

ik = inverseKinematics;
ik.RigidBodyTree = robot;

temp = trajectory(X(1,1:6), T(1));
path(1,:) = temp(1,1:6);

for i= 1:size(T,1)
    %temp = path(T(i));
    jPos(i,:) = X(i,1:6);
    if i ~= size(T,1)
        temp = trajectory(X(i,1:6), T(i+1));
        path(i+1,:) = temp(1,1:6);
    end
    %jExp(i,:) = temp(1,:);
    error(i,:) = (path(i,:) - X(i,1:6));
    temp = robot.getTransform(X(i,1:6), 'body6');
    xPos(i,:) = temp(1:3,4);    
end

%save('error-g.mat', 'error');
%save('T');

figure
for i = 1:numberOfJoints
    subplot(numberOfJoints,1,i)
    plot(T,jPos(:,i));
    ylabel(['Joint #' num2str(i) ' Position (rad)']);    
end
xlabel('Time (s)');

figure
for i = 1:numberOfJoints
    subplot(numberOfJoints,1,i)
    plot(T,error(:,i));
    ylabel(['Joint #' num2str(i) ' Error (rad)']);    
end
xlabel('Time (s)');
% 
% for i=1:size(q,1)
%     temp = robot.getTransform(q(i,:), 'body6');
%     trvec(i,:) = temp(1:3,4);
% end

figure
for i = 1:3
    subplot(3,1,i)    
    plot(T,xPos(:,i),t,q(:,i),'bX');
    if i == 1    
        ylabel('X-Axis Position (m)');
    elseif i == 2
        ylabel('Y-Axis Position (m)');
    else
        ylabel('Z-Axis Position (m)');
    end 
end
   xlabel('Time (s)');
end

