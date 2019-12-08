function plotTrajectory(T, X, trajectory, t, q, robot)
numberOfJoints = 6;

for i= 1:size(T,1)
    temp = trajectory(T(i));
    jPos(i,:) = X(i,1:6);
    %jExp(i,:) = temp(1,:);
    error(i,:) = (temp(1,:) - X(i,1:6));
    temp = robot.getTransform(X(i,1:6), 'body6');
    xPos(i,:) = temp(1:3,4);
    
end

figure
for i = 1:numberOfJoints
    subplot(numberOfJoints,1,i)
    plot(T,jPos(:,i),t,q(:,i),'bX');
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

for i=1:size(q,1)
    temp = robot.getTransform(q(i,:), 'body6');
    trvec(i,:) = temp(1:3,4);
end

figure
for i = 1:3
    subplot(3,1,i)    
    plot(T,xPos(:,i),t,trvec(:,i),'bX');
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

