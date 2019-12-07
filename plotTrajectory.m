function plotTrajectory(T, X, trajectory)
numberOfJoints = 6;
d = [];
for i= 1:size(T,1)
    temp = trajectory(T(i));
    d = [d (temp(1,:) - X(i,1:6)).'];
end

figure
for i = 1:numberOfJoints
subplot(numberOfJoints,1,i)
plot(T,d(i,:));
end
end

