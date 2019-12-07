function plotTrajectory(T, trajectory)
d1 = [];
for i= 1:size(T,1)
    temp = trajectory(T(i));
d1 = [d1 temp(:,1)];
end

figure
subplot(3,1,1)
plot(T,d1(1,:));
subplot(3,1,2)
plot(T,d1(2,:));
subplot(3,1,3)
plot(T,d1(3,:));

end

