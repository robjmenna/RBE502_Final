function showMotion(T, X, robot)

dt = T(2) - T(1);
i = 0;
dtMax = 0;
while dtMax < 0.05
    i = i+1;
    dtMax = i*dt; 
end

figure
for j = i:i:size(T,1)
     robot.show(X(j,:));
     axis([-1 1 -1 1 0 1]);
     pause(dtMax);
end

robot.show(X(size(X,1),:));
axis([-1 1 -1 1 0 1]);

end

