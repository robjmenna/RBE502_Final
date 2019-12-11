function showMotion(T, X, robot)

dt = T(2) - T(1);
i = 0;
dtMax = 0;
while dtMax < 0.05
    i = i+1;
    dtMax = i*dt; 
end

%v = VideoWriter('myFile.avi');
fig = figure;
%open(v);
for j = i:i:size(T,1)
     robot.show(X(j,:));
     axis([0 0.8 -0.5 0.5 0 0.8]);
     %v.writeVideo(getframe(fig));
     pause(dtMax);
end

%close(v);

end

