function trajectory = planContinuousTrajectory(t, q)
    if (size(t,2) ~= size(q,1))
        error('The number of timing points and joint poses must match.');
    end  
    
    qd(1,:) = zeros(1,size(q,2));
    for i = 2:size(t,2)
        if (or(t(i-1) >= t(i), t(i) < 0 ))
            error('Timing points must be positive and in ascending order.');
        end        

        if i == size(t,2)
            qd(i,:) = zeros(1,size(q,2));  
        else
            v0 = (q(i,:) - q(i-1,:)) ./ (t(i) - t(i-1));
            vf = (q(i+1,:) - q(i,:)) ./ (t(i+1) - t(i));

            for k = 1:size(v0)
                if sign(v0(k)) == sign(vf(k))
                    qd(i,k) = 0.5.*(v0(k) + vf(k));
                else
                    qd(i,k) = 0;
                end
            end
        end
        
        trajectoryList{i-1} = getCubicTrajectory(q(i-1,:), qd(i-1,:), q(i,:), qd(i,:), t(i-1), t(i));
    end
    
    trajectory = @(t__)interpolatePoints(t__);
    
    function d = interpolatePoints(t_)
        isFound = false;
        for j = 1:(size(t,2)-1)
            if (and(t_ >= t(j), t_ <= t(j+1)))
                isFound = true;
                tFunc = trajectoryList{j};
                d = tFunc(t_);
            end
        end
        
        if not(isFound)
            error('Invalid value of t.');
        end
    end
end

