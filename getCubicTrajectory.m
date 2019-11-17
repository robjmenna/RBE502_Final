
function trajectory = getCubicTrajectory(q0, v0, qf, vf, t0, tf)
    b = [q0; v0; qf; vf;];
    M = [ 1 t0 t0^2 t0^3;
    0 1 2*t0 3*t0^2;
    1 tf tf^2 tf^3;
    0 1 2*tf 3*tf^2];

    a = inv(M)*b;    
    trajectory = @(t)getCurrentState(t);
    
    function d = getCurrentState(t)
        d(1) = a(1) + a(2)*t + a(3)*t^2 + a(4)*t^3;
        d(2) = a(2) + 2*a(3)*t + 3*a(4)*t^2;
        d(3) = 2*a(3) + 6*a(4)*t;
    end
end