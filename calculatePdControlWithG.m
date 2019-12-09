function tau = calculatePdControlWithG(xd, x, Kv, Kp, robot)
a = Kp*eye(6)*(xd(1:6) - x(1:6));
b = Kv*eye(6)*x(7:12);
G = robot.gravityTorque(x(1:6).');

tau = a - b + G.';
end

