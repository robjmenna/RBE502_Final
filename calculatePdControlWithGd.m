function tau = calculatePdControlLawWithGd(xd, x, Kv, Kp, Gd)
a = Kp*eye(6)*(xd(1:6) - x(1:6));
b = Kv*eye(6)*x(7:12);

tau = a - b + Gd;
end