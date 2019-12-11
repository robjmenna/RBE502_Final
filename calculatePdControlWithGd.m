function tau = calculatePdControlWithGd(theta_d, dtheta_d, x, Kv, Kp, Gd)
theta= x(1:6,1);
dtheta= x(7:12,1);

e = theta_d-theta; % position error
de = dtheta_d - dtheta; % velocity error
tau= Kp*e + Kv*de + Gd;

end