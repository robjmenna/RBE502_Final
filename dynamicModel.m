clc
clear variables

syms q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
syms dq1(t) dq2(t) dq3(t) dq4(t) dq5(t) dq6(t)
syms d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)
syms m1 m2 m3 mp
syms g pi

%%
d = [0.163 0 0 0.34 0 0.08];
theta = [q1(t) q2(t)+pi/2 q3(t) q4(t) q5(t) q6(t)];
a = [0.03 0.34 0.02 0 0 0];
alpha = [pi/2 0 pi/2 -pi/2 pi/2 0];

T01 = getTransformation(alpha(1), a(1), d(1), theta(1))
T12 = getTransformation(alpha(2), a(2), d(2), theta(2))
T23 = getTransformation(alpha(3), a(3), d(3), theta(3))
T34 = getTransformation(alpha(4), a(4), d(4), theta(4))
T45 = getTransformation(alpha(5), a(5), d(5), theta(5))
T5f = getTransformation(alpha(6), a(6), d(6), theta(6))

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T0f = T05*T5f;

v1 = diff(T01(1:3,4), t)
p1 = T01(3,4)*g*m1;
k1 = 0.5*m1*(v1(1)^2 + v1(2)^2+v1(3)^2);
k1 = subs(k1, diff(q1(t),t), dq1(t));

v2 = diff(T02(1:3,4), t)
p2 = T02(3,4)*g*m2;
k2 = 0.5*m2*(v2(1)^2 + v2(2)^2+v2(3)^2);
k2 = subs(k2, [diff(q1(t),t) diff(q2(t),t)], [dq1(t) dq2(t)]);

v3 = diff(T04(1:3,4), t);
p3 = T04(3,4)*g*m3;
k3 = 0.5*m3*(v3(1)^2 + v3(2)^2+v3(3)^2);
k3 = subs(k3, [diff(q1(t),t) diff(q2(t),t) diff(q3(t),t) diff(q4(t),t)], [dq1(t) dq2(t) dq3(t) dq4(t)]);

vp = diff(T0f(1:3,4), t);
pp = T0f(3,4)*g*mp;
kp = 0.5*mp*(vp(1)^2 + vp(2)^2+vp(3)^2);
kp = subs(kp, [diff(q1(t),t) diff(q2(t),t) diff(q3(t),t) diff(q4(t),t) diff(q5(t),t) diff(q6(t),t)], [dq1(t) dq2(t) dq3(t) dq4(t) dq5(t) dq6(t)]);

L = (k1+k2+k3+kp) - (p1+p2+p3+pp);

dL_dth1 = functionalDerivative(L, dq1(t));
dLdth1_dt = diff(dL_dth1, t);
torque_1 = dLdth1_dt - functionalDerivative(L, q1(t));

dL_dth2 = functionalDerivative(L, dq2(t));
dLdth2_dt = diff(dL_dth2, t);
torque_2 = dLdth2_dt - functionalDerivative(L, q2(t));

dL_dth3 = functionalDerivative(L, dq3(t));
dLdth3_dt = diff(dL_dth3, t);
torque_3 = dLdth3_dt - functionalDerivative(L, q3(t));

dL_dth4 = functionalDerivative(L, dq4(t));
dLdth4_dt = diff(dL_dth4, t);
torque_4 = dLdth4_dt - functionalDerivative(L, q4(t));

dL_dth5 = functionalDerivative(L, dq5(t));
dLdth5_dt = diff(dL_dth5, t);
torque_5 = dLdth5_dt - functionalDerivative(L, q5(t));

dL_dth6 = functionalDerivative(L, dq6(t));
dLdth6_dt = diff(dL_dth6, t);
torque_6 = dLdth6_dt - functionalDerivative(L, q6(t));

torques = [simplify(torque_1); simplify(torque_2); simplify(torque_3); simplify(torque_4); simplify(torque_5); simplify(torque_6)];
torques = simplify(subs(torques, [
    diff(q1(t),t) diff(q2(t),t) diff(q3(t),t) diff(q4(t),t) diff(q5(t),t) diff(q6(t),t)...
    diff(dq1(t),t) diff(dq2(t),t) diff(dq3(t),t) diff(dq4(t),t) diff(dq5(t),t) diff(dq6(t),t)],[
    dq1(t) dq2(t) dq3(t) dq4(t) dq5(t) dq6(t) d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)]));

save('vars/torques.mat', 'torques');

%%
clearvars
syms q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
syms dq1(t) dq2(t) dq3(t) dq4(t) dq5(t) dq6(t)
syms d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)
syms m1 m2 m3 mp
syms g pi

load('vars/torques.mat');
M11 = simplify(expand(torques(1) - subs(torques(1), d2q1(t), 0))/d2q1(t));
M12 = simplify(expand(torques(1) - subs(torques(1), d2q2(t), 0))/d2q2(t));
M13 = simplify(expand(torques(1) - subs(torques(1), d2q3(t), 0))/d2q3(t));
M14 = simplify(expand(torques(1) - subs(torques(1), d2q4(t), 0))/d2q4(t));
M15 = simplify(expand(torques(1) - subs(torques(1), d2q5(t), 0))/d2q5(t));
M16 = simplify(expand(torques(1) - subs(torques(1), d2q6(t), 0))/d2q6(t));

M21 = simplify(expand(torques(2) - subs(torques(2), d2q1(t), 0))/d2q1(t));
M22 = simplify(expand(torques(2) - subs(torques(2), d2q2(t), 0))/d2q2(t));
M23 = simplify(expand(torques(2) - subs(torques(2), d2q3(t), 0))/d2q3(t));
M24 = simplify(expand(torques(2) - subs(torques(2), d2q4(t), 0))/d2q4(t));
M25 = simplify(expand(torques(2) - subs(torques(2), d2q5(t), 0))/d2q5(t));
M26 = simplify(expand(torques(2) - subs(torques(2), d2q6(t), 0))/d2q6(t));

M31 = simplify(expand(torques(3) - subs(torques(3), d2q1(t), 0))/d2q1(t));
M32 = simplify(expand(torques(3) - subs(torques(3), d2q2(t), 0))/d2q2(t));
M33 = simplify(expand(torques(3) - subs(torques(3), d2q3(t), 0))/d2q3(t));
M34 = simplify(expand(torques(3) - subs(torques(3), d2q4(t), 0))/d2q4(t));
M35 = simplify(expand(torques(3) - subs(torques(3), d2q5(t), 0))/d2q5(t));
M36 = simplify(expand(torques(3) - subs(torques(3), d2q6(t), 0))/d2q6(t));

M41 = simplify(expand(torques(4) - subs(torques(4), d2q1(t), 0))/d2q1(t));
M42 = simplify(expand(torques(4) - subs(torques(4), d2q2(t), 0))/d2q2(t));
M43 = simplify(expand(torques(4) - subs(torques(4), d2q3(t), 0))/d2q3(t));
M44 = simplify(expand(torques(4) - subs(torques(4), d2q4(t), 0))/d2q4(t));
M45 = simplify(expand(torques(4) - subs(torques(4), d2q5(t), 0))/d2q5(t));
M46 = simplify(expand(torques(4) - subs(torques(4), d2q6(t), 0))/d2q6(t));

M51 = simplify(expand(torques(5) - subs(torques(5), d2q1(t), 0))/d2q1(t));
M52 = simplify(expand(torques(5) - subs(torques(5), d2q2(t), 0))/d2q2(t));
M53 = simplify(expand(torques(5) - subs(torques(5), d2q3(t), 0))/d2q3(t));
M54 = simplify(expand(torques(5) - subs(torques(5), d2q4(t), 0))/d2q4(t));
M55 = simplify(expand(torques(5) - subs(torques(5), d2q5(t), 0))/d2q5(t));
M56 = simplify(expand(torques(5) - subs(torques(5), d2q6(t), 0))/d2q6(t));

M61 = simplify(expand(torques(6) - subs(torques(6), d2q1(t), 0))/d2q1(t));
M62 = simplify(expand(torques(6) - subs(torques(6), d2q2(t), 0))/d2q2(t));
M63 = simplify(expand(torques(6) - subs(torques(6), d2q3(t), 0))/d2q3(t));
M64 = simplify(expand(torques(6) - subs(torques(6), d2q4(t), 0))/d2q4(t));
M65 = simplify(expand(torques(6) - subs(torques(6), d2q5(t), 0))/d2q5(t));
M66 = simplify(expand(torques(6) - subs(torques(6), d2q6(t), 0))/d2q6(t));

M = [M11 M12 M13 M14 M15 M16;
    M21 M22 M23 M24 M25 M26;
    M31 M32 M33 M34 M35 M36;
    M41 M42 M43 M44 M45 M46;
    M51 M52 M53 M54 M55 M56;
    M61 M62 M63 M64 M65 M66;]

G = subs(torques, [
    d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)...
    dq1(t) dq2(t) dq3(t) dq4(t) dq5(t) dq6(t)],... 
    [0 0 0 0 0 0 0 0 0 0 0 0])

C1 = simplify(expand(torques(1) - M(1,:)*[d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)].' - G(1)));
C2 = simplify(expand(torques(2) - M(2,:)*[d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)].' - G(2)));
C3 = simplify(expand(torques(3) - M(3,:)*[d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)].' - G(3)));
C4 = simplify(expand(torques(4) - M(4,:)*[d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)].' - G(4)));
C5 = simplify(expand(torques(5) - M(5,:)*[d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)].' - G(5)));
C6 = simplify(expand(torques(6) - M(6,:)*[d2q1(t) d2q2(t) d2q3(t) d2q4(t) d2q5(t) d2q6(t)].' - G(6)));
C = [C1;C2;C3;C4;C5;C6]

syms th1 th2 th3 th4 th5 th6
syms dth1 dth2 dth3 dth4 dth5 dth6

M = simplify(subs(M, [q1(t) q2(t) q3(t) q4(t) q5(t) q6(t) g pi], [th1 th2 th3 th4 th5 th6 9.8 3.14]));
G = simplify(subs(G, [q1(t) q2(t) q3(t) q4(t) q5(t) q6(t) g pi], [th1 th2 th3 th4 th5 th6 9.8 3.14]));
C = simplify(subs(C, [q1(t) q2(t) q3(t) q4(t) q5(t) q6(t) dq1(t) dq2(t) dq3(t) dq4(t) dq5(t) dq6(t) g pi],...
    [th1 th2 th3 th4 th5 th6 dth1 dth2 dth3 dth4 dth5 dth6 9.8 3.14]));

matlabFunction(M,'File','getMassMatrix','Vars',{m1,m2,m3,mp, [th1 th2 th3 th4 th5 th6]});
matlabFunction(C,'File','getVelocityMatrix','Vars',{m1,m2,m3,mp,[th1 th2 th3 th4 th5 th6], [dth1 dth2 dth3 dth4 dth5 dth6]});
matlabFunction(G,'File','getGravityMatrix','Vars',{m1,m2,m3,mp,[th1 th2 th3 th4 th5 th6]});
