clear
clc
theta = sym('theta',[1, 5],'real');
thetadot = sym('thetadot',[1, 5],'real');
thetaddot = sym('thetaddot',[1, 5],'real');
syms L1 L2 real

for i = 1:3
    frames(i) = Frame('framenumber',i);
end

frames(1).setProperties('rotationaxis', 3, 'rotationvar', theta(1), 'Qcoordinates', [theta(1),thetadot(1),thetaddot(1)], 'cm2joint', [0,0,0],'joint2cm',[1/2,0,0])
frames(1).setProperties('mass', 2, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);
frames(2).setProperties('rotationaxis', 3, 'rotationvar', theta(2), 'Qcoordinates', [theta(2),thetadot(2),thetaddot(2)], 'cm2joint', [1/2,0,0],'joint2cm',[1/2,0,0])
frames(2).setProperties('mass', 2, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);

Q = getQs(frames(2),frames);
B = makeB(frames(2),frames);
Bdot = makeBdot(frames(2),frames);
D = makeD(frames(2),frames);
M = makeM(frames(2),frames);
Mstar = B' * M * B;             Mstar = simplify(Mstar);
Nstar = B' * (M*Bdot + D*M*B);  Nstar = simplify(Nstar);

eqs_of_motion = Mstar * Q(:,3) + Nstar*Q(:,2); eqs_of_motion = simplify(eqs_of_motion)



