clear
clc
noofframes = 5;
theta = sym('theta',[1 14],'real');
thetadot = sym('thetadot',[1 14],'real');
thetaddot = sym('thetaddot',[1 14],'real');
syms theta1 theta2 theta3  real
lambda = sym('lambda',[1,2], 'real')
lambdadot = sym('lambdadot',[1,2], 'real')
lambdaddot = sym('lambdaddot',[1,2], 'real')
syms l g real
j1 = [1.6701e6,  3.6194e5, -1.6755e6; 3.6194e5,  1.2872e7,  1.9375e3; -1.6755e6,  1.9375e3,  1.2143e7];
m1 = 147000;
j2 = [1.4080e4, -3.0649e3,  2.7903e2; -3.0649e3,  3.2480e4,  7.1065e0; 2.7903e2,  7.1065e0,  2.7601e4];
m2 = 16800;
j3 = eye(3);
m3 = 5;
j4 = eye(3);
m4 = 5;
j5 = [ 5000, 0 , 0; 0,20000,0; 0,0,20000];
m5 = 5000;
frames(1) = Frame('Tvec', [0,0,0], 'Fvec', [0,0,0],'Jmatrix', j1, 'framenumber',1, 'rotationaxis', 3, 'rotationvar',theta(1), 'mass', m1,'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0.001],'joint2cm',[0, 0, 0.001]);        
frames(2) = Frame('Tvec', [0,0,0], 'Fvec', [0,0,0],'Jmatrix', j2, 'framenumber',2, 'rotationaxis', 0,  'mass', m2,'Qcoordinates',[lambda(1),lambdadot(1),lambdaddot(1)],'cm2joint',[lambda(1) ,0,0.001],'joint2cm',[0, 0, 0.001]);        
frames(3) = Frame('Tvec', [0,0,0], 'Fvec', [0,0,0],'Jmatrix', j3, 'framenumber',3, 'rotationaxis', 3, 'rotationvar',theta(2), 'mass', m3,'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[0,0,0.001],'joint2cm',[0, 0, 0.001]);        
frames(4) = Frame('Tvec', [0,0,0], 'Fvec', [0,0,0],'Jmatrix', j4, 'framenumber',4, 'rotationaxis', 1, 'rotationvar',theta(3), 'mass', m4,'Qcoordinates',[theta(3),thetadot(3),thetaddot(3)],'cm2joint',[0,0,0.001],'joint2cm',[0, 0, 0.001]);        
frames(5) = Frame('Tvec', [0,0,0], 'Fvec', [0,0,-g*5000],'Jmatrix', j5 , 'framenumber',5, 'rotationaxis', 3, 'rotationvar',theta(4), 'mass', m5,'Qcoordinates',[theta(4),thetadot(4),thetaddot(4)],'cm2joint',[0,0,0.001],'joint2cm',[0, 0, -l]);
B = frames(5).makeB(frames)
Bdot = frames(5).makeBdot(frames)
BT = B'
M = frames(5).makeM(frames)
W = frames(5).makeW(frames)
F = frames(5).makeF(frames)
D = frames(5).makeD(frames)
Q = frames(5).getQs(frames)
Mstar = BT * M * B;
Fstar = BT*F;
Nstar = BT * (M*Bdot + D*M*B);
eq = simplify(Mstar * Q(:,3) + Nstar * Q(:,2) - Fstar == 0)
initCond = zeros(length(frames), 3)
% whos eq
%
% eq1 = isolate(eq(1),thetadot(1))
% eq2 = isolate(eq(2), lambdadot(1))
% eq3 = isolate(eq(3),thetadot(2))
% eq4 = isolate(eq(4),thetadot(4))
% eq5 = isolate(eq(5),thetadot(3))
system = struct( ...
    'Qcoordinates', Q,...
    'initconditions', initCond,...
    'Mstar', Mstar, ...
    'Nstar', Nstar, ...
    'B', B, ...
    'Bt', B',...
    'Bdot', Bdot,...
    'F', F);
overwriteconfig = input('Overwrite Config? y/n: ', 's');
%%overwriteconfig == 'y'
if overwriteconfig == 'y'
    configexport(system, 'Crane5DOF.json')
end