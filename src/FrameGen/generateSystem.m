clear
clc


%{
%%% Crane
theta = sym('theta',[1, 10],'real');
thetadot = sym('thetadot',[1, 10],'real');
thetaddot = sym('thetaddot',[1, 10],'real');
syms cranemass craneL1 craneL3 real;
syms trolleymass trolleyL1 trolleyL1dot trolleyL1ddot trolleyL3 real;
syms wiremass wire1L3 wire2L3 real

noofframes = 5;
for i = 1:noofframes
    frames(i) = Frame('framenumber',i);
end

frames(1).setProperties('rotationaxis', 3, 'rotationvar', theta(1), 'Qcoordinates', [theta(1),thetadot(1),thetaddot(1)], 'cm2joint', [0,0,0],'joint2cm',[craneL1,0,craneL3]);
frames(1).setProperties('mass', cranemass, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);
frames(2).setProperties('rotationaxis', 3, 'rotationvar', 0, 'Qcoordinates', [trolleyL1, trolleyL1dot, trolleyL1ddot], 'cm2joint', [-craneL1,0,trolleyL3],'joint2cm',[trolleyL1,0,0]);
frames(2).setProperties('mass', trolleymass, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);
frames(3).setProperties('rotationaxis', [3,1], 'rotationvar', [theta(3),theta(4)], 'Qcoordinates', [theta(3),thetadot(3),thetaddot(3); theta(4),thetadot(4),thetaddot(4)], 'cm2joint', [0,0,-wire1L3],'joint2cm',[0,0,-wire2L3]);
frames(3).setProperties('mass', wiremass, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);
frames(4).setProperties('rotationaxis', [3,1], 'rotationvar', [theta(5),theta(6)], 'Qcoordinates', [theta(5),thetadot(5),thetaddot(5); theta(6),thetadot(6),thetaddot(6)], 'cm2joint', [0,0,-wire1L3],'joint2cm',[0,0,-wire2L3]);
frames(4).setProperties('mass', wiremass, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);
frames(5).setProperties('rotationaxis', [3,1], 'rotationvar', [theta(7),theta(8)], 'Qcoordinates', [theta(7),thetadot(7),thetaddot(7); theta(8),thetadot(8),thetaddot(8)], 'cm2joint', [0,0,-wire1L3],'joint2cm',[0,0,-wire2L3]);
frames(5).setProperties('mass', wiremass, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);
%}

%%% Double Pendulum
theta = sym('theta',[1, 10],'real');
thetadot = sym('thetad',[1, 10],'real');
thetaddot = sym('thetadd',[1, 10],'real');


g = 9.81;
noofframes = 2;
for i = 1:noofframes
    frames(i) = Frame('framenumber',i);
end

frames(1).setProperties('rotationaxis', 2, 'rotationvar', theta(1), 'Qcoordinates', [theta(1),thetadot(1),thetaddot(1)], 'initconditions', [1,0,0], 'cm2joint', [0,0,0],'joint2cm',[5,0,0], 'mass', 10);
frames(1).setProperties('Jmatrix', [0,0,0;0,0,0; 0,0, 10*1^2/12], 'Fvec', [0,0,-g*10], 'Tvec', [0,0,0]);
frames(2).setProperties('rotationaxis', 2, 'rotationvar', theta(2), 'Qcoordinates', [theta(2),thetadot(2),thetaddot(2)], 'initconditions', [1,0,0], 'cm2joint', [5,0,0],'joint2cm',[5,0,0], 'mass', 10);
frames(2).setProperties('Jmatrix', [0,0,0;0,0,0; 0,0, 10*1^2/12],  'Fvec', [0,0,-g*10], 'Tvec', [0,0,0]);



Q = frames(noofframes).getQs(frames);
B = frames(noofframes).makeB(frames);
Bdot = frames(noofframes).makeBdot(frames);
D = frames(noofframes).makeD(frames)
M = frames(noofframes).makeM(frames)
F = frames(noofframes).makeF(frames)
initCond = frames(noofframes).getInitCond(frames)
Mstar = B' * M * B;
Nstar = B' * (M*Bdot + D*M*B);
% eqs_of_motion = Mstar * Q(:,3) + Nstar*Q(:,2); eqs_of_motion =
% simplify(eqs_of_motion)

%%
X = sym(zeros(noofframes,1));

system = struct( ...
    'Qcoordinates', Q,...
    'initconditions',initCond,... 
    'Mstar', Mstar, ...
    'Nstar', Nstar, ...
    'B', B, ...
    'Bt', B',...
    'F', F);

configexport(system,"testconfig.json")



