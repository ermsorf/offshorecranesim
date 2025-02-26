clear
clc



%%% Crane
theta = sym('theta',[1, 10],'real');
thetad = sym('thetadot',[1, 10],'real');
thetadd = sym('thetaddot',[1, 10],'real');
syms cranemass craneL1 craneL3 real;
syms trolleymass trolleyL1 trolleyL1d trolleyL1dd trolleyL3 real;
syms wiremass wire1L3 wire2L3 real

noofframes = 4;
for i = 1:noofframes
    frames(i) = Frame('framenumber',i);
end
frames(1).setProperties('rotationaxis', 3, 'rotationvar', theta(1), 'Qcoordinates', [theta(1),thetad(1),thetadd(1)],'cm2joint',[0,0,0],'joint2cm',[4.68, 0.61, 4.93])
frames(1).setProperties('mass', 147000, 'Jmatrix', [1.6701e6,  3.6194e5, -1.6755e6; 3.6194e5,  1.2872e7,  1.9375e3; -1.6755e6,  1.9375e3,  1.2143e7])
frames(1).setProperties('Fvec', [0,0,-9.81*147000], 'Tvec', [0,0,10000], 'initconditions', [0,0,0])
frames(2).setProperties('rotationaxis', 0, 'rotationvar', 0, 'Qcoordinates', [trolleyL1, trolleyL1d, trolleyL1dd], 'cm2joint', [0,0,3],'joint2cm', [trolleyL1, 0,0])
frames(2).setProperties('mass', 16800,'Jmatrix', [1.4080e4, -3.0649e3,  2.7903e2; -3.0649e3,  3.2480e4,  7.1065e0; 2.7903e2,  7.1065e0,  2.7601e4])
frames(2).setProperties('Fvec', [0,0,-9.81*16800], 'Tvec', [0,0,0], 'initconditions', [0,0,0])
frames(3).setProperties('rotationaxis', 3, 'rotationvar', theta(2), 'Qcoordinates', [theta(2),thetad(2),thetadd(2)], 'cm2joint', [0,0,-1], 'joint2cm',[0,0,0])
frames(3).setProperties('mass', 5, 'Jmatrix', [41,0,0;0,41,0; 0,0, 0.025])
frames(3).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
frames(4).setProperties('rotationaxis', 1, 'rotationvar', theta(3), 'Qcoordinates', [theta(3),thetad(3),thetadd(3)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,-10])
frames(4).setProperties('mass', 5000, 'Jmatrix', [7500,0,0; 0, 7500,0; 0,0,7500])
frames(4).setProperties('Fvec', [0,0,-9.81*5000], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
%}

%{
%%% Double Pendulum
theta = sym('theta',[1, 10],'real');
thetadot = sym('thetad',[1, 10],'real');
thetaddot = sym('thetadd',[1, 10],'real');

g = 9.81;
noofframes = 2;
for i = 1:noofframes
    frames(i) = Frame('framenumber',i);
end

frames(1).setProperties('rotationaxis', 2, 'rotationvar', theta(1), 'Qcoordinates', [theta(1),thetadot(1),thetaddot(1)], 'initconditions', [0,0,0], 'cm2joint', [0,0,0],'joint2cm',[0.5,0,0], 'mass', 10);
frames(1).setProperties('Jmatrix', [0,0,0;0,0,0; 0,0, 10*1^2/12], 'Fvec', [0,0,-g*10], 'Tvec', [0,0,0]);
frames(2).setProperties('rotationaxis', 2, 'rotationvar', theta(2), 'Qcoordinates', [theta(2),thetadot(2),thetaddot(2)], 'initconditions', [0,0,0], 'cm2joint', [0.5,0,0],'joint2cm',[0.5,0,0], 'mass', 10);
frames(2).setProperties('Jmatrix', [0,0,0;0,0,0; 0,0, 10*1^2/12],  'Fvec', [0,0,-g*10], 'Tvec', [0,0,0]);

%}


Q = frames(noofframes).getQs(frames);
B = frames(noofframes).makeB(frames);
Bdot = frames(noofframes).makeBdot(frames);
D = frames(noofframes).makeD(frames)
M = frames(noofframes).makeM(frames)
F = frames(noofframes).makeF(frames)
initCond = frames(noofframes).getInitCond(frames)
Mstar = B' * M * B;
Nstar = B' * (M*Bdot + D*M*B);
rotations = frames(noofframes).exportRotations(frames)
T = frames(noofframes).getTransformMat(frames)
% Fstar = B' * F;
% eqs_of_motion = Mstar * Q(:,3) + Nstar*Q(:,2); eqs_of_motion =
% simplify(eqs_of_motion)

%%

system = struct( ...
    'Qcoordinates', Q,...
    'initconditions', initCond,... 
    'rotations', rotations,...
    'Mstar', Mstar, ...
    'Nstar', Nstar, ...
    'T', T,...
    'B', B, ...
    'Bt', B',...
    'Bdot', Bdot,...
    'F', F); 

overwriteconfig = input("Overwrite config? y/n: ",'s');

if overwriteconfig == "y"
    configexport(system,"CraneConfig.json")
end



