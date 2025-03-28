clc
clear

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



Q = frames(noofframes).getQs(frames);
B = frames(noofframes).makeB(frames)

Bdot = frames(noofframes).makeBdot(frames)
D = simplify(frames(noofframes).makeD(frames))
M = frames(noofframes).makeM(frames)
F = frames(noofframes).makeF(frames)
initCond = frames(noofframes).getInitCond(frames)
Mstar = B' * M * B; Mstar = simplify(Mstar)
Nstar = B' * (M*Bdot + D*M*B); Nstar = simplify(Nstar)
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
    configexport(system,"singlePendulumConfig.json")
end
