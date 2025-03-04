clear
clc



%%% Crane rotation
%{
psi = sym('psi',[1, 2],'real');
psid = sym('psid',[1, 2],'real');
psidd = sym('psidd',[1, 2],'real');
%}
% Crane Vars
syms cr crd crdd trolley trolleyd trolleydd real

wiresegments = 3;

% Wire rotation - 3
theta = sym('theta',[1, wiresegments],'real');
thetad = sym('thetad',[1, wiresegments],'real');
thetadd = sym('thetadd',[1, wiresegments],'real');

% Wire Rotation - 1
phi = sym('phi',[1, wiresegments],'real');
phid = sym('phid',[1, wiresegments],'real');
phidd = sym('phidd',[1, wiresegments],'real');

% Wire Lengths
lambda = sym('lambda',[1, wiresegments],'real');
lambdad = sym('lambdad',[1, wiresegments],'real');
lambdadd = sym('lambdadd',[1, wiresegments],'real');



for i = 1:(wiresegments*3 + 3)
    frames(i) = Frame('framenumber',i);
end


frames(1).setProperties('rotationaxis', 3, 'rotationvar', cr, 'Qcoordinates', [cr crd crdd],'cm2joint',[0,0,0],'joint2cm',[4.68, 0.61, 4.93])
frames(1).setProperties('mass', 147000, 'Jmatrix', [1.6701e6,  3.6194e5, -1.6755e6; 3.6194e5,  1.2872e7,  1.9375e3; -1.6755e6,  1.9375e3,  1.2143e7])
frames(1).setProperties('Fvec', [0,0,-9.81*147000], 'Tvec', [0,0,10000], 'initconditions', [0,0,0])

frames(2).setProperties('rotationaxis', 0, 'rotationvar', 0, 'Qcoordinates', [trolley, trolleyd, trolleydd], 'cm2joint', [0,0,7.00],'joint2cm', [trolley, 0,0])
frames(2).setProperties('mass', 16800,'Jmatrix', [1.4080e4, -3.0649e3,  2.7903e2; -3.0649e3,  3.2480e4,  7.1065e0; 2.7903e2,  7.1065e0,  2.7601e4])
frames(2).setProperties('Fvec', [0,0,-9.81*16800], 'Tvec', [0,0,0], 'initconditions', [0,0,0])
for i = 1:wiresegments  
    frames(2 + 3*i-2).setProperties('rotationaxis', 3, 'rotationvar', theta(i), 'Qcoordinates', [theta(i),thetad(i),thetadd(i)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
    frames(2 + 3*i-2).setProperties('mass', 5, 'Jmatrix', [41,0,0;0,41,0; 0,0, 0.025])
    frames(2 + 3*i-2).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
    frames(2 + 3*i-1).setProperties('rotationaxis', 1, 'rotationvar', phi(i), 'Qcoordinates', [phi(i),phid(i),phidd(i)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
    frames(2 + 3*i-1).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
    frames(2 + 3*i-1).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
    frames(2 + 3*i).setProperties('rotationaxis', 0, 'rotationvar', 0, 'Qcoordinates', [lambda(i),lambdad(i),lambdadd(i)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,-lambda(i)])
    frames(2 + 3*i).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
    frames(2 + 3*i).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
end

frames(wiresegments*3 + 3).setProperties('rotationaxis', 0, 'rotationvar', 0, 'Qcoordinates', [],'cm2joint',[0,0,0],'joint2cm',[0,0, -1.5])
frames(wiresegments*3 + 3).setProperties('mass', 5000, 'Jmatrix', [7500,  0, 0; 0,  7500,  0; 0,  0,  7500])
frames(wiresegments*3 + 3).setProperties('Fvec', [0,0,-9.81*5000], 'Tvec', [0,0,0], 'initconditions', [0,0,0])
%{
% Wire 1
frames(3).setProperties('rotationaxis', 3, 'rotationvar', theta(1), 'Qcoordinates', [theta(1),thetad(1),thetadd(1)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
frames(3).setProperties('mass', 5, 'Jmatrix', [41,0,0;0,41,0; 0,0, 0.025])
frames(3).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
frames(4).setProperties('rotationaxis', 1, 'rotationvar', phi(1), 'Qcoordinates', [phi(1),phid(1),phidd(1)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
frames(4).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
frames(4).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
frames(5).setProperties('rotationaxis', 0, 'rotationvar', 0, 'Qcoordinates', [lambda(1),lambdad(1),lambdadd(1)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,-lambda(1)])
frames(5).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
frames(5).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
% Wire 2
frames(6).setProperties('rotationaxis', 3, 'rotationvar', theta(2), 'Qcoordinates', [theta(2),thetad(2),thetadd(2)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
frames(6).setProperties('mass', 5, 'Jmatrix', [41,0,0;0,41,0; 0,0, 0.025])
frames(6).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
frames(7).setProperties('rotationaxis', 1, 'rotationvar', phi(2), 'Qcoordinates', [phi(2),phid(2),phidd(2)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
frames(7).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
frames(7).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
frames(8).setProperties('rotationaxis', 0, 'rotationvar', 0, 'Qcoordinates', [lambda(2),lambdad(2),lambdadd(2)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,-lambda(2)])
frames(8).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
frames(8).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
% Wire 3
frames(9).setProperties('rotationaxis', 3, 'rotationvar', theta(3), 'Qcoordinates', [theta(3),thetad(3),thetadd(3)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
frames(9).setProperties('mass', 5, 'Jmatrix', [41,0,0;0,41,0; 0,0, 0.025])
frames(9).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
frames(10).setProperties('rotationaxis', 1, 'rotationvar', phi(1), 'Qcoordinates', [phi(1),phid(1),phidd(1)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
frames(10).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
frames(10).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
frames(11).setProperties('rotationaxis', 0, 'rotationvar', 0, 'Qcoordinates', [lambda(1),lambdad(1),lambdadd(1)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,-lambda(1)])
frames(11).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
frames(11).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0; 0,0,0])
% Wire 4
%}

noofframes = 2 + wiresegments*3

Q = frames(noofframes).getQs(frames)
B = frames(noofframes).makeB(frames)

%{
Bdot = frames(noofframes).makeBdot(frames)
D = frames(noofframes).makeD(frames); D = simplify(D)
M = frames(noofframes).makeM(frames); 
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


%}
