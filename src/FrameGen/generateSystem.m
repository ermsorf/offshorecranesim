% clear
% clc

%%% Crane rotation
%{
psi = sym('psi',[1, 2],'real');
psid = sym('psid',[1, 2],'real');
psidd = sym('psidd',[1, 2],'real');
%}
% Crane Vars
syms cr crd crdd trolley trolleyd trolleydd real

wiresegments = 3 ;

% Wire rotation - 3
theta = sym('theta',[1, wiresegments+1],'real');
thetad = sym('thetad',[1, wiresegments+1],'real');
thetadd = sym('thetadd',[1, wiresegments+1],'real');

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
    frames(2 + 3*i-2).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0])
    frames(2 + 3*i-1).setProperties('rotationaxis', 1, 'rotationvar', phi(i), 'Qcoordinates', [phi(i),phid(i),phidd(i)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,0])
    frames(2 + 3*i-1).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
    frames(2 + 3*i-1).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [0,0,0])
    frames(2 + 3*i).setProperties('rotationaxis', 0, 'rotationvar', 0, 'Qcoordinates', [lambda(i),lambdad(i),lambdadd(i)], 'cm2joint', [0,0,0], 'joint2cm',[0,0,lambda(i)])
    frames(2 + 3*i).setProperties('mass', 5, 'Jmatrix', [41,0,0; 0, 41,0; 0,0,0.025])
    frames(2 + 3*i).setProperties('Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0], 'initconditions', [-5,0,0])
end

frames(wiresegments*3 + 3).setProperties('rotationaxis', 3, 'rotationvar', theta(wiresegments+1), 'Qcoordinates', [theta(wiresegments+1), thetad(wiresegments+1), thetadd(wiresegments+1)],'cm2joint',[0,0,0],'joint2cm',[0,0, -1.5])
frames(wiresegments*3 + 3).setProperties('mass', 5000, 'Jmatrix', [7500,  0, 0; 0,  7500,  0; 0,  0,  7500])
frames(wiresegments*3 + 3).setProperties('Fvec', [0,0,-9.81*5000], 'Tvec', [0,0,0], 'initconditions', [0,0,0])





noofframes = 3 + wiresegments*3

Q = frames(noofframes).getQs(frames)
B = frames(noofframes).makeB(frames)
Bdot = frames(noofframes).makeBdot(frames)
D = frames(noofframes).makeD(frames);
M = frames(noofframes).makeM(frames); 
F = frames(noofframes).makeF(frames)
initCond = frames(noofframes).getInitCond(frames)
Mstar = B' * M * B; 
Nstar = B' * (M*Bdot + D*M*B);


% Fstar = B' * F;
% eqs_of_motion = Mstar * Q(:,3) + Nstar*Q(:,2); eqs_of_motion =
% simplify(eqs_of_motion)


%%
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
    configexport(system, 'cullingtest.json')
end
%}

