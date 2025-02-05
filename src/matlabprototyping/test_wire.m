% test wire
clear
clc
wiresegments = 5;
theta = sym('theta',[1, 2*wiresegments],'real');
thetadot = sym('thetadot',[1, 2*wiresegments],'real');
thetaddot = sym('thetaddot',[1, 2*wiresegments],'real');


g = 9.81;
for i = 1:wiresegments
    frames(2*i-1) = Frame('framenumber',2*i-1,'rotationaxis',3,'rotationvar', theta(2*i-1), 'Qcoordinates', [theta(2*i-1),thetadot(2*i-1),thetaddot(2*i-1)], 'cm2joint',[0,0,0],'joint2cm',[0.5,0,0],'mass', 10, 'Jmatrix', [10*0.1^2/2,0,0;0,10*(3*0.1^2+1^2)/12,0;0,0,10*(3*0.1^2+1^2)/12], 'Fvec', [0,0,-g*10], 'Tvec', [0,0,0]);
    frames(2*i) = Frame('framenumber',2*i,'rotationaxis',1,'rotationvar', theta(2*i), 'Qcoordinates', [theta(2*i),thetadot(2*i),thetaddot(2*i)], 'cm2joint',[0,0,0],'joint2cm',[0.5,0,0],'mass', 10, 'Jmatrix', [10*(3*0.1^2+1^2)/12,0,0;0,10*(3*0.1^2+1^2)/12,0;0,0,10*0.1^2/2], 'Fvec', [0,0,-g*10], 'Tvec', [0,0,0]);
end
n = 6 %wiresegments*2

Q = getQs(frames(n),frames);
B = makeB(frames(n),frames);
Bdot = makeBdot(frames(n),frames);
D = makeD(frames(n),frames);
M = makeM(frames(n),frames);
F = makeF(frames(n),frames);

Mstar = B' * M * B;
Nstar = B' * (M*Bdot + D*M*B);
Fstar = B' * F;
