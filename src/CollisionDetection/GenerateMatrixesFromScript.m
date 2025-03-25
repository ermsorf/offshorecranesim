clear
clc

addpath("C:\Users\erl-l\Documents\GitHub\offshorecranesim\src\FrameGen")
noofframes = 2;
theta = sym('theta',[1 14],'real');
thetadot = sym('thetadot',[1 14],'real');
thetaddot = sym('thetaddot',[1 14],'real');
syms theta1 theta2 theta3  real
syms l g real
syms lambda real % elongation of wire

frames(1) = Frame('Tvec', [0,0,0], 'Fvec', [0,0,0],'Jmatrix', eye(3), 'framenumber',1, 'rotationaxis', 3, 'rotationvar',theta(1), 'mass', 0,'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0],'joint2cm',[0, 0, 0.001]);          % First wire segment    
frames(2) = Frame('Tvec', [0,0,0],'Fvec', [0,0,-g*5000],'Jmatrix', eye(3),'framenumber',2, 'rotationaxis', 1, 'rotationvar',theta(2), 'mass', 5000, 'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[0,0,0],'joint2cm',[0, 0, l]);         % First wire segment


B = frames(2).makeB(frames)
Bdot = frames(2).makeBdot(frames)
BT = B'
BT = B.'

M = frames(2).makeM(frames)
W = frames(2).makeW(frames)
F = frames(2).makeF(frames)
D = frames(2).makeD(frames)
Q = frames(2).getQs(frames)



Mstar = BT * M * B

Fstar = BT*F

Nstar = BT * (M*Bdot + D*M*B)

eq = simplify(Mstar * Q(:,3) + Nstar * Q(:,2) - Fstar == 0)