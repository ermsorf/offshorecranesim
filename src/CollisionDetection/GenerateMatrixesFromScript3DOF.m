clear
clc

addpath("C:\Users\erl-l\Documents\GitHub\offshorecranesim\src\FrameGen")
noofframes = 3;
theta = sym('theta',[1 14],'real');
thetadot = sym('thetadot',[1 14],'real');
thetaddot = sym('thetaddot',[1 14],'real');
syms theta1 theta2 theta3  real
syms l g real

j = [5000*5^2 / 6, 0 ,0; 0, 5000*5^2 / 6, 0; 0,0,5000*5^2 / 6]

frames(1) = Frame('Tvec', [0,0,0], 'Fvec', [0,0,0],'Jmatrix', eye(3), 'framenumber',1, 'rotationaxis', 1, 'rotationvar',theta(1), 'mass', 5,'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0.001],'joint2cm',[0, 0, 0.001]);        
frames(2) = Frame('Tvec', [0,0,0],'Fvec', [0,0,0],'Jmatrix', eye(3),'framenumber',2, 'rotationaxis', 2, 'rotationvar',theta(2), 'mass', 5, 'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[0,0,0.001],'joint2cm',[0, 0, 0.001]);        
frames(3) = Frame('Tvec', [0,0,0], 'Fvec', [0,0,-g*5000],'Jmatrix', j , 'framenumber',3, 'rotationaxis', 3, 'rotationvar',theta(3), 'mass', 5000,'Qcoordinates',[theta(3),thetadot(3),thetaddot(3)],'cm2joint',[0,0,0.001],'joint2cm',[0, 0, -l]); 


B = frames(3).makeB(frames)
Bdot = frames(3).makeBdot(frames)
BT = B'

M = frames(3).makeM(frames)
W = frames(3).makeW(frames)
F = frames(3).makeF(frames)
D = frames(3).makeD(frames)
Q = frames(3).getQs(frames)



Mstar = BT * M * B

Fstar = BT*F

Nstar = BT * (M*Bdot + D*M*B)

eq = (Mstar * Q(:,3) + Nstar * Q(:,2) - Fstar == 0)
eq = simplify(Mstar * Q(:,3) + Nstar * Q(:,2) - Fstar == 0)


