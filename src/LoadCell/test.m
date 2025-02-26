addpath("../FrameGen")

theta = sym('theta',[1 10],'real');
thetadot = sym('thetadot',[1 10],'real');
thetaddot = sym('thetaddot',[1 10],'real');
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9
syms l1 l2dot l2 l3 l4 l5 l6 l7 l8 l9 l10 l11 l12 l13 l14 l15 real

frames(1) = Frame('framenumber',1, 'rotationaxis', 3, 'rotationvar',theta1, 'mass', 1, 'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0],'joint2cm',[0,0,l1]);
frames(2) = Frame('framenumber',2, 'rotationaxis', 3, 'rotationvar',theta2, 'mass', 1, 'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[0,0,l1/2],'joint2cm',[l2,0,0]);
frames(3) = Frame('framenumber',3, 'rotationaxis', 0, 'rotationvar',theta3, 'mass', 1, 'Qcoordinates',[theta(3),thetadot(3),thetaddot(3)],'cm2joint',[0,0,0],'joint2cm',[l2dot,0,0]);
frames(4) = Frame('framenumber',4, 'rotationaxis', 3, 'rotationvar',theta4, 'mass', 1, 'Qcoordinates',[theta(4),thetadot(4),thetaddot(4)],'cm2joint',[0,0,0],'joint2cm',[0,0,-l3/2]);
frames(5) = Frame('framenumber',5, 'rotationaxis', 2, 'rotationvar',theta5, 'mass', 1, 'Qcoordinates',[theta(5),thetadot(5),thetaddot(5)],'cm2joint',[0,0,-l3/2],'joint2cm',[0,0,-l4/2]);
frames(6) = Frame('framenumber',6, 'rotationaxis', 3, 'rotationvar',theta6, 'mass', 1, 'Qcoordinates',[theta(6),thetadot(6),thetaddot(6)],'cm2joint',[0,0,-l4/2],'joint2cm',[0,0,-l5/2]);
frames(7) = Frame('framenumber',7, 'rotationaxis', 2, 'rotationvar',theta7, 'mass', 1, 'Qcoordinates',[theta(7),thetadot(7),thetaddot(7)],'cm2joint',[0,0,-l5/2],'joint2cm',[0,0,-l6/2]);
frames(8) = Frame('framenumber',8, 'rotationaxis', 3, 'rotationvar',theta8, 'mass', 1, 'Qcoordinates',[theta(8),thetadot(8),thetaddot(8)],'cm2joint',[0,0,-l6/2],'joint2cm',[0,0,-l7/2]);
frames(9) = Frame('framenumber',9, 'rotationaxis', 3, 'rotationvar',theta9, 'mass', 1, 'Qcoordinates',[theta(9),thetadot(9),thetaddot(9)],'cm2joint',[0,0,-l7/2],'joint2cm',[0,0,-l8]);
frames(10) = Frame('framenumber',10, 'rotationaxis', 3, 'rotationvar',theta10, 'mass', 1, 'Qcoordinates',[theta(10),thetadot(10),thetaddot(10)],'cm2joint',[0,0,-l7/2],'joint2cm',[0,0,-l8]);

%frames(1) = Frame('framenumber',1, 'rotationaxis', 3, 'rotationvar',theta1, 'mass', 2, 'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[l1,0,0],'joint2cm',[0,l2,0]);
%DoublePendulum(frames)

% ReactionForces(frames)

% B = frames(4).makeB(frames)
% 

% frame5 = Frame()
% frame5.makeBdot(frame5);