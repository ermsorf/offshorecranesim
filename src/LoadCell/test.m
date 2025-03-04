addpath("../FrameGen")

theta = sym('theta',[1 10],'real');
thetadot = sym('thetadot',[1 10],'real');
thetaddot = sym('thetaddot',[1 10],'real');
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 theta10
syms l1 l2dot l2 l3 l4 l5 l6 l7 l8 l9 l10 l11 l12 l13 l14 l15 real
syms lambda real % elongation of wire
% 
 % theta(1)  = 0; thetadot(1) = 0; thetaddot(1) = 0;
% theta2  = 0;
 % theta(3)  = 0; thetadot(3) = 0; thetaddot(3) = 0;
% theta4  = 0;
% theta5  = 0;
% theta6  = 0;
% theta7  = 0;
% theta8  = 0;
% theta9  = 0;
% theta10 = 0;



%% lengths in [mm]

frames(1) = Frame('framenumber',1, 'rotationaxis', 0, 'rotationvar',theta1, 'mass', 1, 'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0],'joint2cm',[0,0,6077]);
frames(2) = Frame('framenumber',2, 'rotationaxis', 3, 'rotationvar',theta2, 'mass', 1, 'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[0,0,12155],'joint2cm',[4680,610,4926]);
frames(3) = Frame('framenumber',3, 'rotationaxis', 0, 'rotationvar',theta3, 'mass', 1, 'Qcoordinates',[theta(3),thetadot(3),thetaddot(3)],'cm2joint',[0,0,0],'joint2cm',[l2dot,0,0]);
frames(4) = Frame('framenumber',4, 'rotationaxis', 3, 'rotationvar',theta4, 'mass', 1, 'Qcoordinates',[theta(4),thetadot(4),thetaddot(4)],'cm2joint',[0,0,0],'joint2cm',[0,0,-12-lambda]);
frames(5) = Frame('framenumber',5, 'rotationaxis', 1, 'rotationvar',theta5, 'mass', 1, 'Qcoordinates',[theta(5),thetadot(5),thetaddot(5)],'cm2joint',[0,0,-12],'joint2cm',[0,0,-12-lambda]);
frames(6) = Frame('framenumber',6, 'rotationaxis', 2, 'rotationvar',theta6, 'mass', 1, 'Qcoordinates',[theta(6),thetadot(6),thetaddot(6)],'cm2joint',[0,0,-12],'joint2cm',[0,0,-12-lambda]);
frames(7) = Frame('framenumber',7, 'rotationaxis', 1, 'rotationvar',theta7, 'mass', 1, 'Qcoordinates',[theta(7),thetadot(7),thetaddot(7)],'cm2joint',[0,0,-12],'joint2cm',[0,0,-12-lambda]);
frames(8) = Frame('framenumber',8, 'rotationaxis', 2, 'rotationvar',theta8, 'mass', 1, 'Qcoordinates',[theta(8),thetadot(8),thetaddot(8)],'cm2joint',[0,0,-12],'joint2cm',[0,0,-12-lambda]);
frames(9) = Frame('framenumber',9, 'rotationaxis', 0, 'rotationvar',theta9, 'mass', 1, 'Qcoordinates',[theta(9),thetadot(9),thetaddot(9)],'cm2joint',[0,0,-12],'joint2cm',[0,0,-207]);
frames(10) = Frame('framenumber',10, 'rotationaxis', 3, 'rotationvar',theta10, 'mass', 100, 'Qcoordinates',[theta(10),thetadot(10),thetaddot(10)],'cm2joint',[0,0,-585],'joint2cm',[0,0,600]);


%ReactionForces(frames)
%DoublePendulum(frames)

 

