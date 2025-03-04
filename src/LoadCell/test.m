
theta = sym('theta',[1 5],'real');
thetadot = sym('thetadot',[1 5],'real');
thetaddot = sym('thetaddot',[1 5],'real');
syms theta1 theta2 theta3
syms l1 l2 real

frames(1) = Frame('framenumber',1, 'rotationaxis', 3, 'rotationvar',theta1, 'mass', 2, 'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0],'joint2cm',[l2/2,0,0]);
frames(2) = Frame('framenumber',2, 'rotationaxis', 3, 'rotationvar',theta2, 'mass', 2, 'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[l1/2,0,0],'joint2cm',[l2/2,0,0]);
frames(3) = Frame('framenumber',3, 'rotationaxis', 3, 'rotationvar',theta3, 'mass', 2, 'Qcoordinates',[theta(3),thetadot(3),thetaddot(3)],'cm2joint',[l2/2,0,0],'joint2cm',[0,0,0]);


%frames(1) = Frame('framenumber',1, 'rotationaxis', 3, 'rotationvar',theta1, 'mass', 2, 'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[l1,0,0],'joint2cm',[0,l2,0]);
DoublePendulum(frames)
%ReactionForces(frames)

% B = frames(4).makeB(frames)
% 

% frame5 = Frame()
% frame5.makeBdot(frame5);