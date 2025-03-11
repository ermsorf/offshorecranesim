

addpath("C:\Users\erl-l\Documents\GitHub\offshorecranesim\src\FrameGen")

theta = sym('theta',[1 14],'real');
thetadot = sym('thetadot',[1 14],'real');
thetaddot = sym('thetaddot',[1 14],'real');
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 theta10 theta11 theta12 theta13 theta14 theta15 theta16 theta17 real
syms l1 L2dot l2 l3 l4 l5 l6 l7 l8 l9 l10 l11 l12 l13 l14 l15 real
syms lambda real % elongation of wire

frames(1) = Frame('framenumber',1, 'rotationaxis', 3, 'rotationvar',theta(1), 'mass', 1,'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0],'joint2cm',[0, 0, 0]);          % First wire segment    
frames(2) = Frame('framenumber',2, 'rotationaxis', 1, 'rotationvar',theta(2), 'mass', 1, 'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[0,0,0],'joint2cm',[0, 0, 0]);         % First wire segment
frames(3) = Frame('framenumber',3, 'rotationaxis', 3, 'rotationvar',theta(3), 'mass', 1, 'Qcoordinates',[theta(3),thetadot(3),thetaddot(3)],'cm2joint',[0,0,0],'joint2cm',[0, 0, -30])       % First wire segment    %Body 1