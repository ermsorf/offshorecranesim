clear
clc

for i = 1:3
    frames(i) = Frame('framenumber',i);
end

frames(1).setProperties('rotationaxis', [3,1], 'rotationvar', [theta(1),theta(2)], 'Qcoordinates', [theta(1),thetadot(1),thetaddot(1);theta(2),thetadot(2),thetaddot(2)], 'cm2joint', [0,0,0],'joint2cm',[1/2,0,0])
frames(1).setProperties('mass', 2, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);
frames(2).setProperties('rotationaxis', 3, 'rotationvar', theta(3), 'Qcoordinates', [theta(3),thetadot(3),thetaddot(3)], 'cm2joint', [1/2,0,0],'joint2cm',[1/2,0,0])
frames(2).setProperties('mass', 2, 'Jmatrix', [1/6,0,0; 0,1/6,0; 0,0,0]);
