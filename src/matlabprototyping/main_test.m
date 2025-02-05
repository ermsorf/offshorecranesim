clear
clc


g = 9.81;
for i = 1:2
    frames(i) = numFrame('framenumber',i);
end

frames(1).setProperties('rotationaxis', 3, 'anglepos', 1, 'cm2joint',[0,0,0],'joint2cm',[0.5,0,0],'mass', 10, 'Jmatrix', [10*0.1^2/2,0,0;0,10*(3*0.1^2+1^2)/12,0;0,0,10*(3*0.1^2+1^2)/12], 'Fvec', [0,-g*10,0], 'Tvec', [0,0,0]);
frames(2).setProperties('rotationaxis', 3, 'anglepos', 1, 'cm2joint', [0.5,0,0], 'joint2cm', [0.5,0,0],'mass', 10, 'Jmatrix', [10*0.1^2/2,0,0;0,10*(3*0.1^2+1^2)/12,0;0,0,10*(3*0.1^2+1^2)/12], 'Fvec', [0,-g*10,0], 'Tvec', [0,0,0]);



frames(2).makeEabs(frames)