addpath("../FrameGen")

clear
clc

% defineSystem
clear
clc

%% Define amount of links on wire
wiresegments = 1;
syms cr crd crdd trolley trolleyd trolleydd container containerd containerdd real
% Wire rotation - 3
theta = sym('theta',[1, wiresegments],'real');
thetad = sym('thetad',[1, wiresegments],'real');
thetadd = sym('thetadd',[1, wiresegments],'real');
% Wire Rotation - 1
phi = sym('phi',[1, wiresegments],'real');
phid = sym('phid',[1, wiresegments],'real');
phidd = sym('phidd',[1, wiresegments],'real');
% Wire Lengths
lambda = sym('lambda',[1, wiresegments],'real');
lambdad = sym('lambdad',[1, wiresegments],'real');
lambdadd = sym('lambdadd',[1, wiresegments],'real');
% J-matrix values [kgm^2] for wire
xxLink = 7296;      
yyLink = 7296;       
zzLink = 0.019456;   
 
for i = 1:(wiresegments*3 + 3)
    frames(i) = Frame('framenumber',i);
end
% Crane Boom %%%%%%%%%%%%%%%%%%%%%%%%%%%%
frames(1).setProperties('Qcoordinates', [cr crd crdd], 'initconditions', [0,0,0], ...
    'rotationaxis', 3, 'rotationvar', cr, ...
    'cm2joint',[0,0,0],'joint2cm',[4.68, 0.61, 4.93], ...
    'mass', 147000, 'Jmatrix', [1.6701e6,  3.6194e5, -1.6755e6; 3.6194e5,  1.2872e7,  1.9375e3; -1.6755e6,  1.9375e3,  1.2143e7], ...
    'Fvec', [0,0,-9.81*147000], 'Tvec', [0,0,0])
% Trolley %%%%%%%%%%%%%%%%%%%%%%%%%%%%
frames(2).setProperties('Qcoordinates', [trolley, trolleyd, trolleydd], 'initconditions', [0,0,0], ...
    'rotationaxis', 0, 'rotationvar', 0, ...
    'cm2joint', [0,-0.61,7.00],'joint2cm', [trolley, 0,0], ...
    'mass', 16800,'Jmatrix', [1.4080e4, -3.0649e3,  2.7903e2; -3.0649e3,  3.2480e4,  7.1065e0; 2.7903e2,  7.1065e0,  2.7601e4], ...
    'Fvec', [0,0,-9.81*16800], 'Tvec', [0,0,0]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if wiresegments >= 1
    % Wiresegment 1, 3-axis
    frames(3).setProperties('Qcoordinates', [theta(1),thetad(1),thetadd(1)], 'initconditions', [0,0,0], ...
        'rotationaxis', 3, 'rotationvar', theta(1), ...
        'cm2joint', [0,0,0], 'joint2cm',[0,0,0], ...
        'mass', 254, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*254], 'Tvec', [0,0,0])
    % Wiresegment 1, 1-axis
    frames(4).setProperties('Qcoordinates', [phi(1),phid(1),phidd(1)], 'initconditions', [0,0,0], ...
        'rotationaxis', 1, 'rotationvar', phi(1), ...
        'cm2joint', [0,0,0], 'joint2cm',[0,0,0], ...
        'mass', 254, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*254], 'Tvec', [0,0,0])
    % Wiresegment 1, extension
    frames(5).setProperties('Qcoordinates', [lambda(1),lambdad(1),lambdadd(1)], 'initconditions', [-5,0,0], ...
        'rotationaxis', 0, 'rotationvar', 0, ...
        'cm2joint', [0,0,0], 'joint2cm',[0,0,lambda(1)/2], ...
        'mass', 254, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*5], 'Tvec', [0,0,0])
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if wiresegments >= 2
    % Wiresegment 2, 3-axis
    frames(6).setProperties('Qcoordinates', [theta(2),thetad(2),thetadd(2);lambda(1),lambdad(1),lambdadd(1)], 'initconditions', [0,0,0], ...
        'rotationaxis', 3, 'rotationvar', theta(2), ...
        'cm2joint', [0,0,lambda(1)/2], 'joint2cm',[0,0,0], ...
        'mass', 254, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*254], 'Tvec', [0,0,0])
    % Wiresegment 2, 1-axis
    frames(7).setProperties('Qcoordinates', [phi(2),phid(2),phidd(2)], 'initconditions', [0,0,0], ...
        'rotationaxis', 1, 'rotationvar', phi(2), ...
        'cm2joint', [0,0,0], 'joint2cm',[0,0,0], ...
        'mass', 0, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*254], 'Tvec', [0,0,0])
    % Wiresegment 2, extension
    frames(8).setProperties('Qcoordinates', [lambda(2),lambdad(2),lambdadd(2)], 'initconditions', [-5,0,0], ...
        'rotationaxis', 0, 'rotationvar', 0, ...
        'cm2joint', [0,0,0], 'joint2cm',[0,0,lambda(2)/2], ...
        'mass', 254, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*254], 'Tvec', [0,0,0])
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if wiresegments >= 3
    % Wiresegment 3, 3-axis
    frames(9).setProperties('Qcoordinates', [theta(3),thetad(3),thetadd(3);lambda(2),lambdad(2),lambdadd(2)], 'initconditions', [0,0,0], ...
        'rotationaxis', 3, 'rotationvar', theta(3), ...
        'cm2joint', [0,0,lambda(2)/2], 'joint2cm',[0,0,0], ...
        'mass', 254, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*254], 'Tvec', [0,0,0])
    % Wiresegment 3, 1-axis
    frames(10).setProperties('Qcoordinates', [phi(3),phid(3),phidd(3)], 'initconditions', [0,0,0], ...
        'rotationaxis', 1, 'rotationvar', phi(3), ...
        'cm2joint', [0,0,0], 'joint2cm',[0,0,0], ...
        'mass', 254, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*254], 'Tvec', [0,0,0])
    % Wiresegment 3, extension
    frames(11).setProperties('Qcoordinates', [lambda(3),lambdad(3),lambdadd(3)], 'initconditions', [-5,0,0], ...
        'rotationaxis', 0, 'rotationvar', 0, ...
        'cm2joint', [0,0,0], 'joint2cm',[0,0,lambda(3)/2], ...
        'mass', 254, 'Jmatrix', [xxLink,0,0; 0,yyLink,0; 0,0,zzLink], ...
        'Fvec', [0,0,-9.81*254], 'Tvec', [0,0,0])
end
% Container %%%%%%%%%%%%%%%%%%%%%%%%%%%%
lastframe = length(frames);
frames(lastframe).setProperties('Qcoordinates', [container, containerd, containerdd; lambda(wiresegments),lambdad(wiresegments),lambdadd(wiresegments)], 'initconditions', [0,0,0],...
    'rotationaxis', 3, 'rotationvar', container, ...
    'cm2joint',[0,0,lambda(wiresegments)/2],'joint2cm',[0,0, -1.5], ...
    'mass', 5000, 'Jmatrix', [7500,  0, 0; 0,  7500,  0; 0,  0,  7500], ...
    'Fvec', [0,0,-9.81*5000], 'Tvec', [0,0,0])


reactions = ReactionForces(frames);   % 1 link 
%reactions = Reaction_3_link(frames);  % 3 link


% Reaction_Only_3_links(frames);
%Equation_of_Motion(frames)
%Reaction_5_link(frames)

%% Creates a JSON file, Comment out if file is not needed.
% reactionsStr = arrayfun(@char, reactions, 'UniformOutput', false);
% 
% % Convert the cell array of strings to a JSON-formatted string
% jsonStr = jsonencode(reactionsStr);
% 
% % Write the JSON string to a file
% fid = fopen('reactions_8_Mai_2025_3_link.json', 'w');
% if fid == -1
%     error('Cannot open file for writing');
% end
% fwrite(fid, jsonStr, 'char');
% fclose(fid);


%% Three wire segment
% frames(1) = Frame('framenumber',1, 'rotationaxis', 0, 'rotationvar',theta(1), 'mass', 147000, 'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0],'joint2cm',[0, 0, 6077]);          % Tower                 %Body 1
% frames(2) = Frame('framenumber',2, 'rotationaxis', 3, 'rotationvar',theta(2), 'mass', 147000, 'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[0,0,12155],'joint2cm',[4680 , 610, 4926]);% Boom                  %Body 2
% frames(3) = Frame('framenumber',3, 'rotationaxis', 0, 'rotationvar',theta(3), 'mass', 17000,  'Qcoordinates',[theta(3),thetadot(3),thetaddot(3)],'cm2joint',[0,0,0],'joint2cm',[0, 0, 0]);             % Cart                  %Body 3
% frames(4) = Frame('framenumber',4, 'rotationaxis', 3, 'rotationvar',theta(4), 'mass', 1,'Qcoordinates',[theta(4),thetadot(4),thetaddot(4)],'cm2joint',[0,0,-20],'joint2cm',[0, 0, -20]);               % First wire segment    
% frames(5) = Frame('framenumber',5, 'rotationaxis', 1, 'rotationvar',theta(5), 'mass', 1, 'Qcoordinates',[theta(5),thetadot(5),thetaddot(5)],'cm2joint',[0,0,0],'joint2cm',[0, 0, 0]);                  % First wire segment
% frames(6) = Frame('framenumber',6, 'rotationaxis', 0, 'rotationvar',theta(6), 'mass', 85, 'Qcoordinates',[theta(6),thetadot(6),thetaddot(6)],'cm2joint',[0,0,0],'joint2cm',[0, 0, 0]);                 % First wire segment    %Body 4
% frames(7) = Frame('framenumber',7, 'rotationaxis', 3, 'rotationvar',theta(7), 'mass', 1,'Qcoordinates',[theta(7),thetadot(7),thetaddot(7)],'cm2joint',[0,0,-20],'joint2cm',[0,0,-20]);                 % Second wire segment   
% frames(8) = Frame('framenumber',8, 'rotationaxis', 1, 'rotationvar',theta(8), 'mass', 1, 'Qcoordinates',[theta(8),thetadot(8),thetaddot(8)],'cm2joint',[0,0,0],'joint2cm',[0,0,0]);                    % Second wire segment
% frames(9) = Frame('framenumber',9, 'rotationaxis', 0, 'rotationvar',theta(9), 'mass', 85, 'Qcoordinates',[theta(9),thetadot(9),thetaddot(9)],'cm2joint',[0,0,0],'joint2cm',[0,0,0]);                   % Second wire segment   %Body 5
% frames(10) = Frame('framenumber',10, 'rotationaxis', 3, 'rotationvar',theta(10), 'mass', 1, 'Qcoordinates',[theta(10),thetadot(10),thetaddot(10)],'cm2joint',[0,0,-20],'joint2cm',[0,0,-20]);          % Third wire segment    
% frames(11) = Frame('framenumber',11, 'rotationaxis', 1, 'rotationvar',theta(11), 'mass', 1, 'Qcoordinates',[theta(11),thetadot(11),thetaddot(11)],'cm2joint',[0,0,0],'joint2cm',[0,0,0]);              % Third wire segment
% frames(12) = Frame('framenumber',12, 'rotationaxis', 0, 'rotationvar',theta(12), 'mass', 85, 'Qcoordinates',[theta(12),thetadot(12),thetaddot(12)],'cm2joint',[0,0,0],'joint2cm',[0,0,0]);             % Third wire segment    %Body 6
% frames(13) = Frame('framenumber',13, 'rotationaxis', 3, 'rotationvar',theta(13), 'mass', 500, 'Qcoordinates',[theta(13),thetadot(13),thetaddot(13)],'cm2joint',[0,0,-15],'joint2cm',[0,0,-473]);       % Hook                  %Body 7
% frames(14) = Frame('framenumber',14, 'rotationaxis', 1, 'rotationvar',theta(14), 'mass', 10000, 'Qcoordinates',[theta(14),thetadot(14),thetaddot(14)],'cm2joint',[0,0,-773],'joint2cm',[0,0,-50]);     % Box                   %Body 8

%Reaction_3_link(frames)
ReactionForces(frames)
%GetXddots(frames)
%DoublePendulum(frames)


% %% B matrise til Erlend
% frames(1) = Frame('framenumber',1, 'rotationaxis', 3, 'rotationvar',theta(1), 'mass', 1,'Qcoordinates',[theta(1),thetadot(1),thetaddot(1)],'cm2joint',[0,0,0],'joint2cm',[0, 0, 0]);          % First wire segment    
% frames(2) = Frame('framenumber',2, 'rotationaxis', 1, 'rotationvar',theta(2), 'mass', 1, 'Qcoordinates',[theta(2),thetadot(2),thetaddot(2)],'cm2joint',[0,0,0],'joint2cm',[0, 0, -30]);         % First wire segment


