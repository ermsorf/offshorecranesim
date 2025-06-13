%% The following function returns reaction forces for the crane that has 3 links. Each link has 3 DOF

function ReactionForces = Reaction_3_link(frames) 

% clear; clc;
% syms t thetad phid real
syms g real
syms Mload mHook m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 m11 m12 real
% syms m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 real
syms lambda1 lambda2 lambda3 real
syms trolley trolleydot trolleyddot L2dot real

syms Re1I Re2I Re3I real
syms Re11 Re21 Re31 real
syms Re12 Re22 Re32 real
syms Re13 Re23 Re33 real
syms Re14 Re24 Re34 real
syms Re15 Re25 Re35 real
syms Re16 Re26 Re36 real
syms Re17 Re27 Re37 real
syms Re18 Re28 Re38 real
syms Re19 Re29 Re39 real
syms Re110 Re210 Re310 real




%%   MASSES IN [kg]:
mHook = 500;
syms Xddot11 Xddot21 Xddot31 real
syms Xddot12 Xddot22 Xddot32 real
syms Xddot13 Xddot23 Xddot33 real
syms Xddot14 Xddot24 Xddot34 real
syms Xddot15 Xddot25 Xddot35 real
syms Xddot16 Xddot26 Xddot36 real
syms Xddot17 Xddot27 Xddot37 real
syms Xddot18 Xddot28 Xddot38 real
syms Xddot19 Xddot29 Xddot39 real
syms Xddot110 Xddot210 Xddot310 real



%   MASSES IN [kg]:

m1 = frames(1).mass;
m2 = frames(2).mass;
m3 = frames(3).mass;
m4 = frames(4).mass;
m5 = frames(5).mass;
m6 = frames(6).mass;
m7 = frames(7).mass;
m8 = frames(8).mass;
m9 = frames(9).mass;
m10 = frames(10).mass;
m11 = frames(11).mass;
m12 = frames(12).mass;
% m13 = frames(13).mass;
% m14 = frames(14).mass;



%   GRAVITY [m/s^2]
g = 9.81;


% THE ROTATION MATRICES AND THEIR TRANSPOSES
R1 = frames(1).makeEr();
R1 = R1(1:3,1:3);           %Rotation
R1T= R1.';                  %Transpose

R21 = frames(2).makeEr();
R21 = R21(1:3,1:3);         %Rotation
R21T= R21.';                %Transpose

R32 = frames(3).makeEr();
R32 = R32(1:3,1:3);         %Rotation
R32T= R32.';                %Transpose

R43 = frames(4).makeEr();
R43 = R43(1:3,1:3);         %Rotation
R43T= R43.';                %Transpose

R54 = frames(5).makeEr();
R54 = R54(1:3,1:3);         %Rotation
R54T= R54.';                %Transpose

R65 = frames(6).makeEr();
R65 = R65(1:3,1:3);         %Rotation
R65T= R65.';                %Transpose

R76 = frames(7).makeEr();
R76 = R76(1:3,1:3);         %Rotation
R76T= R76.';                %Transpose

R87 = frames(8).makeEr();
R87 = R87(1:3,1:3);         %Rotation
R87T= R87.';                %Transpose

R98 = frames(9).makeEr();
R98 = R98(1:3,1:3);         %Rotation
R98T= R98.';                %Transpose

R109 = frames(10).makeEr();
R109 = R109(1:3,1:3);       %Rotation
R109T= R109.';              %Transpose

R1110 = frames(11).makeEr();
R1110 = R1110(1:3,1:3);     %Rotation
R1110T = R1110.';           %Transpose

R1211 = frames(12).makeEr();
R1211 = R1211(1:3,1:3);     %Rotation
R1211T = R1211.';%Transpose


%% Qd - coordinates
numFrames = 12; % Define number of frames
QdMatrix = sym(zeros(numFrames, 1)); % Preallocate for efficiency

for i = 1:numFrames
    QdMatrix(i) = frames(i).Qcoordinates(2);
end

QdMatrix;

% %% Qdd-COORDINATES
% % Extract the generalized coordinates (Thetaddot) for frames 1 to 10
% % Chat GPT was used to optimize the code
% 
numFrames = 12; % Define number of frames
QddMatrix = sym(zeros(numFrames, 1)); % Preallocate for efficiency

for i = 1:numFrames
    QddMatrix(i) = frames(i).Qcoordinates(3);
end

QddMatrix;


% %%  DEFINING Xddots UPDATE THIS, 
% %   Extracting the x dots from array.
B    = frames(12).makeB(frames);
Bdot = frames(12).makeBdot(frames);

% Xddots = Bdot*QddMatrix;

Xddots = Bdot*QdMatrix + B*QddMatrix;

Xddot11 = Xddots(1);            % Acceleration Body 1 (frame 1)
Xddot21 = Xddots(2);
Xddot31 = Xddots(3);


Xddot12 = Xddots(7);            % Acceleration Body 2 (frame 2)
Xddot22 = Xddots(8);
Xddot32 = Xddots(9);

Xddot13 = Xddots(25);           % Acceleration Body 3 (frame 5)
Xddot23 = Xddots(26);
Xddot33 = Xddots(27);

Xddot14 = Xddots(43);           % Acceleration Body 4 (Account for frame 8 here)
Xddot24 = Xddots(44);
Xddot34 = Xddots(45);

Xddot15 = Xddots(61);           % Acceleration Body 5 (Account for frame 11)
Xddot25 = Xddots(62);
Xddot35 = Xddots(63);

Xddot16 = Xddots(67);           % Acceleration Body 6  (Account for frame 12)
Xddot26 = Xddots(68);
Xddot36 = Xddots(69);

% Xddot17 = Xddots(73);           % Acceleration Frame 7 (frame 13)
% Xddot27 = Xddots(74);
% Xddot37 = Xddots(75);

% Xddot18 = Xddots(66);           % Acceleration Frame 8 (frame 14)
% Xddot28 = Xddots(67); 
% Xddot38 = Xddots(68);


%% Pre-calculating "pull back and fourth" matrices
%  There are 14 frames, but 9 bodies. Each wire segment consists of 3
%  frames with their respected degree of freedom. This is to make the derivative
%  of the B-matrix easier. 

FromIto2  = R21T * R1T;
From1to2  = R21T;
From3to2  = R32;
From4to2  = R32 * R43;
From5to2  = R32 * R43 * R54;
From6to2  = R32 * R43 * R54 * R65;
From7to2  = R32 * R43 * R54 * R65 * R76;
From8to2  = R32 * R43 * R54 * R65 * R76 * R87;
From9to2  = R32 * R43 * R54 * R65 * R76 * R87 * R98;
From10to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109;
From11to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110;
From12to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211;
% From13to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312;
% From14to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413;
% From15to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514;
% From16to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514 * R1615;
% From17to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514 * R1615 * R1716;
 
%% Equation from FBD (Free Body Diagram)  The formulas below assume thet the wire is 3 bodies

% Body 1 (frame 1)
  Equation1 = FromIto2 * [0; 0; -m1*g] + FromIto2 * [Re1I; Re2I; Re3I] + From1to2 * [-Re11; -Re21; -Re31] == FromIto2 * m1* [Xddot11; Xddot21; Xddot31];
% simplify(Equation1);

eq1 = Equation1(1);
eq2 = Equation1(2);
eq3 = Equation1(3);

%  BODY 2 (frame 2)
  Equation2 = FromIto2 * [0; 0; -m2*g] + From1to2 * [Re11 ; Re21; Re31] + [-Re12; -Re22; -Re32] == FromIto2 * m2* [Xddot12; Xddot22; Xddot32];
% simplify(Equation2);

eq4 = Equation2(1);
eq5 = Equation2(2);
eq6 = Equation2(3);

% Body 3 (frame 5)
  Equation3 = FromIto2 * [0; 0; -m3*g] +  From5to2 * [-Re13; -Re23; -Re33] + [Re12; Re22; Re32] == FromIto2 *m3 * [Xddot13; Xddot23; Xddot33];
% simplify(Equation3);

eq7 = Equation3(1);
eq8 = Equation3(2);
eq9 = Equation3(3);

% Body 4 (frame 8)
  Equation4 = FromIto2 * [0; 0; -m4*g] + From5to2 * [Re13; Re23; Re33] + From8to2 * [-Re14; -Re24; -Re34] == FromIto2 * m4 * [Xddot14; Xddot24; Xddot34];
% simplify(Equation4);

eq10 = Equation4(1);
eq11 = Equation4(2);
eq12 = Equation4(3);

% Body 5 (frame 11)
  Equation5 = FromIto2 * [0; 0; -m5*g] + From11to2 * [-Re15; -Re25; -Re35] + From8to2 * [Re14; Re24; Re34] == FromIto2 * m5 * [Xddot15; Xddot25; Xddot35];
% simplify(Equation5);

eq13 = Equation5(1);
eq14 = Equation5(2);
eq15 = Equation5(3);

% Body 6 (frame 12)


  Equation6 = FromIto2 * [0; 0; -g*(mHook+m6)] + From11to2 * [Re15; Re25; Re35]  == FromIto2 * m6 * [Xddot16; Xddot26; Xddot36];
% simplify(Equation6);

eq16 = Equation6(1);
eq17 = Equation6(2);
eq18 = Equation6(3);



%% SOLVING SYSTEM OF EQUATIONS

eqs     =  [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, ...
             eq10, eq11, eq12, eq13, eq14, eq15, eq16, eq17, eq18, ...
             ];


unknowns =  [Re1I, Re2I, Re3I, ... 
             Re11, Re21, Re31, ...
             Re12, Re22, Re32, ... 
             Re13, Re23, Re33, ... 
             Re14, Re24, Re34, ... 
             Re15, Re25, Re35, ...
             ];


sol = solve(eqs, unknowns);
solArray = struct2array(sol);
ReactionForces = (solArray(6)/g)-m3-m4-m5-mHook;  % Reaction forces now in terms of mass for load cell output
end
