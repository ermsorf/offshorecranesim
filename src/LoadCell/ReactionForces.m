% This script calculates the reaction forces for the crane. The following calculations account for 4 bodies (1 wire segment).
%  This script has been used for the results in the report of the bachelor projetct.

 
function reactions_1_wireSeg = ReactionForces(frames) 

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


%% The masses are passed in as varibles, these will be replaced by values in JavaScript
syms Mload mHook m1 m2 m3 m4 m5 m6 real

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

% R76 = frames(7).makeEr();
% R76 = R76(1:3,1:3);         %Rotation
% R76T= R76.';                %Transpose

% R87 = frames(8).makeEr();
% R87 = R87(1:3,1:3);         %Rotation
% R87T= R87.';                %Transpose

% R98 = frames(9).makeEr();
% R98 = R98(1:3,1:3);         %Rotation
% R98T= R98.';                %Transpose

% R109 = frames(10).makeEr();
% R109 = R109(1:3,1:3);       %Rotation
% R109T= R109.';              %Transpose
% 
% R1110 = frames(11).makeEr();
% R1110 = R1110(1:3,1:3);     %Rotation
% R1110T = R1110.';           %Transpose
% 
% R1211 = frames(12).makeEr();
% R1211 = R1211(1:3,1:3);     %Rotation
% R1211T = R1211.';%Transpose
% 
% R1312 = frames(13).makeEr();
% R1312 = R1312(1:3,1:3);     %Rotation
% R1312T = R1312.';%Transpose
% 
% R1413 = frames(14).makeEr();
% R1413 = R1413(1:3,1:3);     %Rotation
% R1413T = R1413.';%Transpose
% 
% R1514 = frames(15).makeEr();
% R1514 = R1514(1:3,1:3);%Rotation
% R1514T = R1514.';%Transpose
% 
% R1615 = frames(16).makeEr();
% R1615 = R1615(1:3,1:3);%Rotation
% R1615T = R1615.';%Transpose
% 
% R1716 = frames(17).makeEr();
% R1716 = R1716(1:3,1:3);%Rotation
% R1716T = R1716.';%Transpose

%% Qd - coordinates (velocities)
numFrames = 6; % Define number of frames
QdMatrix = sym(zeros(numFrames, 1)); % Preallocate for efficiency

for i = 1:numFrames
    QdMatrix(i) = frames(i).Qcoordinates(2);
end

QdMatrix;

 %% Qdd-COORDINATES (accelerations)
% % Extract the generalized coordinates (Thetaddot) for frames 1 to 10
% % Chat GPT was used to optimize the code
% 
numFrames = 6; % Define number of frames
QddMatrix = sym(zeros(numFrames, 1)); % Preallocate for efficiency

for i = 1:numFrames
    QddMatrix(i) = frames(i).Qcoordinates(3);
end

QddMatrix;


% 
% %%  DEFINING Xddots UPDATE THIS, 
% %   Extracting the x dots from array.
B    = frames(6).makeB(frames);
Bdot = frames(6).makeBdot(frames);

Xddots = B*QddMatrix+Bdot*QdMatrix;

Xddot11 = Xddots(1);            % Acceleration Body 1 (frame 1) Boom
Xddot21 = Xddots(2);
Xddot31 = Xddots(3);


Xddot12 = Xddots(7);            % Acceleration Body 2 (frame 2) Trolley
Xddot22 = Xddots(8);
Xddot32 = Xddots(9);

Xddot13 = Xddots(25);           % Acceleration Body 3 (frame 5) Wire
Xddot23 = Xddots(26);
Xddot33 = Xddots(27);

Xddot14 = Xddots(31);           % Acceleration Body 4 (frame 6) Lifting load
Xddot24 = Xddots(32);
Xddot34 = Xddots(33);

% Xddot15 = Xddots(25);           % Acceleration Frame 7 Body 5 
% Xddot25 = Xddots(26);
% Xddot35 = Xddots(27);
% 
% Xddot16 = Xddots(31);           % Acceleration Frame 8 Body 6 
% Xddot26 = Xddots(32);
% Xddot36 = Xddots(33);

% Xddot17 = Xddots(37);           % Acceleration Frame 7 
% Xddot27 = Xddots(38);
% Xddot37 = Xddots(39);
% 
% 
% Xddot18 = Xddots(43);           % Acceleration Frame 8 
% Xddot28 = Xddots(44);
% Xddot38 = Xddots(45);
% 
% Xddot19 = Xddots(49);
% Xddot29 = Xddots(50);
% Xddot39 = Xddots(51);
% 
% Xddot110 = Xddots(55);
% Xddot210 = Xddots(56);
% Xddot310 = Xddots(57);




%% Pre-calculating "pull back and fourth" matrices
%  There are 14 frames, but 9 bodies. Each wire segment consists of 3
%  frames with their respected degree of freedom. This is to make the derivative
%  of the B-matrix easier. 

% FromIto2  = R21T * R1T;
% From1to2  = R21T;
% From3to2  = R21 * R32;
% From4to2  = R21 * R32 * R43;
% From5to2  = R21 * R32 * R43 * R54;
% From6to2  = R21 * R32 * R43 * R54 * R65;

FromIto2  = R21T * R1T;
From1to2  = R21T;
From3to2  = R32;
From4to2  = R32 * R43;
From5to2  = R32 * R43 * R54;
From6to2  = R32 * R43 * R54 * R65;
% From7to2  = R21 * R32 * R43 * R54 * R65 * R76;
% From8to2  = R21 * R32 * R43 * R54 * R65 * R76 * R87;
% From9to2  = R21 * R32 * R43 * R54 * R65 * R76 * R87 * R98;
% From10to2 = R21 * R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109;
% From11to2 = R21 * R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110;
% From12to2 = R21 * R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211;
% From13to2 = R21 * R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312;
% From15to3 = R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514;
% From16to3 = R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514 * R1615;
% From17to3 = R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514 * R1615 * R1716;


%%   Equation from FBD (Free Body Diagram)  The formulas below assume the wire is one body
%    Note: See Word document for Load cell calculations for future work

%  BODY 1 Boom (Frame 1)

    % Equation2 = R32T * R21T * R1T * [0; 0; -m2*g] + R32T * R21T *[-Re11 ; -Re21; -Re31] + R32T * [-Re12; -Re22; -Re32] == R32T * R21T * R1T * m2* [Xddot12; Xddot22; Xddot32];
 
    Equation1 = FromIto2 * [0; 0; -m1*g] + FromIto2 * [Re1I ; Re2I; Re3I] + From1to2 * [-Re11; -Re21; -Re31] == FromIto2 * m1* [Xddot11; Xddot21; Xddot31];
    simplify(Equation1);

eq1 = Equation1(1);
eq2 = Equation1(2);
eq3 = Equation1(3);


% Body 2 Cart (Frame 2)
    Equation2 = FromIto2 * [0; 0; -m2*g] + From1to2 * [Re11; Re21; Re31] + [Re12; Re22; Re32] == FromIto2 * m2 * [Xddot12; Xddot22; Xddot32];
    simplify(Equation2);

eq4 = Equation2(1);
eq5 = Equation2(2);
eq6 = Equation2(3);

% Body 3 First Wiresegment (Stacked frames )(Frame 5)
    Equation3 = FromIto2 * [0; 0; -m3*g] + [-Re12; -Re22; -Re32] + From5to2 * [Re15; Re25; Re35] == FromIto2 * m3 * [Xddot13; Xddot23; Xddot33];
    simplify(Equation3);

eq7 = Equation3(1);
eq8 = Equation3(2);
eq9 = Equation3(3);

% Body 4 Lifting Load (+hook) (Frame 6)
    Equation4 = FromIto2 * [0; 0; -m4*g-mHook*g]  + From5to2 * [-Re15; -Re25; -Re35] == FromIto2 * m4 * [Xddot14; Xddot24; Xddot34];
    simplify(Equation4);

eq10 = Equation4(1);
eq11 = Equation4(2);
eq12 = Equation4(3);


%% SOLVING SYSTEM OF EQUATIONS
 % extracting the equations by index from "Equations"

eqs     =  [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, ...
             eq10, eq11, eq12];

unknowns =  [Re1I, Re2I, Re3I, ...
             Re11, Re21, Re31, ...
             Re12, Re22, Re32, ... 
             Re13, Re23, Re33, ... 
             ];

sol = solve(eqs, unknowns);

solArray = struct2array(sol);
reactions_1_wireSeg = solArray(6); %Newtons
%reactions_1_wireSeg = (solArray(6)/g-m2-m3-mHook); % Output in kg

end
