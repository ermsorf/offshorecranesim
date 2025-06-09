%% Reaction forces for the crane with 5 links. Could not finnish execution due to lack of memory. 

function sol=Reaction_5_link(frames)


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
syms Re111 Re211 Re311 real
syms Re112 Re212 Re312 real
syms Re113 Re213 Re313 real
syms Re114 Re214 Re314 real
syms Re115 Re215 Re315 real
syms Re116 Re216 Re316 real
syms Re117 Re217 Re317 real
syms Re118 Re218 Re318 real
syms Re119 Re219 Re319 real


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
syms Xddot111 Xddot211 Xddot311 real
syms Xddot112 Xddot212 Xddot312 real
syms Xddot113 Xddot213 Xddot313 real
syms Xddot114 Xddot214 Xddot314 real
syms Xddot115 Xddot215 Xddot315 real
syms Xddot116 Xddot216 Xddot316 real
syms Xddot117 Xddot217 Xddot317 real
syms Xddot118 Xddot218 Xddot318 real
syms Xddot119 Xddot219 Xddot319 real


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
m13 = frames(13).mass;
m14 = frames(14).mass;
m15 = frames(15).mass;
m16 = frames(16).mass;
m17 = frames(17).mass;
m18 = frames(18).mass;
m19 = frames(19).mass;



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
R1211T = R1211.';           %Transpose

R1312 = frames(13).makeEr();
R1312 = R1312(1:3,1:3);     %Rotation
R1312T = R1312.';           %Transpose

R1413 = frames(14).makeEr();
R1413 = R1413(1:3,1:3);     %Rotation
R1413T = R1413.';           %Transpose

R1514 = frames(15).makeEr();
R1514 = R1514(1:3,1:3);     %Rotation
R1514T = R1514.';           %Transpose

R1615 = frames(16).makeEr();
R1615 = R1615(1:3,1:3);     %Rotation
R1615T = R1615.';           %Transpose

R1716 = frames(17).makeEr();
R1716 = R1716(1:3,1:3);     %Rotation
R1716T = R1716.';           %Transpose

R1817 = frames(18).makeEr();
R1817 = R1817(1:3,1:3);     %Rotation
R1817T= R1817.';            %Transpose

R1918 = frames(19).makeEr();
R1918 = R1918(1:3,1:3);     %Rotation
R1918T= R1918.';            %Transpose

% %% Q-COORDINATES
% % Extract the generalized coordinates (Thetaddot) for frames 1 to 10
% % Chat GPT was used to optimize the code
% 
numFrames = 19; % Define number of frames
QddMatrix = sym(zeros(numFrames, 1)); % Preallocate for efficiency

for i = 1:numFrames
    QddMatrix(i) = frames(i).Qcoordinates(3);
end

QddMatrix;

% %%  DEFINING Xddots UPDATE THIS, 
% %   Extracting the x dots from array.
Bdot = frames(19).makeBdot(frames);

Xddots = Bdot*QddMatrix;

Xddot11 = Xddots(1);            % Acceleration Body 1 (frame 1)
Xddot21 = Xddots(2);
Xddot31 = Xddots(3);


Xddot12 = Xddots(7);            % Acceleration Body 2 (frame 2)
Xddot22 = Xddots(8);
Xddot32 = Xddots(9);

Xddot13 = Xddots(25);           % Acceleration Body 3 (Frame 5) Wire seg 1
Xddot23 = Xddots(26);
Xddot33 = Xddots(27);

Xddot14 = Xddots(43);           % Acceleration Body 4 (Account for frame 8 here) Wire seg 2
Xddot24 = Xddots(44);
Xddot34 = Xddots(45);

Xddot15 = Xddots(61);           % Acceleration Body 5 (Account for frame 11) Wire seg 3
Xddot25 = Xddots(62);
Xddot35 = Xddots(63);

Xddot16 = Xddots(79);           % Acceleration Body 6  (Account for frame 14) Wire seg 4
Xddot26 = Xddots(80);
Xddot36 = Xddots(81);

Xddot17 = Xddots(97);           % Acceleration Body 7 (frame 17) Wire seg 5
Xddot27 = Xddots(98);
Xddot37 = Xddots(99);

Xddot18 = Xddots(103);           % Acceleration Body 8 (frame 18)
Xddot28 = Xddots(104); 
Xddot38 = Xddots(105);

Xddot19 = Xddots(109);            % Acceleration Body 9 (frame 19)
Xddot29 = Xddots(110);
Xddot39 = Xddots(111);



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
From13to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312;
From14to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413;
From15to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514;
From16to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514 * R1615;
From17to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514 * R1615 * R1716;
From18to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514 * R1615 * R1716 * R1817;
From19to2 = R32 * R43 * R54 * R65 * R76 * R87 * R98 * R109 * R1110 * R1211 * R1312 * R1413 * R1514 * R1615 * R1716 * R1817 * R1918;

%% Equations 

% Body 1 (frame 1)
  Equation1 = FromIto2 * [0; 0; -m1*g] + FromIto2 * [Re1I; Re2I; Re3I] + From1to2 * [-Re11; -Re21; -Re31] == FromIto2 * m1* [Xddot11; Xddot21; Xddot31];

eq1 = Equation1(1);
eq2 = Equation1(2);
eq3 = Equation1(3);

%  BODY 2 (frame 2)
  Equation2 = FromIto2 * [0; 0; -m2*g] + From1to2 * [Re11 ; Re21; Re31] + [Re12; Re22; Re32] == FromIto2 * m2* [Xddot12; Xddot22; Xddot32];

eq4 = Equation2(1);
eq5 = Equation2(2);
eq6 = Equation2(3);

% BODY 3 (frame 5)
    Equation3 = FromIto2 * [0; 0; -m5*g] + [-Re12; -Re22; -Re32] + From5to2 * From5to2 * [Re13; Re23; Re33] == FromIto2 * m5 *[Xddot13;Xddot23;Xddot33];

eq7 = Equation3(1);
eq8 = Equation3(2);
eq9 = Equation3(3);

% Body 4 (frame 8)
  Equation4 = FromIto2 * [0; 0; -m8*g] + From5to2 * [Re13; Re23; Re33] + From8to2 * [-Re14; -Re24; -Re34] == FromIto2 * m8 * [Xddot14; Xddot24; Xddot34];

eq10 = Equation4(1);
eq11 = Equation4(2);
eq12 = Equation4(3);

% Body 5 (frame 11)
  Equation5 = FromIto2 * [0; 0; -m11*g] +  From8to2 * [Re14; Re24; Re34] + From11to2 * [-Re15; -Re25; -Re35] == FromIto2 * m11 * [Xddot15; Xddot25; Xddot35];

eq13 = Equation5(1);
eq14 = Equation5(2);
eq15 = Equation5(3);

% Body 6 (frame 14)
  Equation6 = FromIto2 * [0; 0; -m14*g] + From11to2 * [Re15; Re25; Re35] + From14to2 * [-Re16; -Re26; -Re36] == FromIto2 * m14 * [Xddot16; Xddot26; Xddot36];

eq16 = Equation6(1);
eq17 = Equation6(2);
eq18 = Equation6(3);

% Body 7 (frame 17)
Equation7 = FromIto2 * [0; 0; -m17*g] + From14to2 * [Re16; Re26; Re36] + From17to2 * [-Re17; -Re27; -Re37] == FromIto2 * m17 * [Xddot17; Xddot27; Xddot37];

eq19 = Equation7(1);
eq20 = Equation7(2);
eq21 = Equation7(3);

% Body 8 (frame 18)
Equation8 = FromIto2 * [0; 0; -m18*g] +  From17to2 * [Re17; Re27; Re37] + From18to2 * [-Re18; -Re28; -Re38]  == FromIto2 * m18 * [Xddot18; Xddot28; Xddot38];

eq22 = Equation8(1);
eq23 = Equation8(2);
eq24 = Equation8(3);

% Body 9 (frame 19)
Equation8 =  FromIto2 * [0; 0; -m19*g] +  From18to2 * [Re18; Re28; Re38]  == FromIto2 * m19 * [Xddot19; Xddot29; Xddot39];

eq25 = Equation8(1);
eq26 = Equation8(2);
eq27 = Equation8(3);

eqs     =  [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, ...
             eq10, eq11, eq12, eq13, eq14, eq15, eq16, eq17, eq18, ...
             eq19, eq20, eq21, eq22, eq23, eq24,eq25, eq26, eq27];
            

unknowns =  [Re1I, Re2I, Re3I, ... 
             Re11, Re21, Re31, ...
             Re12, Re22, Re32, ... 
             Re13, Re23, Re33, ... 
             Re14, Re24, Re34, ... 
             Re15, Re25, Re35, ...
             Re16, Re26, Re36, ...
             Re17, Re27, Re37, ...
             Re18, Re28, Re38];


sol = solve(eqs, unknowns);
end