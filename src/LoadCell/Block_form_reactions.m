%% This script calculates the reaction force equations in block form in order to be used in the bachelor report. 

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

syms g real

syms m1 m2 m3 m4 m5 m6 mhook real

syms R1 R21 R32 R43 R54 R65 R76 R87 R98 R109 R1110 R1T R21T

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Reactions 1 link, block form

% THE ROTATION MATRICES AND THEIR TRANSPOSES
% R1 = frames(1).makeEr();
% R1 = R1(1:3,1:3);           %Rotation
% R1T= R1.';                  %Transpose
% 
% R21 = frames(2).makeEr();
% R21 = R21(1:3,1:3);         %Rotation
% R21T= R21.';                %Transpose
% 
% R32 = frames(3).makeEr();
% R32 = R32(1:3,1:3);         %Rotation
% R32T= R32.';                %Transpose
% 
% R43 = frames(4).makeEr();
% R43 = R43(1:3,1:3);         %Rotation
% R43T= R43.';                %Transpose
% 
% R54 = frames(5).makeEr();
% R54 = R54(1:3,1:3);         %Rotation
% R54T= R54.';                %Transpose
% 
% R65 = frames(6).makeEr();
% R65 = R65(1:3,1:3);         %Rotation
% R65T= R65.';                %Transpose


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

%%   Equation from FBD (Free Body Diagram)  The formulas below assume the wire is one body

%  BODY 1 Boom (Frame 1)
 
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
    Equation4 = FromIto2 * [0; 0; -m4*g-mhook*g]  + From5to2 * [-Re15; -Re25; -Re35] == FromIto2 * m4 * [Xddot14; Xddot24; Xddot34];
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
             Re15, Re25, Re35];


sol = solve(eqs, unknowns);
solArray = struct2array(sol);
reactions_1_wireSeg = solArray(6); %Newtons
%reactions_1_wireSeg = (solArray(6)/g-m2-m3-mHook); % Output in kg

