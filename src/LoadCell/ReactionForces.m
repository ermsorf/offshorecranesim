

%   The load cells on the crane have a tendency to fail. This function will
%   give an estimate of the reaction forces in caused by the applied load.

%   Newtons third law (F = ma )will be used to calculate the reaction forces. 
%   Gravity will be accounted for in the inertial frame. 
%   The reaction forces caused by the applied load will be in the 1-frame.  

%   The function will calculate the matrices in the equation,
%   such that we get the equations for the reaction forces. 
 
function ReactionForces(frames) 

% clear; clc;
syms t thetad phid g real

syms m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 real

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


%   GRAVITY [m/s^2]
g = 9.81;


% THE ROTATION MATRICES AND THEIR TRANSPOSES
R1 = frames(1).makeEr();
R1 = R1(1:3,1:3);        %Rotation
R1T= R1.';              %Transpose

R21 = frames(2).makeEr();
R21 = R21(1:3,1:3);      %Rotation
R21T= R21.';             %Transpose

R32 = frames(3).makeEr();
R32 = R32(1:3,1:3);     %Rotation
R32T= R32.';             %Transpose

R43 = frames(4).makeEr();
R43 = R43(1:3,1:3);     %Rotation
R43T= R43.';             %Transpose

R54 = frames(5).makeEr();
R54 = R54(1:3,1:3);     %Rotation
R54T= R54.';             %Transpose

R65 = frames(6).makeEr();
R65 = R65(1:3,1:3);     %Rotation
R65T= R65.';             %Transpose

R76 = frames(7).makeEr();
R76 = R76(1:3,1:3);     %Rotation
R76T= R76.';             %Transpose

R87 = frames(8).makeEr();
R87 = R87(1:3,1:3);     %Rotation
R87T= R87.';             %Transpose

R98 = frames(9).makeEr();
R98 = R98(1:3,1:3);     %Rotation
R98T= R98.';             %Transpose

R109 = frames(10).makeEr();
R109 = R109(1:3,1:3);   %Rotation
R109T= R109.';           %Transpose


%%  COMPONENTS FROM B MATRIX
%   Need to express Xddot in terms of the generalized coordinates. 
%   accessing B-matrix and extracting terms. 
%   Bdot = frames(2).makeBdot(frames);
%   B = B(2,[1 2])



% % % %%  Q-COORDINATES
% % % %   The generalized coordinates
% % % %   Thetaddot
% % %     Qdd1 = frames(1).Qcoordinates(3);
% % %     Qdd2 = frames(2).Qcoordinates(3);
% % %     Qdd3 = frames(3).Qcoordinates(3);
% % %     Qdd4 = frames(4).Qcoordinates(3);
% % %     Qdd5 = frames(5).Qcoordinates(3);
% % %     Qdd6 = frames(6).Qcoordinates(3);
% % %     Qdd7 = frames(7).Qcoordinates(3);
% % %     Qdd8 = frames(8).Qcoordinates(3);
% % %     Qdd9 = frames(9).Qcoordinates(3);
% % %     Qdd10 = frames(10).Qcoordinates(3);
% % % 
% % % %   Column for the generalized coordinates:
% % %     QddMatrix = [Qdd1 ; Qdd2; Qdd3; Qdd4; Qdd5; Qdd6; Qdd7; Qdd8; Qdd9; Qdd10];


%% Q-COORDINATES
% Extract the generalized coordinates (Thetaddot) for frames 1 to 10
% Chat GPT was used to optimize the code

% numFrames = 10; % Define number of frames
% QddMatrix = sym(zeros(numFrames, 1)); % Preallocate for efficiency
% 
% for i = 1:numFrames
%     QddMatrix(i) = frames(i).Qcoordinates(3);
% end
% 
% QddMatrix;



%%  DEFINING Xddots UPDATE THIS, WHAT DOES IT DO????
%   Extracting the x dots from array.
% Bdot = frames(10).makeBdot();
% 
% Xddots = Bdot*QddMatrix;

%% Pre-calculating "pull back and fourth" matrices

FromItoThree  = R32T * R21T * R1T;
From1toThree  = R32T * R21T;
From5toThree  = R43 * R54;
From6toThree  = R43 * R54 * R65;
From7toThree  = R43 * R54 * R65 * R76;
From8toThree  = R43 * R54 * R65 * R76 * R87;
From9toThree  = R43 * R54 * R65 * R76 * R87 * R98;
From10toThree = R43 * R54 * R65 * R76 * R87 * R98 * R109;

%%   Equation from FBD (Free Body Diagram) 
%    Note: See Word document for Load cell calculations for future work

%% Body 1
    % Equation1 = R32T * R21T * R1T * [0; 0; -m1*g] + R32T * R21T * R1T * [Re1I; Re2I; Re3I] + R32T * R21T * [Re11; Re21; Re31] == R32T * R21T * R1T * m1 * [Xddot11; Xddot21; Xddot31];
    % simplify(Equation1);

    Equation1 = FromItoThree * [0; 0; -m1*g] + FromItoThree * [Re1I; Re2I; Re3I] + From1toThree * [Re11; Re21; Re31] == FromItoThree * [Xddot11; Xddot21; Xddot31];


eq1 = Equation1(1);
eq2 = Equation1(2);
eq3 = Equation1(3);


    
%%  BODY 2

    % Equation2 = R32T * R21T * R1T * [0; 0; -m2*g] + R32T * R21T *[-Re11 ; -Re21; -Re31] + R32T * [-Re12; -Re22; -Re32] == R32T * R21T * R1T * m2* [Xddot12; Xddot22; Xddot32];
    % simplify(Equation2);

    Equation2 = FromItoThree * [0; 0; -m2*g] + From1toThree * [-Re11 ; -Re21; -Re31] + R32T * [-Re12; -Re22; -Re32] == FromItoThree* m2* [Xddot12; Xddot22; Xddot32];
    

eq4 = Equation2(1);
eq5 = Equation2(2);
eq6 = Equation2(3);


%% Body 3
    % Equation3 = R32T * R21T * R1T * [0; 0; -m3*g] + [-Re13; -Re23; -Re33] + R32T * [Re12; Re22; Re32] == R32T * R21T * R1T * m3 * [Xddot13; Xddot23; Xddot33];
    % simplify(Equation3);

    Equation3 = FromItoThree * [0; 0; -m3*g] + [-Re13; -Re23; -Re33] + R32T * [Re12; Re22; Re32] == FromItoThree * m3 * [Xddot13; Xddot23; Xddot33];

eq7 = Equation3(1);
eq8 = Equation3(2);
eq9 = Equation3(3);

%% Body 4
    % Equation4 = R32T * R21T * R1T * [0; 0; -m4*g] + [Re13; Re23; Re33] + R43 * [-Re14; -Re24; -Re34] == R32T * R21T * R1T * m4 * [Xddot14; Xddot24; Xddot34];
    % simplify(Equation4);

    Equation4 = FromItoThree * [0; 0; -m4*g] + [Re13; Re23; Re33] + R43 * [-Re14; -Re24; -Re34] == FromItoThree * m4 * [Xddot14; Xddot24; Xddot34];

eq10 = Equation4(1);
eq11 = Equation4(2);
eq12 = Equation4(3);

%% Body 5
    % Equation5 = R32T * R21T * R1T * [0; 0; -m5*g] + R43 * R54 * [-Re15; -Re25; -Re35] + R43 * [Re14; Re24; Re34] == R32T * R21T * R1T * m5 * [Xddot15; Xddot25; Xddot35];
    % simplify(Equation5);
  
    Equation5 = FromItoThree * [0; 0; -m5*g] + From5toThree * [-Re15; -Re25; -Re35] + R43 * [Re14; Re24; Re34] == FromItoThree * m5 * [Xddot15; Xddot25; Xddot35];

eq13 = Equation5(1);
eq14 = Equation5(2);
eq15 = Equation5(3);

%% Body 6
    % Equation6 = R32T * R21T * R1T * [0; 0; -m6*g] + R43 * R54 * [Re15; Re25; Re35] + R43 * R54 * R65 * [-Re16; -Re26; -Re36] == R32T * R21T * R1T * m6 * [Xddot16; Xddot26; Xddot36];
    % simplify(Equation6);

    Equation6 = FromItoThree * [0; 0; -m6*g] + From5toThree * [Re15; Re25; Re35] + From6toThree * [-Re16; -Re26; -Re36] == FromItoThree * m6 * [Xddot16; Xddot26; Xddot36];
  
eq16 = Equation6(1);
eq17 = Equation6(2);
eq18 = Equation6(3);

%% Body 7
    % Equation7 = R32T * R21T * R1T * [0; 0; -m7*g] + R43 * R54 * R65 * R76 * [-Re17; -Re27; -Re37] + R43 * R54 * R65 * [Re16; Re26; Re36] == R32T * R21T * R1T * m7 * [Xddot17; Xddot27; Xddot37];
    % simplify(Equation7);
 
    Equation7 = FromItoThree * [0; 0; -m7*g] + From7toThree * [-Re17; -Re27; -Re37] + From6toThree * [Re16; Re26; Re36] == FromItoThree * m7 * [Xddot17; Xddot27; Xddot37];

eq19 = Equation7(1);
eq20 = Equation7(2);
eq21 = Equation7(3);

%% Body 8
    % Equation8 = R32T * R21T * R1T * [0; 0; -m8*g] + R43 * R54 * R65 * R76 * [Re17; Re27; Re37] + R43 * R54 * R65 * R76 * R87 * [-Re18; -Re28; -Re38] == R32T * R21T * R1T * m8 * [Xddot18; Xddot28; Xddot38];
    % simplify(Equation8);
  
    Equation8 = FromItoThree * [0; 0; -m8*g] + From7toThree * [Re17; Re27; Re37] + From8toThree * [-Re18; -Re28; -Re38] == FromItoThree * m8 * [Xddot18; Xddot28; Xddot38];

eq22 = Equation8(1);
eq23 = Equation8(2);
eq24 = Equation8(3);

%% Body 9

    % Equation9 = R32T * R21T * R1T * [0; 0; -m9*g] + R43 * R54 * R65 * R76 * R87 * R98 * [-Re19; -Re29; -Re39] + R43 * R54 * R65 * R76 * R87; * [Re18; Re28; Re38] == R32T * R21T * R1T * m9 * [Xddot19; Xddot29; Xddot39];

    Equation9 = FromItoThree * [0; 0; -m9*g] + From9toThree * [-Re19; -Re29; -Re39] + From8toThree * [Re18; Re28; Re38] == FromItoThree * m9 * [Xddot19; Xddot29; Xddot39];
    % simplify(Equation9);
  

eq25 = Equation9(1);
eq26 = Equation9(2);
eq27 = Equation9(3);

%% Body 10
    
    % Equation10 = R43 * R54 * R65 * R76 * R87 * R98 * R109; * [0; 0; -m10*g] + R43 * R54 * R65 * R76 * R87 * R98 * [Re19; Re29; Re39] == R32T * R21T * R1T * m10 * [Xddot110; Xddot210; Xddot310];
    Equation10 = From10toThree * [0; 0; -m10*g] + From9toThree * [Re19; Re29; Re39] == FromItoThree * m10 * [Xddot110; Xddot210; Xddot310];
    % simplify(Equation10);
   

eq28 = Equation10(1);
eq29 = Equation10(2);
eq30 = Equation10(3);


%%  SOLVING SYSTEM OF EQUATIONS
%   extracting the equations by index from "Equations"


% 
% eqns     =  [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, ...
%              eq10, eq11, eq12, eq13, eq14, eq15, eq16, eq17, eq18, ...
%              eq19, eq20, eq21, eq22, eq23, eq24, eq25, eq26, eq27, ...
%              eq28, eq29, eq30];
% 
% 
% unknowns =  [Re1I, Re2I, Re3I, ... 
%              Re11, Re21, Re31, ...
%              Re12, Re22, Re32, ... 
%              Re13, Re23, Re33, ... 
%              Re14, Re24, Re34, ... 
%              Re15, Re25, Re35, ... 
%              Re16, Re26, Re36, ... 
%              Re17, Re27, Re37, ... 
%              Re18, Re28, Re38, ... 
%              Re19, Re29, Re39,];
% 
% 
% 
% 
% solutions = solve(eqns, unknowns, 'ReturnConditions', true);
% 
% disp(solutions)

end

%%  Plot
% % Convert symbolic solutions to functions for numerical evaluation
% Reaction1_fun = matlabFunction(Reaction1, 'Vars', {t, thetad, phid});
% Reaction2_fun = matlabFunction(Reaction2, 'Vars', {t, thetad, phid});
% Reaction3_fun = matlabFunction(Reaction3, 'Vars', {t, thetad, phid});
% 
% % Define time range and parameter values for plotting
% time = linspace(0, 10, 100); % Time range [0, 10] with 100 points
% thetad_val = 1; % Example theta dot value
% phid_val = 2; % Example phi dot value
% 
% Reaction1_vals = Reaction1_fun(time, thetad_val, phid_val);
% Reaction2_vals = Reaction2_fun(time, thetad_val, phid_val);
% Reaction3_vals = Reaction3_fun(time, thetad_val, phid_val);
% 
% % Plot the reaction force vector components
% figure;
% plot3(Reaction1_vals, Reaction2_vals, Reaction3_vals, 'b', 'LineWidth', 1.5);
% grid on;
% xlabel('Re1');
% ylabel('Re2');
% zlabel('Re3');
% title('Reaction Force Vector');
% legend('Reaction Forces');
% end

