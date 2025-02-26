%   The load cells on the crane have a tendency to fail. This function will
%   give an estimate of the reaction forces in caused by the applied load.

%   Newtons third law (F = ma )will be used to calculate the reaction forces. 
%   Gravity will be accounted for in the inertial frame. 
%   The reaction forces caused by the applied load will be in the 1-frame.  

%   The function will calculate the matrices in the equation,
%   such that we get the equations for the reaction forces. 
%   We are interested in the reaction force on the cart that moves along
%   the boom, therefore we only solve the equations for Body 2. 


function ReactionForces(frames) 

syms t thetad phid g real

syms m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 real


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

%%  THE VARIABLES:
%   t         - time
%   thetad    - theta dot
%   phid      - phi dot 
%   Re11      - Reaction force in e1 direction in 1 frame
%   Re21      - Reaction force in e2 direction in 1 frame
%   Re31      - Reaction force in e3 direction in 1 frame
%   Xddot12   - X dobbel dot (acceleration) in 1 direction in 2-frame
%   Xddot22   - X dobbel dot (acceleration) in 2 direction in 2-frame
%   Xddot32   - X dobbel dot (acceleration) in 3 direction in 2-frame
%   m2        - mass of the cart
%   m3        - mass of cargo

%   MASSES IN [kg]:
m2 = frames(1).mass;
m3 = frames(2).mass;

%   GRAVITY [m/s^2]
g = 9.81;

%   APPLIED LOAD:
%   F = -m3*g

% THE ROTATION MATRICES
R1 = frames(1).makeEr();
R1 = R1(1:3,1:3)
R21 = frames(2).makeEr();
R21 = R21(1:3,1:3)
R32 = frames(3).makeEr();
R32 = R32(1:3,1:3)
R43 = frames(4).makeEr();
R43 = R43(1:3,1:3)


%%  COMPONENTS FROM B MATRIX
%   Need to express Xddot in terms of the generalized coordinates. 
%   accessing B-matrix and extracting terms. 
Bdot = frames(2).makeBdot(frames);
%B = B(2,[1 2])

%%  GENERALIZED COORDINATES
%   The generalized coordinates from 1 and 2 frame
%   Thetaddot: 
    Qdd1 = frames(1).Qcoordinates(3);
    Qdd2 = frames(2).Qcoordinates(3);
    
%   Column for the generalized coordinates:
    QddMatrix = [Qdd1 ; Qdd2];

%%  DEFINING Xddots
%   Extracting the x dots from array.

Xddots2 = Bdot*QddMatrix;

Xddot12 =   Xddots2(1);
Xddot22 =   Xddots2(2);
Xddot32 =   Xddots2(3);

%% Body 1


%%  BODY 2
%   Equation from FBD (Free Body Diagram) for the crane
    Equation = [0  ;   0;  -m2*g]  +   R1*R21*[Re11  ;  Re21;  Re31 - m3*g]  == R1*R21*m2  *  [Xddot12;    Xddot22;    Xddot32];

%% Body 3
%% Body 4
%% Body 5
%% Body 6
%% Body 7
%% Body 8
%% Body 9
%% Body 10

%   simplifies equation
simplify(Equation);

%%  SYSTEM OF EQUATIONS
%   extracting the equations by index from "Equations"
eq1 = Equation(1);
eq2 = Equation(2);
eq3 = Equation(3);

[Reaction1,Reaction2, Reaction3] = solve([eq1,eq2,eq3],[Re11,Re21,Re31]);


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

