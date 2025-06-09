%%  This is a test for calculation of reactionforces of a double pendulum.

function DoublePendulum(frames)

syms t thetad phid Re11 Re21 Re31 Xddot12 Xddot22 Xddot32 m1 m2 m3 g Xddot11 Xddot21 Xddot31 ReI1  ReI2   ReI3 real

%%  THE VARIABLES:

%   MASSES IN [kg]:
m1 = frames(1).mass;
m2 = frames(2).mass;

%   GRAVITY [m/s^2]
g = 9.81;

% THE ROTATION MATRICES
R1 = frames(1).makeE();
R1 = R1(1:3,1:3);
R21 = frames(2).makeE();
R21 = R21(1:3,1:3);
R32 = frames(3).makeE();
R32 = R32(1:3,1:3);

%%  COMPONENTS FROM B MATRIX
%   Need to express Xddot in terms of the generalized coordinates. 
%   accessing B-matrix and extracting terms. 
B    = frames(2).makeB(frames);
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
Xddots = Bdot*QddMatrix;
%   Xddots for body 1
Xddot11 =   Xddots(1);
Xddot21 =   Xddots(2);
Xddot31 =   Xddots(3);


%   Xddots for body 2
Xddot12 =   Xddots(7);
Xddot22 =   Xddots(8);
Xddot32 =   Xddots(9);



%%  Body 1 (eq1, eq2, eq3)
Equation1 = [ReI1;  ReI2;   ReI3]   +  [0;  -m1*g;  0] + [Re11; Re21; Re31] ==  m1*[Xddot11; Xddot21; Xddot31];
Equation1 = simplify(Equation1);         %Simplifies the equation

eq1 = Equation1(1);
eq2 = Equation1(2);
eq3 = Equation1(3);



%%  Body 2 (eq4, eq5, eq6)
Equation2 = [0; -m2*g;  0] + R1*[-Re11; -Re21;  -Re31]    ==  m2*[Xddot12;    Xddot22;    Xddot32];
Equation2 = simplify(Equation2);        %Simplifies the equation

eq4 = Equation2(1);
eq5 = Equation2(2);
eq6 = Equation2(3);


%% Solving system of equations 
[ReI1, ReI2, ReI3] = solve([eq1,eq2,eq3],[ReI1, ReI2, ReI3]);


[Re11, Re21, Re31] = solve([eq4,eq5,eq6],[Re11, Re21, Re31]);




%% Checking that the solution is valid 
% Substitute solutions into original equations for Body 1
eq1_check = simplify(subs(eq1, [ReI1, ReI2, ReI3], [ReI1, ReI2, ReI3]));
eq2_check = simplify(subs(eq2, [ReI1, ReI2, ReI3], [ReI1, ReI2, ReI3]));
eq3_check = simplify(subs(eq3, [ReI1, ReI2, ReI3], [ReI1, ReI2, ReI3]));

% Substitute solutions into original equations for Body 2
eq4_check = simplify(subs(eq4, [Re11, Re21, Re31], [Re11, Re21, Re31]));
eq5_check = simplify(subs(eq5, [Re11, Re21, Re31], [Re11, Re21, Re31]));
eq6_check = simplify(subs(eq6, [Re11, Re21, Re31], [Re11, Re21, Re31]));

% Display results
disp('Verification for Body 1:');
disp(['eq1_check: ', char(eq1_check)]); % Should simplify to 0
disp(['eq2_check: ', char(eq2_check)]); % Should simplify to 0
disp(['eq3_check: ', char(eq3_check)]); % Should simplify to 0

disp('Verification for Body 2:');
disp(['eq4_check: ', char(eq4_check)]); % Should simplify to 0
disp(['eq5_check: ', char(eq5_check)]); % Should simplify to 0
disp(['eq6_check: ', char(eq6_check)]); % Should simplify to 0
