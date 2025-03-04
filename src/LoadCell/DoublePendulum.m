%%  This is a test for calculation of reactionforces of a double pendulum.

function DoublePendulum(frames)

syms t thetad phid Re11 Re21 Re31 Xddot12 Xddot22 Xddot32 m1 m2 m3 g Xddot11 Xddot21 Xddot31 ReI1  ReI2   ReI3 real

%%  THE VARIABLES:
%   t         - time
%   thetad    - theta dot
%   phid      - phi dot 
%   Re11      - Reaction force in e1 direction in 1 frame
%   Re21      - Reaction force in e2 direction in 1 frame
%   Re31      - Reaction force in e3 direction in 1 frame
%   Xddot11   - X dobbel dot (acceleration) in 1 direction in 1-frame   (notice notation)
%   Xddot21   - X dobbel dot (acceleration) in 2 direction in 1-frame
%   Xddot31   - X dobbel dot (acceleration) in 3 direction in 1-frame

%   m2        - mass of the cart
%   m3        - mass of cargo

%   Xddot12   - X dobbel dot (acceleration) in 1 direction in 2-frame   (notice notation)
%   Xddot22   - X dobbel dot (acceleration) in 2 direction in 2-frame
%   Xddot32   - X dobbel dot (acceleration) in 3 direction in 2-frame


%   MASSES IN [kg]:
m1 = frames(1).mass;
m2 = frames(2).mass;

%   GRAVITY [m/s^2]
g = 9.81;

% THE ROTATION MATRICES
R1 = frames(1).makeEr();
R1 = R1(1:3,1:3);
R21 = frames(2).makeEr();
R21 = R21(1:3,1:3);
R32 = frames(3).makeEr();
R32 = R32(1:3,1:3);

%%  COMPONENTS FROM B MATRIX
%   Need to express Xddot in terms of the generalized coordinates. 
%   accessing B-matrix and extracting terms. 
B    = frames(2).makeB(frames)
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

Xddot12 =   Xddots(1);
Xddot22 =   Xddots(2);
Xddot32 =   Xddots(3);


%%  Body 1 (eq1, eq2, eq3)
%% consider using matrix notation for position within the vectors 'ij'
   Equation1 = [ReI1;  ReI2;   ReI3]   +  [0;  -m1*g;  0] + [Re11; Re21; Re31] ==  R1*m1*[Xddot11; Xddot21; Xddot31];

eq1 = Equation1(1);
eq2 = Equation1(2);
eq3 = Equation1(3);



%%  Body 2 (eq4, eq5, eq6)
   Equation2 = [0; -m2*g;  0] + R1*[-Re11; -Re21;  -Re31]    ==  R1*R21*m2*[Xddot12;    Xddot22;    Xddot32];

eq4 = Equation2(1);
eq5 = Equation2(2);
eq6 = Equation2(3);


%% System of equations using struct

 sol = solve([eq1,eq2,eq3,eq4,eq5,eq6],[Re11,Re21,Re31,ReI1,ReI2,ReI3]);















