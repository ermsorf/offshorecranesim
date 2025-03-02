%   The load cells on the crane have a tendency to fail. This function will
%   give an estimate of the applied load.

%   Newtons third law (F = ma )will be used to calculate the reaction forces. 
%   Gravity will be accounted for in the inertial frame. 
%   The reaction forces caused by the applied load will be in the 1-frame.  

%   The function will calculate the matrices in the equation,
%   such that we get the equations for the reaction forces. 
%   We are interested in the reaction force on the cart that moves along
%   the boom, therefore we only solve the equations for Body 2. 
 
function ReactionForces(framelist) 

syms t thetad phid Re11 Re21 Re31 Xddot12 Xddot22 Xddot32 m2 m3 g real

%%  The variables:
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

%   masses in [kg]: Remember to update when more frames are added to
%   Frame.m

%m2 = framelist(1).mass;
%m3 = framelist(2).mass;

m2=10;
m3=20;

%   Gravity [m/s^2]
g = 9.81;

%   applied load:
%F = -m3*g

% The Rotation matrices 
R1 = framelist(1).makeEr();
R1 = R1(1:3,1:3);
R21 = framelist(2).makeEr();
R21 = R21(1:3,1:3);
R32 = framelist(3).makeEr();
R32 = R32(1:3,1:3);


%   Body 2
%   Equation from FBD (Free Body Diagram)
Equation = [0  ;   0;  -m2*g]  +   R1*R21*[Re11  ;  Re21;  Re31 - m3*g]  == R1*R21*m2  *  [Xddot12;    Xddot22;    Xddot32];

% simplifies equation
simplify(Equation);

%extracting the equations by index from "Equations"
eq1 = Equation(1);
eq2 = Equation(2);
eq3 = Equation(3);

[Reaction1,Reaction2, Reaction3] = solve([eq1,eq2,eq3],[Re11,Re21,Re31])


%% To do 
%   Use masses from framelist
%
%  Extract q doubel dots into the equation.

