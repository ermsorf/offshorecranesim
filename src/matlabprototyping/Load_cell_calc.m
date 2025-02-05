
function ReactionForces(framelist) 

syms t thetad phid Re11 Re21 Re31 Xddot12 Xddot22 Xddot32 m2 m3 g real


%masses in [kg]:
m2 = 11000;
m3 = 55000;

%Gravity [m/s^2]
g = 9.81;

% Get the rotation matrices for the formula

%   Body 2
%   Equation from FBD (Free Body Diagram)

%Equation = [0  ;   0;  -m2*g]  +   R1*R21*[Re11  ;  Re21;  Re31 - m3*g]  == R1*R21*m2  *  [Xddot12;    Xddot22;    Xddot32];


% simplifies equation
%simplify(Equation);

%extracting the equations by index from "Equations"
%eq1 = Equation(1);
%eq2 = Equation(2);
%eq3 = Equation(3);


%[Reaction1,Reaction2, Reaction3] = solve([eq1,eq2,eq3],[Re11,Re21,Re31])

R = framelist(1).makeEr()