clear
clc

syms theta psi phi  thetadot_t psidot_t phidot_t L a b c m g

R1 = [ 1,         0,          0;
      0, cos(theta), -sin(theta);
      0, sin(theta),  cos(theta) ];

% Rotation about the y-axis by angle beta
R2 = [ cos(phi),  0, sin(phi);
           0,     1,      0;
      -sin(phi),  0, cos(phi) ];

R3 = [ 1,         0,          0;
      0, cos(psi), -sin(psi);
      0, sin(psi),  cos(psi); ]


% Parameters

a = 5
b = 10
c = 7
g = -9.81
L = -30

m = 400

% Inertia matrix components about the center-of-mass:
I1 = (1/12) * m * (a^2 + c^2) + L^2*m;  % Since a = b for a square base
I2 = (1/12) * m * (a^2 + b^2) + L^2*m;                   % Same as Ixx because the base is square
I3 = (1/12) * m * (a^2 + a^2); % Which simplifies to I3 = (1/6) * M * a^2

% Assemble the inertia matrix:
I = [I1,   0,    0;
     0,    I2    0;
     0,    0,   I3]



% Construct the angular velocity vector
omega = [phidot_t * cos(psi) + thetadot_t * sin(phi) * sin(psi);
          - phidot_t * sin(psi) + thetadot_t * sin(phi)*cos(psi);
         thetadot_t * cos(phi) + psidot_t                          ]




% Display the result
disp('Inertia matrix for the square box about its center of mass:');
disp(I);

E1 = R1 * R2

E2 = simplify(E1*R3 * [0;0;L])

Mm = [[m*eye(3),zeros(3)];[zeros(3),I]]

%z = cat(1,xd,omega)

KE = eye(6)

KE =  simplify((1/2) * omega.' * I * omega) % (1/2) * z.' * m * z  faller bort da den kun roterer i det faste punktet

UE = m * g *(L/2)*(1-cos(phi))

