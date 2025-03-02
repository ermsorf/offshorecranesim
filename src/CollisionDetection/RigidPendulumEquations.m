clear
clc

%% Theta is precession, phi is nutation, and psi is spin.

syms theta psi phi thetadot psidot phidot thetaddot psiddot phiddot L a b c m g real

% Rotation Matrices
R1 = [1,  0,           0;
      0, cos(theta), -sin(theta);
      0, sin(theta),  cos(theta)];

R2 = [cos(phi),  0, sin(phi);
           0,     1,      0;
      -sin(phi),  0, cos(phi)];

R3 = [1,         0,          0;
      0, cos(psi), -sin(psi);
      0, sin(psi),  cos(psi)];

% Parameters
a = 5;
b = 10;
c = 7;
g = -9.81;
L = -30;
m = 400;

% Inertia matrix components about the center of mass:
I1 = (1/12) * m * (a^2 + c^2) + L^2 * m;  
I2 = (1/12) * m * (a^2 + b^2) + L^2 * m;  
I3 = (1/12) * m * (a^2 + a^2);

% Assemble the inertia matrix:
I = [I1,   0,    0;
     0,    I2,   0;
     0,    0,   I3];

% Construct the angular velocity vector
omega = [phidot * cos(psi) + thetadot * sin(phi) * sin(psi);
         - phidot * sin(psi) + thetadot * sin(phi) * cos(psi);
         thetadot * cos(phi) + psidot];

% Compute kinetic and potential energy
KE = simplify((1/2) * omega.' * I * omega);
UE = simplify(m * g * (L/2) * (1 - cos(phi)));

% Compute partial derivatives of KE and UE
dKE_dpsidot_t = diff(KE, psidot);
dKE_dthetadot_t = diff(KE, thetadot);
dKE_dphidot_t = diff(KE, phidot);
dKE_dphi = diff(KE, phi);
dKE_dpsi = diff(KE, psi);
dUE_dphi = diff(UE, phi);

% Compute time derivatives of momentum terms (Euler-Lagrange equations)
EL_theta = thetaddot - diff(KE, theta);
EL_phi = phiddot - diff(KE, phi) + dUE_dphi;
EL_psi = psiddot - diff(KE, psi);

% Display results
disp('Euler-Lagrange Equations:')
disp('EL_theta ='), pretty(simplify(EL_theta))
disp('EL_phi ='), pretty(simplify(EL_phi))
disp('EL_psi ='), pretty(simplify(EL_psi))


