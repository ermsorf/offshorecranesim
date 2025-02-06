%% Test code for å pendulum to insert data into the collision detection algorithm. 

% Length vector

L = [0;0,;-5]

% Velocity vector

V = [0;0;0]

% Side length load

sll = 2

% Mass load (kg)

ml = 10

% Load J matrix (cube for now)

Jl = [ (1/2) * ml * (2*sll^2), 0, 0;
       0, (1/2) * ml * (2*sll^2) , 0;
       0,0, (1/2) * ml * (2*sll^2)]

% Wire J matrix (Might not be nessesary) 

% Mass Wire

% Rotation matrix 1

% Rotation matrix 2

% Time step

function 

