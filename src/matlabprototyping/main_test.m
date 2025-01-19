clr()

% Create Sym variables
syms t real

% Create frames
framecount = 5;
n_theta = framecount;
n_length = framecount;

%Create sym vars within frames
theta = sym('theta',[1, n_theta],'real');
thetadot = sym('thetadot',[1, n_theta],'real');
thetaddot = sym('thetaddot',[1, n_theta],'real');
length = sym('L',[n_length,3],'real');


for i = 1:framecount
    frames(i) = makeFrame();
    frames(i).framenumber = i; 
    frames(i).rotationaxis = 3;
    frames(i).rotationvar = theta(i);
    frames(i).rotationvardot = thetadot(i); 
    frames(i).Qcoordinates(1,1) = theta(i);
    frames(i).Qcoordinates(2,1) = thetadot(i);
end

makeEdot(frames(3),frames)
