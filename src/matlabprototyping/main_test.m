syms t real

framecount = 2;
n_theta = framecount; 
theta = sym('theta',[1, n_theta],'real'); thetadot = sym('thetadot',[1, n_theta],'real')
n_length = framecount; directions = 3; 

length = sym('L',[n_length,directions],'real')  ;

for i = 1:framecount
    frame(i) = makeFrame();
    frame(i).rotationaxis = 3;
    frame(i).rotationvar = theta(i);
    frame(i).rotationvardot = thetadot(i); 
    frame(i).joint2cm = [length(i,1), length(i,2), length(i,3)];
    frame(i).cm2joint = [0,0,0];

    frame(i).Qcoordinates(1,i) = theta(i);
    frame(i).Qcoordinates(2,i) = thetadot(i);

end

for i = 1:framecount;
    makeEdot(frame(i))
end