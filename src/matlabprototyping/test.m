n_theta = 5; theta = sym('theta',[1, n_theta]); thetadot = sym('thetadot',[1, n_theta])
n_length = 5; directions = 3; length = sym('L',[n_length,directions])  ;

n = 3;
for i = 1:n;
    frames{i} = makeFrame();
    frames{i}.rotationaxis = i;
    frames{i}.rotationvar = theta(i);
    frames{i}.rotationvardot = thetadot(i);
    frames{i}.joint2cm = [length(i,1), length(i,2), length(i,3)];
    frames{i}.cm2joint = [i, i, i];
    makeR(frames{i})
end




