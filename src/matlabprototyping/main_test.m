clear
clc
theta = sym('theta',[1, 5],'real');
thetadot = sym('thetadot',[1, 5],'real');
thetaddot = sym('thetaddot',[1, 5],'real');

frames(1) = Frame(1,3,theta(1),[theta(1),thetadot(1),thetaddot(1)],0)
frames(2) = Frame(2,3,theta(2),[theta(2),thetadot(2),thetaddot(2)],0)



% Bdot = makeBdot(2,frames)
% D = makeD(2,frames)
% M = makeM(2,frames)
% Mstar = B' * M * B;             Mstar = simplify(Mstar);
% Nstar = B' * (M*Bdot + D*M*B);  Nstar = simplify(Nstar);
% 
% eqs_of_motion = Mstar * Q(:,3) + Nstar*Q(:,2); eqs_of_motion = simplify(eqs_of_motion)





