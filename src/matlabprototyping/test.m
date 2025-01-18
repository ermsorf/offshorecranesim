syms theta1 theta2 thetadot1 thetadot2 L1 L2 t real
framecount = 3;
n_theta = framecount; theta = sym('theta',[1, n_theta]); thetadot = sym('thetadot',[1, n_theta]);

for i = 1:framecount
    frames(i) = makeFrame();
    frames(i).rotationaxis = 3;
    frames(i).rotationvar = theta(i);
    frames(i).Qcoordinates(1,i) = theta(i);
    frames(i).Qcoordinates(2,i) = thetadot(i);
end

frames(1).joint2cm = [L1/2,0,0];


% B = sym([]);
% 
% for i = 1:framecount
%     E = makeEv(frames(i).cm2joint) * makeEr(frames(i)) * makeEv(frames(i).joint2cm);
%     E_prediff = subs(E,frames(i).Qcoordinates(1,i), t * frames(i).Qcoordinates(2,i))
%     E_postdiff = diff(E_prediff,t)
%     E_dot = subs(E_postdiff,t * frames(i).Qcoordinates(2,i), frames(i).Qcoordinates(1,i))
%     preB = E_dot(1:3,4)
%     
% 
%     for j = 1:3
%     coeffsList = coeffs(preB(j),frames(i).Qcoordinates(2,i))
%     end
%     
% end
% 
% preB = E1dot(1:3,4);
% 
% B = sym(zeros(3,1)); % Initialize B as a symbolic column vector
% 
% for i = 1:3
%     coeffsList = coeffs(preB(i), thetadot1, 'All'); % Get all coefficients
%     if isempty(coeffsList)
%         B(i,1) = 0; % Assign 0 if the coefficient does not exist
%     else
%         B(i,1) = coeffsList(1); % Extract first coefficient
%     end
% end



