function Bdot = makeBdot(framenumber,framelist)
% Get Bdot
%   Detailed explanation goes here
    B = makeB(framenumber,framelist);
    Q = getQs(framenumber, framelist);  % Get Q values
    Qsize = size(Q);
    
    syms t real
    
    prediff = B;
    for i = 1:Qsize(2)  % Replace Q with differentiable variables
        prediff = subs(prediff, Q(2,i), t*Q(3,i));
        prediff = subs(prediff, Q(1,i), t*Q(2,i));
    end

    postdiff = diff(prediff, t);  % Differentiate with respect to t
    Bdot = postdiff;
    
    for i = 1:Qsize(2)  % Replace t*Q(2,i) with Q(1,i)
        Bdot = subs(Bdot, t*Q(3,i), Q(2,i));
        Bdot = subs(Bdot, t*Q(2,i), Q(1,i));
    end
    
    Bdot = simplify(Bdot)  % Simplify result
end
