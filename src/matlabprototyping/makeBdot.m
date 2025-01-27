function Bdot = makeBdot(framenumber,framelist)
% Get Bdot
%   Detailed explanation goes here
    B = makeB(framenumber,framelist);
    Q = getQs(framenumber, framelist);  % Get Q values
    
    syms t real
    
    prediff = B;
    for i = 1:height(Q)  % Replace Q with differentiable variables
        prediff = subs(prediff, Q(i,2), t*Q(i,3));
        prediff = subs(prediff, Q(i,1), t*Q(i,2));
    end

    postdiff = diff(prediff, t);  % Differentiate with respect to t
    Bdot = postdiff;
    
    for i = 1:height(Q)  % Replace t*Q(2,i) with Q(1,i)
        Bdot = subs(Bdot, t*Q(i,3), Q(i,2));
        Bdot = subs(Bdot, t*Q(i,2), Q(i,1));
    end
    
    Bdot = simplify(Bdot)  % Simplify result
end


