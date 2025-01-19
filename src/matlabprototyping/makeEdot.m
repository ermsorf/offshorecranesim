function Edot = makeEdot(framenumber, frameslist)
    % Computes the time derivative of the transformation matrix E
    % Inputs:
    %   framenumber - Frame index
    %   frameslist  - List of frame data
    % Output:
    %   Edot - Time derivative of transformation matrix

    E = makeE(framenumber, frameslist);  % Get transformation matrix
    
    Q = getQs(framenumber, frameslist);  % Get Q values
    Qsize = size(Q);
    
    syms t real
    
    prediff = E;
    for i = 1:Qsize(2)  % Replace Q with differentiable variables
        prediff = subs(prediff, Q(1,i), t*Q(2,i));
    end

    postdiff = diff(prediff, t);  % Differentiate with respect to t
    Edot = postdiff;
    
    for i = 1:Qsize(2)  % Replace t*Q(2,i) with Q(1,i)
        Edot = subs(Edot, t*Q(2,i), Q(1,i));
    end
    
    Edot = simplify(Edot);  % Simplify result
end
