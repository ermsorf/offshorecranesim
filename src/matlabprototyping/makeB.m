function B = makeB(framenumber, framelist)
    % Constructs B matrix 
    %
    % Usage:
    %   B = makeB(5, frames)

    Q = getQs(framenumber, framelist);
    Qsize = size(Q);
    % Initialize B matrix
        B = sym(zeros(framenumber*6, Qsize(2)));

        for i = 1:framenumber
            Edot = makeEdot(i,framelist);
            posvec = Edot(1:3,4);

            for q = 1:Qsize(2) %% For every q in q list
                for direction = 1:3
                    [c, t] = coeffs(posvec(direction), Q(q)); % Get coefficients and terms
            
                    if isempty(c) || isempty(t) % Handle empty symbolic vectors
                        coeffMatrix(direction, q) = sym(0);
                    else
                        idx = find(t == Q(q), 1); % Find the coefficient of Q(q)^1
                        if ~isempty(idx)
                            B( (6*i-6+direction)  , q) = c(idx);
                        else
                            B((6*i-6+direction),q) = sym(0); % Store 0 if Q(q) is absent
                        end
                    end
                end
                    
            end
        
        end

        

end