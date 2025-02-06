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
            O = makeO(i,framelist);
            Ovec = unskew(O(1:3,1:3));
            for q = 1:Qsize(2) %% For every q in q list
                for direction = 1:3 %% For every direction
                    %% Positions
                    [cp, tp] = coeffs(posvec(direction), Q(2,q)); % Get coefficients and terms
                    if isempty(cp) || isempty(tp) % Handle empty symbolic vectors
                        B((6*i-6+direction), q) = sym(0);
                    else
                        idx = find(tp == Q(2,q), 1); % Find the coefficient of Q(q)^1
                        if ~isempty(idx)
                            B((6*i-6+direction), q) = cp(idx);
                        else
                            B((6*i-6+direction), q) = sym(0); % Store 0 if Q(q) is absent
                        end
                    end
                    %% Omegas
                    [cr, tr] = coeffs(Ovec(direction), Q(2,q)); % Get coefficients and terms
                    if isempty(cr) || isempty(tr) % Handle empty symbolic vectors
                        B((6*i-3+direction), q) = sym(0);
                    else
                        idx = find(tr == Q(2,q), 1); % Find the coefficient of Q(q)^1
                        if ~isempty(idx)
                            B((6*i-3+direction), q) = cr(idx);
                        else
                            B((6*i-3+direction), q) = sym(0); % Store 0 if Q(q) is absent
                        end
                    end
                    
                end
            end         
        end

end
