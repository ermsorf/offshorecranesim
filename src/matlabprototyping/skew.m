function M = skew(vector)
% Skew a 1x3 or 3x1 vector
%   Detailed explanation goes here
    if size(vector) == [3,1]
        vector = vector' ; 
    end
    M = zeros(3,3);
    M(2,3) = -vector(1); M(3,2) =  vector(1);
    M(1,3) =  vector(2); M(3,1) = -vector(2);
    M(1,2) = -vector(3); M(2,1) =  vector(3);
end