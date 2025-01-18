function V = unskew(matrix)
% Unskew SO3 matrix to vector

V = [matrix(3,2), matrix(1,3), matrix(2,1)];
end