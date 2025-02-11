function exportSymbolicSystem(system, filename)
    fields = fieldnames(system);
    for i = 1:numel(fields)
        field = fields{i};
        matrix = system.(field);
        [nRows, nCols] = size(matrix);
        % Build a 1D cell array where each element is a row (a 1D cell array)
        nestedArray = cell(1, nRows);
        for row = 1:nRows
            rowArray = cell(1, nCols);
            for col = 1:nCols
                exprStr = formatExpression(char(matrix(row, col)));
                varsList = string(symvar(matrix(row, col)));
                if isempty(varsList)
                    varsList = [];
                end
                rowArray{col} = struct('expr', exprStr, 'vars', varsList);
            end
            nestedArray{row} = rowArray;
        end
        system.(field) = nestedArray;
    end

    jsonStr = jsonencode(system, 'PrettyPrint', true);
    fid = fopen(filename, 'w');
    fwrite(fid, jsonStr);
    fclose(fid);
end

function formattedExpr = formatExpression(expr)
    % Replace MATLAB-style with JavaScript-style expressions
    expr = strrep(expr, '^', '**');
    expr = strrep(expr, 'sin', 'Math.sin');
    expr = strrep(expr, 'cos', 'Math.cos');
    formattedExpr = expr;
end
