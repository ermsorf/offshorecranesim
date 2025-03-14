function configexport(system, filename)
    fields = fieldnames(system);
    for i = 1:numel(fields)
        field = fields{i};
        matrix = system.(field);
        [nRows, nCols] = size(matrix);
        nestedArray = cell(1, nRows);

        if strcmp(field, "rotations") || strcmp(field, "initconditions")
            continue; % Ignore special case matrices
        end

        % Default behavior for other matrices
        for row = 1:nRows
            rowArray = cell(1, nCols);
            for col = 1:nCols
                exprStr = formatExpression(char(matrix(row, col)));
                varsList = string(symvar(matrix(row, col)));
                
                % Filter variables based on system.Qcoordinates
                varsList = varsList(ismember(varsList, string(system.Qcoordinates)));
                
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
    expr = strrep(expr, '^', '**');
    
    % Use regex to replace whole function names only
    expr = regexprep(expr, '\<asin\>', 'Math.asin');
    expr = regexprep(expr, '\<acos\>', 'Math.acos');
    expr = regexprep(expr, '\<atan\>', 'Math.atan');
    expr = regexprep(expr, '\<sin\>', 'Math.sin');
    expr = regexprep(expr, '\<cos\>', 'Math.cos');
    expr = regexprep(expr, '\<tan\>', 'Math.tan');

    formattedExpr = expr;
end
