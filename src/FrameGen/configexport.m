function configexport(system, filename)
    fields = fieldnames(system);
    Qdeepcopy = system.Qcoordinates;
    trigMap = containers.Map;
    
    for i = 1:numel(fields)
        field = fields{i};
        matrix = system.(field);
        [nRows, nCols] = size(matrix);
        nestedArray = cell(1, nRows);

        if strcmp(field, "rotations") || strcmp(field, "initconditions") || strcmp(field, "info")
            continue; % Ignore special case matrices
        end

        % Default behavior for other matrices
        for row = 1:nRows
            rowArray = cell(1, nCols);
            for col = 1:nCols
                exprStr = char(matrix(row, col));
                [exprStr, trigMap] = extractTrigFunctions(exprStr, trigMap);
                
                % Extract variables from original expression
                varsList = string(symvar(matrix(row, col)));  
                qCoordsStr = string(Qdeepcopy);  
                varsList = varsList(ismember(varsList, qCoordsStr));  
                
                % Extract trig terms from substituted exprStr
                trigTerms = regexp(exprStr, 'trig\d+', 'match');
                trigTerms = string(trigTerms);
                varsList = [varsList, trigTerms];
                varsList = unique(varsList); % Remove duplicates
                
                % Wrap all variables (Q and trig) in parentheses
                for v = varsList
                    exprStr = regexprep(exprStr, ['(?<![\w$])' char(v) '(?![\w$])'], ['(' char(v) ')']);
                end

                if isempty(varsList)
                    varsList = [];
                end

                rowArray{col} = struct('expr', exprStr, 'vars', varsList);
            end
            nestedArray{row} = rowArray;
        end

        system.(field) = nestedArray;
    end

    % Convert trigMap to an array of structs for JSON export
    trigFunctions = cellfun(@(k, v) struct('name', v, 'expr', k), keys(trigMap), values(trigMap), 'UniformOutput', false);
    system.trigFunctions = trigFunctions;

    jsonStr = jsonencode(system, 'PrettyPrint', true);
    fid = fopen(filename, 'w');
    fwrite(fid, jsonStr);
    fclose(fid);
end

function [formattedExpr, trigMap] = extractTrigFunctions(expr, trigMap)
    expr = strrep(expr, '^', '**');
    
    trigPatterns = {'sin', 'cos', 'tan', 'asin', 'acos', 'atan'};
    for i = 1:numel(trigPatterns)
        pattern = trigPatterns{i};
        matches = regexp(expr, [pattern '\(([^)]+)\)'], 'match');
        
        for j = 1:numel(matches)
            trigExpr = ['Math.' matches{j}];
            if ~isKey(trigMap, trigExpr)
                trigKey = sprintf('trig%02d', trigMap.Count + 1); % Leading zero formatting
                trigMap(trigExpr) = trigKey;
            end
            expr = strrep(expr, matches{j}, trigMap(trigExpr));
        end
    end
    expr = regexprep(expr, '(trig\d+)', '($1)');
    
    formattedExpr = expr;
end