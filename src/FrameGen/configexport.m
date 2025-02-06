function configexport(system)
    % Convert all matrices in system struct to structured format
    matrixNames = fieldnames(system);
    
    for i = 1:numel(matrixNames)
        name = matrixNames{i};
        if ~strcmp(name, 'variables')  % Skip the variable list
            exprMatrix = system.(name);
            system.(name) = arrayfun(@(expr) struct(...
                'expr', char(expr), 'vars', symvar(expr)), exprMatrix, 'UniformOutput', false);
        end
    end

    % Convert variables to strings
    if isfield(system, 'variables')
        system.variables = arrayfun(@char, system.variables, 'UniformOutput', false);
    end

    % Save as JSON
    jsonStr = jsonencode(system);
    fid = fopen('config.json', 'w');
    fprintf(fid, '%s', jsonStr);
    fclose(fid);

    fprintf('System configuration exported to config.json\n');
end
