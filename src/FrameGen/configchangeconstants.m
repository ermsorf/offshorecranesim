% Load JSON file as a string
filename = 'data.json'; % Change to your actual file name
fid = fopen(filename, 'r');
jsonStr = fread(fid, '*char')';
fclose(fid);

% Define replacements (modify as needed)
replacements = {
    'oldVar1', 'newVar1';
    'oldVar2', 'newVar2'
};

% Perform replacements
for i = 1:size(replacements, 1)
    jsonStr = strrep(jsonStr, replacements{i, 1}, replacements{i, 2});
end

newfilename = 'newconfigname';
% Save modified JSON
fid = fopen(newfilename, 'w');
fwrite(fid, jsonStr, 'char');
fclose(fid);

fprintf('JSON file updated and saved as %s\n', newfilename);
