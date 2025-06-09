%% This script was made with the help of Chat GPT o3-mini-high


% Defining the subfolder
subfolder = 'C:\GitHub\offshorecranesim\src\LoadCell\js';

eqsStruct = Reaction_3_link(frames);
fields = fieldnames(eqsStruct);
eqsForJS = struct();
for i = 1:length(fields)
    eqsForJS.(fields{i}) = char(eqsStruct.(fields{i}));
end

% Convert the structure to JSON
jsonText = jsonencode(eqsForJS);

% Build file path
filepath = fullfile(subfolder, 'equations_3_link.json');

% Save to the file using the complete file path
fid = fopen(filepath, 'w');
if fid == -1
    error('Failed to open file: %s', filepath);
end
fprintf(fid, '%s', jsonText);
fclose(fid);q