function filename = sanitizeFilename(filename)
% allow only alphanumeric characters, replace others with underscore
indices_invalid = regexp(filename,  '[^a-zA-Z0-9 -]');
filename(indices_invalid) = '_';