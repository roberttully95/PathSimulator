function data = readJson(file)
%READJSON Summary of this function goes here
%   Detailed explanation goes here
    
    % Read .json file into struct
    fid = fopen(file); 
    struct_ = jsondecode(char(fread(fid, inf)'));
    fclose(fid); 
    
    % Determine the number of paths.
    paths = struct_.features;
    n = size(paths, 1);
    
    % Assert
    if n ~= 2
        error("There must be exactly 2 paths provided (left, right)");
    end
    
    % Copy the map-level properties.
    data.properties = struct_.properties;

    % Init paths
    data.paths = Path.empty(n, 0);
    
    % Parse through objects
    for i = 1:n
        coords = squeeze(paths(i).geometry.coordinates);
        data.paths(i) = Path(coords(:, 1), coords(:, 2));
    end
end

