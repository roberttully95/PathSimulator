function data = readJson(file)
%READJSON Summary of this function goes here
%   Detailed explanation goes here
    
    % Read .json file into struct
    fid = fopen(file); 
    struct_ = jsondecode(char(fread(fid, inf)'));
    fclose(fid); 
    
    % Determine the number of objects.
    objects = struct_.features;
    n = size(objects, 1);
    
    % Copy the map-level properties.
    data.properties = struct_.properties;

    % Init obstacles
    data.paths = Path.empty(n, 0);

    % Parse through objects
    j = 1;
    for i = 1:n
        obj = objects(i);
        coords = squeeze(obj.geometry.coordinates);
        switch obj.type
            case "Path"
                data.paths(j) = Path(coords(:, 1), coords(:, 2));
                j = j + 1;
            otherwise
                warning("Invalid map object provided.")
        end
        if j == 3
            break;
        end
    end
end

