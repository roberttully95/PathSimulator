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
    
    % Set map type
    data.type = struct_.type;
    
    % Copy the map-level properties.
    data.properties = struct_.properties;

    % Init obstacles
    data.paths = Path.empty(n, 0);
    data.triangles = Triangle.empty(n, 0);
    
    % Parse through objects
    jPath = 1;
    jTri = 1;
    for i = 1:n
        obj = objects(i);
        coords = squeeze(obj.geometry.coordinates);
        switch obj.type
            case "Path"
                data.paths(jPath) = Path(coords(:, 1), coords(:, 2));
                jPath = jPath + 1;
            case "Triangle"
                data.triangles(jTri) = Triangle(coords(1, :), coords(2, :), coords(3, :), 'Vertices');
                jTri = jTri + 1;
            otherwise
                warning("Invalid map object provided.")
        end
    end
end

