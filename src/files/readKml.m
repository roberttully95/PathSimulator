function dataStruct = readKml(kmlFile)
    %READKML Takes an input kml file and parses it into a structure
    %containing all of the kml data.
    
    % Parse KML File
    [FID, msg] = fopen(kmlFile, 'rt');
    if FID<0
        error(msg)
    end
    txt = fread(FID, 'uint8=>char')';
    fclose(FID);

    % Init
    expr = '<Placemark.+?>.+?</Placemark>';
    objectStr = regexp(txt,expr,'match');
    nObjects = length(objectStr);
    dataStruct(nObjects) = struct();

    % Iterate
    for i = 1:nObjects

        % Find Object Name Field
        bucket = regexp(objectStr{i},'<name.*?>.+?</name>','match');
        if isempty(bucket)
            name = 'undefined';
        else
            % Clip off flags
            name = regexprep(bucket{1},'<name.*?>\s*','');
            name = regexprep(name,'\s*</name>','');
        end

        % Identify Object Type
        if ~isempty(regexp(objectStr{i},'<Point', 'once'))
            geometry = 'Point';
        elseif ~isempty(regexp(objectStr{i},'<LineString', 'once'))
            geometry = 'LineString';
        elseif ~isempty(regexp(objectStr{i},'<Polygon', 'once'))
            geometry = 'Polygon';
        else
            geometry = '';
        end

        % Find Coordinate Field
        bucket = regexp(objectStr{i},'<coordinates.*?>.+?</coordinates>','match');

        % Clip off flags
        coordStr = regexprep(bucket{1},'<coordinates.*?>(\s+)*','');
        coordStr = regexprep(coordStr,'(\s+)*</coordinates>','');

        % Split coordinate string by commas or white spaces, and convert string
        % to doubles
        coordMat = str2double(regexp(coordStr,'[,\s]+','split'));

        % Rearrange coordinates to form an x-by-3 matrix
        [m,n] = size(coordMat);
        coordMat = reshape(coordMat,3,m*n/3)';

        % define polygon in clockwise direction, and terminate
        coordMat = toClockwise(coordMat);
        
        % Create items
        Lon = coordMat(:, 1, :);
        Lat = coordMat(:, 2, :);
        Alt = coordMat(:, 3, :);
        Bounds.minLon = min(Lon);
        Bounds.maxLon = max(Lon);
        Bounds.minLat = min(Lat);
        Bounds.maxLat = max(Lat);
        
        % Create structure
        dataStruct(i).Name = name;
        dataStruct(i).Geometry = geometry;
        dataStruct(i).Lon = Lon;
        dataStruct(i).Lat = Lat;
        dataStruct(i).Alt = Alt;
        dataStruct(i).BoundingBox = Bounds;
    end
    
    function val = toClockwise(val)
        %TOCLOCKWISE Orients a set of points in a clockwise
        %direction.
        x = val(:, 1);
        y = val(:, 2);
        xcent = mean(x);
        ycent = mean(y);
        [~, order] = sort(atan2(y - ycent, x - xcent), 'descend');
        val = unique(val(order, :), 'rows');
    end
end