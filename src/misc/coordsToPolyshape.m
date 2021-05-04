function p = coordsToPolyshape(coords)
%COORDSTOPOLYSHAPE Takes an 'n x 2' array of coordinates (lat, lon) and
%converts them to meters using the lower left coordinate as the (0, 0)
%anchor point.
    
    % Get the lower left
    ll = [min(coords(:, 1)), min(coords(:, 2))];

    % Init the meters coordinates
    mCoords = NaN(size(coords));

    % Convert coordinates to meters
    for i = 1:size(coords, 1)

        th = atan2(coords(i, 2) - ll(2), coords(i, 1) - ll(1));
        meters = haversine(coords(i, :), ll);

        % Convert to meters from (0, 0)
        mCoords(i, :) = [meters * sin(th), meters * cos(th)];
    end
    
    % Create polyshape
    p = polyshape(mCoords(:, 1), mCoords(:, 2));
end

function m = haversine(loc1, loc2)
    % HAVERSINE     Compute distance between locations using Haversine formula
   
    locs = {loc1 loc2};     % Combine inputs to make checking easier

    % Convert all decimal degrees to radians
    locs = cellfun(@(x) x .* pi./180, locs, 'UniformOutput', 0);
    
    % Begin calculation
    R = 6371;                                   % Earth's radius in km
    delta_lat = locs{2}(1) - locs{1}(1);        % difference in latitude
    delta_lon = locs{2}(2) - locs{1}(2);        % difference in longitude
    a = sin(delta_lat/2)^2 + cos(locs{1}(1)) * cos(locs{2}(1)) * ...
        sin(delta_lon/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    m = 1000 * R * c;
end