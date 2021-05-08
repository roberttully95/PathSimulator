function regions = closestTriangulation(p1, p2)
    %CLOSESTTRIANGULATION Generates a triangulation by using the closest
    %available unvisited vertex at each time step.
    
    % Get params
    m = p1.length;
    n = p2.length;
    
    % Init triangles
    regions = Region.empty(max(m, n) + 1, 0);
    
    % Initialize
    i = 1;
    j = 1;
    index = 1;
    while (i < m || j < n)
        
        % Get the current vertices
        v1a = p1.coords(min(i, m), :);
        v2a = p2.coords(min(j, n), :);

        % Get the next vertices
        v1b = p1.coords(min(i + 1, m), :);
        v2b = p2.coords(min(j + 1, n), :);

        % Get distances
        d1 = norm(v1a - v2b);
        d2 = norm(v1b - v2a);

        % If one path is finished, we need to do the other.
        if i == m
            regions(index) = Region([v2a; v1a; v2b], 'r');
            regions(index).thetaCmd = vecHeading(vecNorm(v2b - v2a));
            regions(index).prevIndex = index - 1;
            j = j + 1;
            index = index + 1;
            continue;
        elseif j == n
            regions(index) = Region([v1a; v2a; v1b], 'r');
            regions(index).thetaCmd = vecHeading(vecNorm(v1b - v1a));
            regions(index).prevIndex = index - 1;
            i = i + 1;
            index = index + 1;
            continue;
        end

        % Create edge
        if d1 <= d2
            regions(index) = Region([v2a; v1a; v2b], 'r');
            regions(index).thetaCmd = vecHeading(vecNorm(v2b - v2a));
            regions(index).prevIndex = index - 1;
            regions(index).nextIndex = index + 1;
            j = j + 1;
            index = index + 1;
            continue;
        else
            regions(index) = Region([v1a; v2a; v1b], 'r');
            regions(index).thetaCmd = vecHeading(vecNorm(v1b - v1a));
            regions(index).prevIndex = index - 1;
            regions(index).nextIndex = index + 1;
            i = i + 1;
            index = index + 1;
            continue;
        end

    end

end

