classdef Vehicle < handle
    %VEHICLE Vehicle class.
    
    properties
        x               % x coordinate of the vehicle
        y               % y coordinate of the vehicle
        th              % heading of the vehicle (-pi -> pi)
        v               % Current velocity of the vehicle.
        tInit           % The time at which the vehicle enters the 'map'
        tEnd            % The time at which the vehicle exits the 'map'
        r               % The radius of the vehicle
        wMax            % The max turn rate of the vehicle
        finished        % Flag that determines if the vehicle has exited the 'map'.
        triangleIndex   % The index of the triangle within the triangles array that the vehicle is currently in.
        active          % Flag that determines if the vehicle is actively in the map.
        distTravelled   % Stores the distance travelled by the vehicle
    end
    
    properties (Dependent)
        pos         % The current position of the vehicle. [x, y]
        pose        % The current pose of the vehicle. [x, y, th]
        vx          % The x component of the velocity vector.
        vy          % The y component of the velocity vector.
        lifeSpan    % The length of time the vehicle was present in the map.
    end
    
    methods
        
        function this = Vehicle(x0, y0, th0, v0, t0, r, wMax)
            % VEHICLE2D Constructor taking initial conditions
            this.setInitialConditions(x0, y0, th0, v0, t0);
            this.tEnd = Inf;
            this.triangleIndex = 1;
            this.active = (t0 == 0);
            this.r = r;
            this.wMax = wMax;
            this.distTravelled = 0;
        end
        
        function setInitialConditions(this, x0, y0, th0, v0, t0)
            % SETINITIALCONDITIONS Sets the initial conditions of the
            % vehicle. Can be invoked after the creation of the vehicle
            % object.
            this.x = x0;
            this.y = y0;
            this.th = th0;
            this.v = v0;
            this.tInit = t0;
            this.finished = false;
        end
        
        function propogate(this, dT, thDesired)
            % PROPOGATE Propogates the vehicle forward based on the current
            % position and velocity.
            
            % Store previous position
            xStore = this.x;
            yStore = this.y;
            
            % Update heading
            this.th = thDesired;
            
            % Update position and 
            this.x = this.x + this.v * cos(this.th) * dT;
            this.y = this.y + this.v * sin(this.th) * dT;
            
            % Update distance travelled
            this.distTravelled = this.distTravelled + norm([xStore, yStore] - [this.x, this.y]);
        end
        
        function val = get.pos(this)
            % GETPOS Returns the 2D position of the vehicle.
            val = [this.x, this.y];
        end
        
        function val = get.pose(this)
            % GETPOS Returns the 2D pose of the vehicle.
            val = [this.x, this.y, this.th];
        end
        
        function val = get.vx(this)
            % GETVX Returns the x component of the velocity vector.
            val = this.v * cos(this.th);
        end
        
        function val = get.vy(this)
            % GETVY Returns the y component of the velocity vector.
            val = this.v * sin(this.th);
        end
        
        function val = get.lifeSpan(this)
            % GETLIFESPAN Returns the length of time for which the vehicle has been active.
            val = this.tEnd - this.tInit;
        end
        
        function plot(this, ax, color)
            % PLOT Plots the 2D location of the vehicle.
            arrows(ax, this.x, this.y, 1, 90 - this.th * 180 / pi);
            scatter(ax, this.x, this.y, color, '*');
        end
        
        function terminate(this, t)
            % TERMINATE Terminates the vehicle.
            this.active = false;
            this.finished = true;
            this.tEnd = t;
        end
    end
end

