classdef ClosestTriangulationSimulator < Simulator
    %CLOSESTTRIANGULATIONSIMULATOR This is an example of a simulator class that derives the base class. Much
    %less code needs to be written here which makes the iteration process faster (and easier).
        
    methods
        function this = ClosestTriangulationSimulator(varargin)
            %CLOSESTTRIANGULATIONSIMULATOR Instantiate the class.
            
            % Define map type
            this.type = "Paths";
            
            % Parse input data
            if nargin == 1
                varargs_ = varargin(1);
            elseif nargin == 2
            	varargs_ = varargin(1:2);
            else
                error("Invalid number of arguments provided!")
            end
            
            % Read the input.
            this.init(varargs_);

            % triangulate
            this.triangulate();
            
            % Plot triangles
            this.plotTriangles();
        end
        
        function triangulate(this)
            %TRIANGULATE Specify triangulation function for this simulation.
            
            % Get trianges
            this.triangles = closestTriangulation(this.path1, this.path2);
        
            % Set the vehicle thetas
            dir = this.triangles(1).dir;
            for i = 1:this.nVehicles
                this.vehicles(i).th = atan2(dir(2), dir(1));
            end 
        end
        
        function propogate(this)
            % PROPOGATE Propogates the simulation by 'dT' seconds.
            
            if ~this.finished
                
                % TODO: Activate vehicles at the current time step
                % TODO: Get list of valid vehicles at this point
                % TODO: Terminate vehicles in a collision state
                % TODO: Iterate through the remaining vehicles
                
                % Iterate through all vehicles.
                for i = 1:this.nVehicles

                    % All checks must be done before the propogation step. 
                    
                    % CHECK 1: Vehicle is finished
                    if this.vehicles(i).finished
                        continue;
                    end

                    % CHECK 2: Vehicle has not yet started
                    if this.t < this.vehicles(i).tInit
                        continue;
                    end

                    % CHECK 3: Vehicle has just been initialized
                    if this.t - this.dT < this.vehicles(i).tInit && this.t >= this.vehicles(i).tInit
                        this.vehicles(i).active = true;
                        continue;
                    end

                    % CHECK 4: Vehicle is inside map
                    if isnan(this.vehicles(i).triangleIndex)
                        this.terminateVehicle(i, 0);
                        continue;
                    end
                    
                    % CHECK 4: Vehicle - corridor collision check.
                    pos = this.vehicles(i).pos;
                    dirEdge = this.triangles(this.vehicles(i).triangleIndex).directionEdge;
                    [d, ~] = distToLineSegment(dirEdge, pos);
                    if d < this.vehicles(i).r
                        this.terminateVehicle(i, 2);
                        continue;
                    end
                    
                    % CHECK 5: Vehicle - vehicle collision check.
                    collision = find(this.distances(i, :) < this.collDists(i, :) == 1);
                    if size(collision, 2) > 0
                        this.terminateVehicle(i, 2, collision(1));
                        for j = 1:size(collision, 2)
                            this.terminateVehicle(collision(j), 2, i);
                        end
                    end

                    % Get the desired heading
                    triangle = this.triangles(this.vehicles(i).triangleIndex);
                    thDesired = atan2(triangle.dir(2), triangle.dir(1));
                    
                    % Propogate vehicle
                    this.vehicles(i).propogate(this.dT, thDesired);
                    
                    % If it has changed triangles, update triangle.
                    if ~triangle.containsPt(this.vehicles(i).pos)
                        this.vehicles(i).triangleIndex = triangle.nextIndex;
                    end

                end

                % Update vehicle-to-vehicle distance matrix. This function
                % detects collisions that occur between vehicles and
                % updates the distance matrix for non-collision vehicles.
                this.updateDistances();
                
                % log new data
                this.TIMELOG();

                % Single call to 'scatter' to plot all points
                this.plotVehicles();

                % If all the vehicles are finished, set flag.
                if this.finished
                    this.wrapUp();
                    return;
                end

                % Update time
                this.t = this.t + this.dT;
                this.pause();
            end
        end
    end
    

end

