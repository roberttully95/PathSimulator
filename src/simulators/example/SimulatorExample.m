classdef SimulatorExample < Simulator
    %SIMULATOREXAMPLE This is an example of a simulator class that derives the base class. Much
    %less code needs to be written here which makes the iteration process faster (and easier).
        
    methods
        function this = SimulatorExample(varargin)
            %SIMULATOREXAMPLE Instantiate the class.
            
            % Define map type
            this.type = "Paths";
            
            % Parse input data
            if nargin == 1
                varargs_ = varargin(1);
                override = 0;
            elseif nargin >= 2
            	varargs_ = varargin(1:2);
                override = 0;
            elseif nargin == 2
                override = 1;
            end
                        
            % Read the input.
            this.init(varargs_);
            if override
                this.overrideData();
            end
            
            % triangulate
            this.triangulate();
            
            % Plot triangles
            this.plotTriangles();
        end
        
        function overrideData(this)
             % WRITE STUFF HERE
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
                % Iterate through vehicles.
                for i = 1:this.nVehicles

                    % If finished has finished crossing the map
                    if this.vehicles(i).finished
                        continue;
                    end

                    % If the vehicle has not yet started crossing the map.
                    if this.t < this.vehicles(i).tInit
                        continue;
                    end

                    % If vehicle has just been initialized within the map in the last time step.
                    if this.t - this.dT < this.vehicles(i).tInit && this.t >= this.vehicles(i).tInit
                        this.vehicles(i).active = true;
                    end

                    % Get the desired heading
                    triangle = this.triangles(this.vehicles(i).triangleIndex);
                    thDesired = atan2(triangle.dir(2), triangle.dir(1));
                    
                    % Propogate vehicle
                    this.vehicles(i).propogate(this.dT, thDesired);

                    % If it has changed triangles, update triangle.
                    if ~triangle.containsPt(this.vehicles(i).pos)

                        % Get the next index
                        next = triangle.nextIndex;

                        % Check if at goal
                        if isnan(next)
                            this.terminateVehicle(i);
                        else
                            this.vehicles(i).triangleIndex = next;
                        end
                    end
                end

                % Update distance matrix
                this.updateDistances();

                % log new data
                this.TIMELOG();

                % Single call to 'scatter' to plot all points
                this.plotVehicles();

                % If all the vehicles are finished, set flag.
                if this.finished
                    this.DUMP();
                    return;
                end

                % Update time
                this.t = this.t + this.dT;
                this.pause();
            end
        end
    end
    

end

