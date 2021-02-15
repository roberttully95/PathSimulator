classdef ManualTriangleSimulator < Simulator
    %MANUALTRIANGLESIMULATOR Summary of this class goes here
    %   Detailed explanation goes here

    methods
        function this = ManualTriangleSimulator(varargin)
            %MANUALTRIANGLESIMULATOR Instantiate the class.

            % Define map type
            this.type = "Triangles";

            % Parse input data
            if nargin == 1
                varargs_ = varargin(1);
                override = 0;
            elseif nargin >= 2 
                varargs_ = varargin(1:2);
                override = 0;
            elseif nargin == 3
                override = 1;
            end

            % Read the input.
            this.init(varargs_);
            if override
                this.overrideData();
            end

            % Triangulate
            this.triangulate();
            
            % Plot triangles
            this.plotTriangles();
        end
        
        function triangulate(this)
            %TRIANGULATE Specify triangulation function for this simulation.
            
            % Get trianges
            this.triangles = this.simData.triangles;
        end
         
        function propogate(this)
            % PROPOGATE Propogates the simulation by 'dT' seconds.
            
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
                
                % Propogate vehicle
                this.vehicles(i).propogate(this.dT);

                % If it has changed triangles, update heading / velocity.
                if ~this.triangles(this.vehicles(i).triangleIndex).containsPt(this.vehicles(i).pos)
                    
                    % Get the next index
                    next = this.triangles(this.vehicles(i).triangleIndex).nextIndex;
                    
                    % Check if at goal
                    if isnan(next)
                        this.terminateVehicle(i);
                    else
                        this.vehicles(i).triangleIndex = next;
                        dir = this.triangles(next).dir;
                        this.vehicles(i).th = atan2(dir(2), dir(1));
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
            this.pause(this.dT);
        end
        
    end
end

