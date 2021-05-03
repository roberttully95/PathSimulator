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
            elseif nargin == 2
                override = 1;
            end

            % Read the input.
            this.init(varargs_);
            if override
                this.overrideData();
            end

            % Triangulate
            this.triangulate();
            
            % Initialize the first vehicle.
            this.initVehicle(1);
            
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
            
            % Propogate
            for j = 1:this.nActiveVehicles
                i = this.activeVehicles(j);

                % Get desired heading
                triangle = this.triangles(this.vehicles(i).triangleIndex);
                thDesired = atan2(triangle.dir(2), triangle.dir(1));

                % Propogate vehicle
                this.vehicles(i).propogate(this.dT, thDesired);

                % Update triangle
                if ~triangle.containsPt(this.vehicles(i).pos)
                    this.vehicles(i).triangleIndex = triangle.nextIndex;
                end

            end
            this.t = this.t + this.dT;
            this.pause();
            
            %%%%%%%%%%%%%
            % POSTCHECK %
            %%%%%%%%%%%%%
            this.postPropogationUpdate();
            
            % Plot vehicles in their new state
            this.plotVehicles();  
            
            %{
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
                
                % If it has changed triangles, update heading / velocity.
                if ~triangle.containsPt(this.vehicles(i).pos)
                    
                    % Get the next index
                    next = triangle.nextIndex;
                    
                    % Check if at goal
                    if isnan(next)
                        this.terminateVehicle(i, 0);
                    else
                        this.vehicles(i).triangleIndex = next;
                        dir = this.triangles(next).dir;
                        this.vehicles(i).th = atan2(dir(2), dir(1));
                    end
                end
                
                % Process the case in which the vehicle has collided
                % with the corridor.
                pos = this.vehicles(i).pos;
                dirEdge = this.triangles(this.vehicles(i).triangleIndex).directionEdge;
                [d, ~] = distToLineSegment(dirEdge, pos);
                if d < this.vehicles(i).r
                    this.terminateVehicle(i, 2)
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
                this.wrapUp();
                return;
            end
            
            % Update time
            this.t = this.t + this.dT;
            this.pause();
            %}
        end
        
    end
end

