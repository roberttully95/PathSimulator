classdef SimulatorExample < Simulator
    %SIMULATOREXAMPLE This is an example of a simulator class that derives the base class. Much
    %less code needs to be written here which makes the iteration process faster (and easier).
        
    methods
        function this = SimulatorExample(varargin)
            %SIMULATOREXAMPLE Instantiate the class.

            % Read the input.
            this.init(varargin);
            
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
            
            % Store x and y positions for all vehicles
            x = NaN(this.nVehicles, 1);
            y = NaN(this.nVehicles, 1);
            
            % Iterate through vehicles.
            for i = 1:this.nVehicles
                                       
                % If finished skip
                if this.vehicles(i).finished
                    continue;
                end
                
                % Don't consider ended vehicles
                if this.t < this.vehicles(i).tInit
                    continue;
                end
                
                % Propogate vehicle
                this.vehicles(i).propogate(this.dT);
                x(i) = this.vehicles(i).x;
                y(i) = this.vehicles(i).y;
                
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
            this.LOG();
            
            % Single call to 'scatter' to plot all points
            if this.hasAxis
                hold(this.axis, 'on');
                delete(this.handle)
                this.handle = scatter(this.axis, x, y, 'r', '*'); 
            end
            
            % If all the vehicles are finished, set flag.
            if this.finished
                this.DUMP();
            end
            
            % Update distance matrix
            this.updateDistances();
                        
            % Update time
            this.t = this.t + this.dT;
        end

    end
    

end

