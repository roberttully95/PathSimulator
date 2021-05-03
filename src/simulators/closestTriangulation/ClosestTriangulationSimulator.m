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

            % Define the triangulation
            this.triangulate();
            
            % Initialize the first vehicle.
            for i = 1:this.nVehicles
                this.initVehicle(i);
            end
            
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
        end
    end
    

end

