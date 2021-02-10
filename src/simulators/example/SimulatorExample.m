classdef SimulatorExample < Simulator
    %SIMULATOREXAMPLE This is an example of a simulator class that derives the base class. Much
    %less code needs to be written here which makes the iteration process faster (and easier).
    
    methods
        function this = SimulatorExample(file)
            %SIMULATOREXAMPLE Instantiate the class.

            % Read the input file.
            this.init(file);
            
            % triangulate
            this.triangles = this.triangulate();
            
            % Plot triangles
            this.plotTriangles(gca);
        end

        function triangles = triangulate(this)
            %TRIANGULATE Specify triangulation function for this simulation.
            triangles = closestTriangulation(this.path1, this.path2);
        end
        
    end
end

