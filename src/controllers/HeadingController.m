classdef HeadingController < handle
    %HEADINGCONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dT          % Specifies the time step of the controller.
        Kp          % Proportional gain coefficient
        Ki          % Integral gain coefficient
        Kd          % Derivitive gain coefficient
        
        thDotMax    % The maximum heading turn rate
        
        integral    % Stores the value of the integral term
        prevError   % Stores the value of the previous error value
        prevThDelta % Stores the previous heading change
    end
    
    methods
        function this = HeadingController(dT, Kp, Ki, Kd, thDotMax)
            %HEADINGCONTROLLER Initialize the heading controller object.
            
            % Initialize values
            this.dT = dT;
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            
            % Define the maximum turn rate for the vehicle
            this.thDotMax = thDotMax;
            
            % Initialize PID parameters
            this.integral = 0;
            this.prevError = 0;
            this.prevThDelta = 0;
        end
        
        function thDelta = calculate(this, setpoint, current)
            
            % Get error
            error = this.getError(setpoint, current);
            
            % Get proportional term
            P = this.Kp * this.dT * abs(error);
            
            % Get integral term
            this.integral = this.integral + this.dT * abs(error);
            I = this.Ki * this.integral;
            
            % Get derivative term
            derivative = (error - this.prevError) / this.dT;
            D = this.Kd * derivative;
            
            % Get combined value
            thDelta = sign(error) * min(P + I + D, this.dT * this.thDotMax);
            
            % Update the previous values
            this.prevError = error;
            this.prevThDelta = thDelta;
        end
        
    end
    
    methods (Static, Access = private)

        function error = getError(setpoint, current)
            % GETERROR2 Determines the heading error
            
            % Get the difference between the angles
            error = setpoint - current;
            if error > pi
                error = error - 2*pi; 
            elseif error < -pi
                error = error + 2*pi;
            end

        end
        
    end

end

