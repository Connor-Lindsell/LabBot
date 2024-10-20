classdef LabBotEnviornment
    %% Constructor method
    methods 
        function obj = LabBotEnviornment
            obj.InitialiseEnviorment();
        end
    end

    %% Functions
    methods (Static)
        function InitialiseEnviorment() 
            %% Base transforms

            %% Set Up Enviorment 

            % Call Enviorment class here

            % Temporary Enviorment 
            % Configure the axes and labels for the environment
            axis([-1 1 -1 1 0 1]);  % Set the axis limits to fit all objects in the environment
            xlabel('X-axis');  % Label the X-axis
            ylabel('Y-axis');  % Label the Y-axis
            zlabel('Z-axis');  % Label the Z-axis
            grid on;  % Display a grid for better visualization of object positions
            hold on;  % Keep the plot active for additional elements
        end
    end


end