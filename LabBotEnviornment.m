classdef LabBotEnviornment
    %% Properties
    properties 
        % Initialise Robot models variables
        rUR3
        rLabBot
    end 

    %% Constructor method
    % methods 
    %     function obj = LabBotEnviornment
    %         obj.InitialiseEnviorment();
    %     end
    % end

    %% Functions
    methods 
        function InitialiseEnviorment(self) 
            %% Robot Initialisation
            % Initaialising Robot Models
            self.rUR3 = UR3;
            self.rLabBot = LabBot_7DOF;

            %% Base transforms
            UR3baseTr = transl(0,0,0);
            LabBotbaseTr = transl(2,2,0);

            self.rUR3.model.base = self.rUR3.model.base.T * UR3baseTr;
            self.rUR3.model.base = self.rUR3.model.base.T * LabBotbaseTr;

            %% Set Up Enviorment 

            % Call Enviorment class here

            % Temporary Enviorment 
            % Configure the axes and labels for the environment
            axis([-2 2 -2 2 0 2]);  % Set the axis limits to fit all objects in the environment
            xlabel('X-axis');  % Label the X-axis
            ylabel('Y-axis');  % Label the Y-axis
            zlabel('Z-axis');  % Label the Z-axis
            grid on;  % Display a grid for better visualization of object positions
            hold on;  % Keep the plot active for additional elements
        end
    end


end