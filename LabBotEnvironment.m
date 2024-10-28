classdef LabBotEnvironment < handle
   
    properties 
        Conical;
        Beaker;
        Table;
        Cube;
        rUR3
        rCustomBot
    end
    
    methods
        function InitialiseEnvironment(self) 
            clf;
    %% Scene Initialisation
            axis([-5 5 -5 5 0 5]);  % Set the axis limits to fit all objects in the environment
            xlabel('X-axis');  % Label the X-axis
            ylabel('Y-axis');  % Label the Y-axis
            zlabel('Z-axis');  % Label the Z-axis
            grid on;  % Display a grid for better visualization of object positions
            hold on;  % Keep the plot active for additional elements
            camlight;
   %% Object Initialiastion         
            
        self.Table = Table;
        self.Beaker = Beaker(1,0.5,1.5);
        self.Conical = Conical(-1,0.5,1.5);
        % self.Cube = Cube;

    % Robot Initialisation
            % Initaialising Robot Models
            self.rUR3 = UR3(transl(0,0,1.5));
            self.rCustomBot = CustomBot(transl(-1,0,1.5));

    %% Scene Initialisation
            axis([-5 5 -5 5 0 5]);  % Set the axis limits to fit all objects in the environment
            xlabel('X-axis');  % Label the X-axis
            ylabel('Y-axis');  % Label the Y-axis
            zlabel('Z-axis');  % Label the Z-axis
            grid on;  % Display a grid for better visualization of object positions
            hold on;  % Keep the plot active for additional elements
            camlight;
    %% Base transforms
            % UR3baseTr = transl(0,0,2);
            % LabBotbaseTr = transl(1,0,2);
            % 
            % self.rUR3.model.base = self.rUR3.model.base.T * UR3baseTr;
            % self.rUR3.model.base = self.rUR3.model.base.T * LabBotbaseTr;

    
        end
    end
end

