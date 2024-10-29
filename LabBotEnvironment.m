classdef LabBotEnvironment < handle
   
    properties 
        Conical;
        BeakerA;
        BeakerB;
        Table;
        Cube;
        rUR3
        rCustomBot
        movementController
        objects


    end
    
    methods
        function InitialiseEnvironment(self) 
            clf;

            tableHeight = 0.8; 


            %% Scene Initialisation
            axis([-3 3 -3 3 0 3]);  % Set the axis limits to fit all objects in the environment
            xlabel('X-axis');  % Label the X-axis
            ylabel('Y-axis');  % Label the Y-axis
            zlabel('Z-axis');  % Label the Z-axis
            grid on;  % Display a grid for better visualization of object positions
            hold on;  % Keep the plot active for additional elements
            camlight;
            
            %% Static Object Initialisation
            PlaceObject('fireExtinguisher.ply', [2,2,0]);
            PlaceObject('emergencyStopWallMounted.ply', [1,2,0])
            PlaceObject('TPose.ply', [0,1.9,0]);
            PlaceObject('BMan.ply', [0,1.5,0])

            %% Object Initialiastion         
            self.Table = Table();
            self.BeakerA = Beaker(transl(1,0.5,tableHeight));
            self.BeakerB = Beaker(transl(1.3,0.2,tableHeight));
            self.Conical = Conical(-1,0.5,tableHeight);
            % self.Cube = Cube;

            % Define ellipsoid properties for each object
            self.objects = {struct('Center', [1, 0.5, tableHeight], 'Radii', [0.1, 0.1, 0.2]), ...  % BeakerA
                            struct('Center', [1.3, 0.2, tableHeight], 'Radii', [0.1, 0.1, 0.2]), ... % BeakerB
                            struct('Center', [1.5, -0.5, tableHeight], 'Radii', [0.15, 0.15, 0.25]), ... % Conical
                            struct('Center', [0, 0, tableHeight + 0.4], 'Radii', [1, 1, 0.1])}; % Table
        

            %% Robot Initialisation
            % Initaialising Robot Models
            self.rUR3 = UR3(transl(1,0,tableHeight),true,'Gripper');
            self.rCustomBot = CustomBot(transl(-0.5,0,tableHeight), true, 'CGripper');

            % Pass Table to Movement Controller
            % existingTable = self.Table;
            % self.movementController = LabBotMovementControl(existingTable);  % Use existing table

            
            %% Base transforms
            % UR3baseTr = transl(0,0,2);
            % LabBotbaseTr = transl(1,0,2);
            % 
            % self.rUR3.model.base = self.rUR3.model.base.T * UR3baseTr;
            % self.rUR3.model.base = self.rUR3.model.base.T * LabBotbaseTr;

    
        end
    end
end

