classdef LabBotEnvironment < handle
   
    properties 
        ConicalA;
        ConicalB;
        ConicalC;
        BeakerA;
        BeakerB;
        BeakerC;
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

            %% Ground Texture
            surf([-5,-5;5,5] ...
            ,[-5,5;-5,5] ...
            ,[0.01,0.01;0.01,0.01] ...
            ,'CData',imread('concrete.jpg') ...
            ,'FaceColor','texturemap');
            
            %% Static Object Initialisation
            PlaceObject('fireExtinguisher.ply', [2,1.5,0]);
            PlaceObject('fireExtinguisher.ply', [2,1,0]);
            PlaceObject('emergencyStopWallMounted.ply', [-0.5,1.2,0.5])
            PlaceObject('TPose.ply', [0.2,1.9,0]); %% https://creazilla.com/media/3d-model/1795171/scientist
            PlaceObject('BMan.ply', [0,1.5,0])
            % PlaceTransparentObject('TPose.ply', [0,0,1]); % Ghost

            PlaceObject('Enclosure.ply', [0,0,0])
            PlaceObject('EyeWash.ply', [0,0,0]) %% https://sketchfab.com/3d-models/emergency-shower-with-eye-wash-61a536cf4e1f4995a8128863f8994e3e
            PlaceObject('LoudSpeaker.ply', [0,0,0]) %% https://free3d.com/3d-model/speaker-99850.html
            PlaceObject('SignalTower.ply', [0,0,0]) %% https://grabcad.com/library/signal-tower-1
            PlaceTransparentObject('Window.ply',[0,0,0]);
            PlaceTransparentObject('Door.ply',[0,0,0]); % Door Closed
            % PlaceTransparentObject('Door.ply',[1.8,0,0]); % Door Opened
           

            %% Object Initialiastion         
            self.Table = Table();
            self.BeakerA = Beaker(transl(1,0.5,tableHeight));
            self.BeakerB = Beaker(transl(1.2,0.5,tableHeight));
            self.BeakerC = Beaker(transl(1.4,0.5,tableHeight));
            self.ConicalA = Conical(transl(-1,0.5,tableHeight));
            self.ConicalB = Conical(transl(-1.2,0.5,tableHeight));
            self.ConicalC = Conical(transl(-1.4,0.5,tableHeight));
            % self.Cube = Cube;

            % Define ellipsoid properties for each object
            self.objects = {...
                            struct('Center', [1, 0.5, tableHeight], 'Radii', [0.1, 0.1, 0.2]), ...     % BeakerA
                            struct('Center', [1.2, 0.5, tableHeight], 'Radii', [0.1, 0.1, 0.2]), ...   % BeakerB
                            struct('Center', [1.4, 0.5, tableHeight], 'Radii', [0.1, 0.1, 0.2]), ...   % BeakerC
                            struct('Center', [-1, 0.5, tableHeight], 'Radii', [0.15, 0.15, 0.25]), ... % ConicalA
                            struct('Center', [-1.2, 0.5, tableHeight], 'Radii', [0.15, 0.15, 0.25]),...% ConicalB
                            struct('Center', [-1.4, 0.5, tableHeight], 'Radii', [0.15, 0.15, 0.25])};  % ConicalC

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

