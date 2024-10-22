classdef LabBotMainControl
    %% Properties
    properties 
        % Objects of Parent classes
        movementController
        enviornment

        % Initialise Robot models variables
        rUR3
        rLabBot

        % Initialise GUI 
        guiApp
    end 

    %% Constructor method
    methods
        function obj = LabBotMainControl
            % Initialise Parent classes 
            obj.movementController = LabBotMovementControl();
            obj.enviornment = LabBotEnviornment();

            % Call SimultaneousControl
            obj.DemonstrationControl();

            % Call GUI interface
            obj.GUI_InterfaceControl();

        end
    end 

    %% Functions
    methods 
        %% Control Through GUI Interface
        function GUI_InterfaceControl(obj)
            % Start GUI and environment, but wait for the On switch to enable controls
        
            % Create the GUI
            obj.guiApp = GUI();
            
            % Initially disable interactive controls (sliders, etc.)
            obj.disableControls();
            
            % Initialize the environment
            disp('Initializing environment...');
            obj.enviornment.initializeEnvironment();  % Add logic to initialize the environment
            
            % Wait for the "On" switch to enable controls
            disp('Waiting for On switch...');
        end

        % Function to disable all interactive controls in the GUI
        function disableControls(obj)
            % Disable all sliders and buttons
            obj.guiApp.Joint1Slider_LabBot.Enable = 'off';
            obj.guiApp.Joint2Slider_LabBot.Enable = 'off';
            obj.guiApp.Joint3Slider_LabBot.Enable = 'off';
            obj.guiApp.Joint4Slider_LabBot.Enable = 'off';
            obj.guiApp.Joint5Slider_LabBot.Enable = 'off';
            obj.guiApp.Joint6Slider_LabBot.Enable = 'off';
            obj.guiApp.Joint7Slider_LabBot.Enable = 'off';

            % Similarly disable sliders for UR3 if applicable
        end

        % Function to enable all interactive controls in the GUI
        function enableControls(obj)
            % Enable all sliders and buttons
            obj.guiApp.Joint1Slider_LabBot.Enable = 'on';
            obj.guiApp.Joint2Slider_LabBot.Enable = 'on';
            obj.guiApp.Joint3Slider_LabBot.Enable = 'on';
            obj.guiApp.Joint4Slider_LabBot.Enable = 'on';
            obj.guiApp.Joint5Slider_LabBot.Enable = 'on';
            obj.guiApp.Joint6Slider_LabBot.Enable = 'on';
            obj.guiApp.Joint7Slider_LabBot.Enable = 'on';

            % Similarly enable sliders for UR3 if applicable
        end

        % This function will handle the movement of joints, which is called by the sliders
        function JointMovement(obj, robot, joint, angle)
            fprintf('Moving Joint %d of robot to %.2f degrees\n', joint, angle);
            % Logic to move the robot joint goes here
        end
        

        %% Demonstration of Robot Control
        function DemonstrationControl(obj)
            clf;
            %% Robot Initialisation
            % Initaialising Robot Models
            self.rUR3 = UR3;
            self.rLabBot = LabBot_7DOF;
           
            %% Initialisation of Enviorment 
            obj.enviornment.InitialiseEnviorment();
                              
            %% Trasforms
            % UR3 End Effector Goal Destinations 
            % Cheecking for correct orientation
            UR3_Pos1 = [0.3,0.2,0.2]; % positive x Q++
            UR3_Pos2 = [-0.3,0.2,0.2]; % negative x Q-+
            UR3_Pos3 = [0.2,0.3,0.2]; % positive y Q++
            UR3_Pos4 = [0.2,-0.3,0.2]; % negative y Q+-
            UR3_Pos5 = [0.3,-0.2,0.2]; % positive x Q+-
            UR3_Pos6 = [-0.3,-0.2,0.2]; % negative x Q--
            UR3_Pos7 = [-0.2,0.3,0.2]; % positive y Q-+
            UR3_Pos8 = [-0.2,-0.3,0.2]; % negative y Q--
              
            % LabBot End Effector Goal Destinations 
            LabBot_Pos1 = [0.2,0.2,0.2];
            LabBot_End = [0.2,0.2,0.2];


            %% Perform Movements
            % Calling Move2Global using self
            % For UR3
            obj.movementController.Move2Global(UR3_Pos1, self.rUR3);
            obj.movementController.Move2Global(UR3_Pos2, self.rUR3);
            obj.movementController.Move2Global(UR3_Pos3, self.rUR3);
            obj.movementController.Move2Global(UR3_Pos4, self.rUR3);
            obj.movementController.Move2Global(UR3_Pos5, self.rUR3);
            obj.movementController.Move2Global(UR3_Pos6, self.rUR3);
            obj.movementController.Move2Global(UR3_Pos7, self.rUR3);
            obj.movementController.Move2Global(UR3_Pos8, self.rUR3);

            
            % For LabBot
            obj.movementController.Move2Global(LabBot_Pos1, self.rLabBot);
            obj.movementController.Move2Global(LabBot_End, self.rLabBot);             

            %% Mix Chemicals 
            % Not nessecary while testing optimisation 
            % Define chemicals to mix and their test tube locations
            chemicals2mix1 = {{'Bromine', 1}, ...  % Notice the use of curly braces
                              {'Iodine', 2}, ... 
                              {'Nitrogen', 4} ...
                              };

            % Call MixChem with 3 chemicals and mixing location 3
            MixChem(self, 3, chemicals2mix1, 3)

            chemicals2mix2 = {{'New Mixture', 3}, ... 
                              {'Nitrogen', 4} ...
                              };

            % Call MixChem with 2 chemicals and mixing location 5
            MixChem(self, 2, chemicals2mix2, 5)
                    
        end
    end
end
