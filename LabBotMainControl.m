classdef LabBotMainControl
    %% Properties
    properties 
        % Objects of Parent classes
        movementController
        enviornment
        GUI_Func

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
            obj.GUI_Func = GUI_Functions ();

            % Call SimultaneousControl
            % obj.DemonstrationControl();

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
            obj.guiApp = GUI();  % Start GUI

            % Initially disable the interactive controls (sliders, etc.)
            obj.GUI_Func.disableControls();
            
            %% Environment not working 
            % Initialize the environment
            % disp('Initializing environment...');
            % obj.enviornment.InitialiseEnviorment();  % Logic to initialize the environment

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
            
            %% Robot Initialisation
            % Initaialising Robot Models
            self.rUR3 = UR3;
            self.rLabBot = LabBot_7DOF;

            %% Base transforms
            UR3baseTr = transl(0,0,0);
            LabBotbaseTr = transl(2,2,0);

            self.rUR3.model.base = self.rUR3.model.base.T * UR3baseTr;
            self.rUR3.model.base = self.rUR3.model.base.T * LabBotbaseTr;


            %%

            % Wait for the "On" switch to enable controls
            disp('Waiting for On switch...');

            % Add listener for the switch (on/off control)
            addlistener(obj.guiApp.Switch, 'ValueChanged', @(src, event) obj.toggleControl(src, event));

            % Add listeners to the sliders for real-time joint control
            addlistener(obj.guiApp.Joint1Slider_LabBot, 'ValueChanged', @(src, event) obj.jointSliderMoved('LabBot', 1, src.Value));
            addlistener(obj.guiApp.Joint2Slider_LabBot, 'ValueChanged', @(src, event) obj.jointSliderMoved('LabBot', 2, src.Value));

            % Similarly, add listeners for UR3 joint sliders
            addlistener(obj.guiApp.Joint1Slider_UR3, 'ValueChanged', @(src, event) obj.jointSliderMoved('UR3', 1, src.Value));
            addlistener(obj.guiApp.Joint2Slider_UR3, 'ValueChanged', @(src, event) obj.jointSliderMoved('UR3', 2, src.Value));

            % Add listeners for edit fields (for joint control using values)
            addlistener(obj.guiApp.EditField_LabBotJoint1, 'ValueChanged', @(src, event) obj.jointSliderMoved('LabBot', 1, src.Value));
            addlistener(obj.guiApp.EditField_UR3Joint1, 'ValueChanged', @(src, event) obj.jointSliderMoved('UR3', 1, src.Value));

            % Let the GUI remain interactive while the event loop continues to run
            disp('GUI initialized. You can now control the robot using the sliders.');

            % Wait for the GUI to be closed
            uiwait(obj.guiApp.UIFigure);
        end

        % Toggle the control based on the switch state (on/off)
        function toggleControl(obj, src, event)
            if strcmp(src.Value, 'On')
                obj.GUI_Func.enableControls();  % Enable the control
                disp('Controls enabled.');
            else
                obj.GUI_Func.disableControls();  % Disable the control
                disp('Controls disabled.');
            end
        end

        % Called when a slider is moved (for joint control)
        function jointSliderMoved(obj, robotName, jointIndex, value)
            if obj.GUI_Func.controlEnabled
                if strcmp(robotName, 'LabBot')
                    obj.GUI_Func.JointMovement(obj.GUI_Func.rLabBot, jointIndex, value);
                elseif strcmp(robotName, 'UR3')
                    obj.GUI_Func.JointMovement(obj.GUI_Func.rUR3, jointIndex, value);
                end
            else
                disp('Control is disabled. Please turn the switch on.');
            end
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
            obj.GUI_Func.MixChem(self, 2, chemicals2mix2, 5)
                    
        end
    end
end
 