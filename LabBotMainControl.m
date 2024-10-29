classdef LabBotMainControl
    %% Properties
    properties 
        % Objects of Parent classes
        movementController
        environment
        GUI_Func
        wrkspaceCalc

        % Initialise Robot models variables
        rUR3
        rCustomBot

        % Initialise GUI 
        guiApp
    end 

    %% Constructor method
    methods
        function obj = LabBotMainControl(controlCase)
            % Initialise Parent classes 
            obj.movementController = LabBotMovementControl();
            obj.environment = LabBotEnvironment();
            obj.GUI_Func = GUI_Functions();
            obj.wrkspaceCalc = LabBotCalculations();

                
            % Switch based on the numeric control case provided
            switch controlCase
                case 1 % GUI
                    % Debugging Line
                    fprintf('Received controlCase: %d\n', controlCase);

                    % Call GUI interface function
                    obj.GUI_InterfaceControl();
                        
                case 2 % Demo
                    % Debugging Line
                    fprintf('Received controlCase: %d\n', controlCase);

                    % Call SimultaneousControl function
                    obj.DemonstrationControl();
    
                case 3 % Calc
                    % Debugging Line
                    fprintf('Received controlCase: %d\n', controlCase);

                    % Call workspace calculation function
                    obj.WorkspaceCalculation();
    
                otherwise
                    % Debugging Line
                    fprintf('Received controlCase: %d\n', controlCase);

                    % If an unknown case is provided, display an error message
                    error('Unknown control case specified: %d', controlCase);
            end
        end
    end


    %% Functions
    methods 
        %% Control Through GUI Interface

        function GUI_InterfaceControl(obj)
            disp('GUI Interface Control Comencing')
        
            %% Create GUI
            % Create the GUI
            obj.guiApp = GUI();  % Start GUI
                        
            %% Environment  
            % Initialize the environment
            disp('Initialising environment...');
            obj.environment.InitialiseEnvironment();  
            disp('Enviornment Initialised');

            % Initaialising Robot Models
            % self.rUR3 = UR3(transl(0,0,1.5));
            % self.rCustomBot = CustomBot(transl(-1,0,1.5));

            %% Call Listening function 
            % robot = self.rUR3;
            % robot = obj.environment.rUR3;
            % obj.GUI_Func.GUITeachUR3(robot);

            % robot = obj.environment.rCustomBot;
            % obj.GUI_Func.GUITeachCustomBot(robot);

            %% Call Listening functions in parallel
            % pool = gcp('nocreate');  % Get current parallel pool
            % if isempty(pool)
            %     pool = parpool;  % Start a new parallel pool if none exists
            % end
            % 
            % % Run each teach function in parallel
            % futureUR3 = parfeval(pool, @obj.GUI_Func.GUITeachUR3, 0, robotUR3);
            % futureCustomBot = parfeval(pool, @obj.GUI_Func.GUITeachCustomBot, 0, robotCustomBot);
            % 

            %% Wait for Gui          
            % Wait for the GUI to be closed
            uiwait(obj.guiApp.UIFigure);

            % Cancel parallel tasks if GUI is closed
            % cancel(futureUR3);
            % cancel(futureCustomBot);

  
        end                

        %% Demonstration of Robot Control

        function DemonstrationControl(obj)
            clf;
            disp('Demonstration Control Commencing')
                       
            %% Initialisation of Enviorment   
            % Initialize the environment
            disp('Initialising environment...');
            obj.environment.InitialiseEnvironment();  
            disp('Enviornment Initialised');
                              
            %% Trasforms
            % UR3 End Effector Goal Destinations 
            % Cheecking for correct orientation
            UR3_Pos1 = [0.3,0.2,1.6]; % positive x Q++
            UR3_Pos2 = [-0.3,0.2,1.6]; % negative x Q-+
            UR3_Pos3 = [0.2,0.3,1.6]; % positive y Q++
            UR3_Pos4 = [0.2,-0.3,1.6]; % negative y Q+-
            UR3_Pos5 = [0.3,-0.2,1.6]; % positive x Q+-
            UR3_Pos6 = [-0.3,-0.2,1.6]; % negative x Q--
            UR3_Pos7 = [-0.2,0.3,1.6]; % positive y Q-+
            UR3_Pos8 = [-0.2,-0.3,1.6]; % negative y Q--
              
            % LabBot End Effector Goal Destinations 
            % LabBot_Pos1 = [0.2,0.2,0.2];
            % LabBot_End = [0.2,0.2,0.2];


            %% Perform Movements
            % Calling Move2Global using self
            % For UR3
            obj.movementController.Move2Global(UR3_Pos1, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos2, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos3, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos4, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos5, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos6, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos7, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos8, obj.environment.rUR3);

            
            % For LabBot
            % obj.movementController.Move2Global(LabBot_Pos1, obj.environment.rLabBot);
            % obj.movementController.Move2Global(LabBot_End, obj.environment.rLabBot);             

            %% Mix Chemicals 
            % Not nessecary while testing optimisation 
            % Define chemicals to mix and their test tube locations
            chemicals2mix1 = {{'Bromine', 1}, ...  % Notice the use of curly braces
                              {'Iodine', 2}, ... 
                              {'Nitrogen', 4} ...
                              };

            % Call MixChem with 3 chemicals and mixing location 3
            obj.GUI_Func.MixChem( 3, chemicals2mix1, 3)

            chemicals2mix2 = {{'New Mixture', 3}, ... 
                              {'Nitrogen', 4} ...
                              };

            % Call MixChem with 2 chemicals and mixing location 5
            obj.GUI_Func.MixChem(2, chemicals2mix2, 5)
                    
        end

        %% Workspace Calculation 

        function WorkspaceCalculation(obj)
            disp('Workspace Calculation Comencing')

            %% Initialisation of Enviorment   
            % Initialize the environment
            disp('Initialising environment...');
            obj.environment.InitialiseEnvironment();  
            disp('Enviornment Initialised');

            %% Workspace Calculation 
            %% UR3 Calculation
            % Calculate workspace for UR3
            disp('Calculating workspace for UR3...');
            robot = obj.environment.rUR3;
            obj.wrkspaceCalc.calculateRobotWorkspace(robot, 'UR3');

            %% Break
            input('Press "c" to continue \n',"s")

            %% LabBot Calculation
            % Calculate workspace for LabBot
            disp('Calculating workspace for LabBot...');
            robot = obj.environment.rCustomBot;
            obj.wrkspaceCalc.calculateRobotWorkspace(robot, 'CustomBot');

        end
    end
end
 