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
            obj.wrkspaceCalc = LabBotCalculations();
            obj.environment = LabBotEnvironment();
            obj.GUI_Func = GUI_Functions(); 
            obj.movementController = LabBotMovementControl();

            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % Tried to have the enviorment initalise in the constructor as
            % well as the robot models, but for some reason i do this and
            % get pos stops working for the robot models in demo control
            % also trying to get the code to modularly work through classes
            % Didnt work howvere having too much trouble with robot
            % initialisation passing through gui to other classes
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % disp('Initialising environment...');
            % obj.environment.InitialiseEnvironment(); 
            % hold on;
            % disp('Enviornment Initialised');

            % Initialize robots here directly
            % obj.rUR3 = UR3(transl(0,0,1.5));
            % obj.rCustomBot = CustomBot(transl(-1,0,1.5));
            
            % obj.GUI_Func.rUR3 = obj.rUR3;      
            % obj.GUI_Func.rCustomBot = obj.rCustomBot;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
                
            % Switch based on the numeric control case provided
            switch controlCase
                case 1 % GUI
                    % Call GUI interface function
                    obj.GUI_InterfaceControl();
                        
                case 2 % Demo
                    % Call SimultaneousControl function
                    obj.DemonstrationControl();
    
                case 3 % Calc
                    % Call workspace calculation function
                    obj.WorkspaceCalculation();
                    
                case 4 % teach
                    % Call workspace calculation function
                    obj.GUITeach_UR3(robot);

                case 5 % teach
                    % Call workspace calculation function
                    obj.GUITeach_CustomBot();
    
    
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
            obj.guiApp = GUI();  

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Attempting to send the objects to the gui app but then was
            % getting errors saying to many inputs, couldnt find a solution
            % so moved functions to this class from gui functions
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Set robot properties in GUI after creation
            % obj.GUI_Func.rUR3 = obj.rUR3;
            % obj.GUI_Func.rCustomBot = obj.rCustomBot;

            % Optionally, call startupFcn if necessary to initialize further
            % obj.guiApp.startupFcn();
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %% Environment  
            % Initialize the environment
            disp('Initialising environment...');
            obj.environment.InitialiseEnvironment();  
            disp('Enviornment Initialised');
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Errors: have to comment out which one you dont want out 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Call Listening function 
            % robot = obj.environment.rUR3;
            % obj.GUI_Func.GUITeachUR3(robot);

            % robot = obj.environment.rCustomBot;
            % obj.GUI_Func.GUITeachCustomBot(robot);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Parallel listenesr so bothj teachs could wokr at the same
            % time - Not Woring took too long
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %% Wait for Gui          
            % Wait for the GUI to be closed
            uiwait(obj.guiApp.UIFigure);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % relates back to parellel pool 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Cancel parallel tasks if GUI is closed
            % cancel(futureUR3);
            % cancel(futureCustomBot);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
        end                

        %% Demonstration of Robot Control

        function DemonstrationControl(obj)
            clf;
            disp('Demonstration Control Commencing')
            
            % obj.rCustomBot = CustomBot(transl(-1,0,1.5));
            % obj.rUR3 = UR3(transl(0,0,1.5));
            % 
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
            % obj.movementController.Move2Global(LabBot_Pos1, obj.environment.rCustomBot);
            % obj.movementController.Move2Global(LabBot_End, obj.environment.rCustomBot);             

            %% Mix Chemicals 
            % Not nessecary while testing optimisation 
            % Define chemicals to mix and their test tube locations
            chemicals2mix1 = {{'Bromine', 1}, ...  % Notice the use of curly braces
                              {'Iodine', 2}, ... 
                              {'Nitrogen', 4} ...
                              };

            % Call MixChem with 3 chemicals and mixing location 3
            obj.MixChem( 3, chemicals2mix1, 3)

            chemicals2mix2 = {{'New Mixture', 3}, ... 
                              {'Nitrogen', 4} ...
                              };

            % Call MixChem with 2 chemicals and mixing location 5
            obj.MixChem(2, chemicals2mix2, 5)
                    
        end

        %% Workspace Calculation 

        function WorkspaceCalculation(obj)
            disp('Workspace Calculation Comencing')

            % %% Initialisation of Enviorment   
            % % Initialize the environment
            % disp('Initialising environment...');
            % obj.environment.InitialiseEnvironment();  
            % disp('Enviornment Initialised');

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

        %% Mix Chemical Function 
        % Selects Chemicals and then mixes the chemicals by pouring them
        % together
        % 
        % Inputs - 
        % numOfChem: The number of chemicals to be mixed 
        % chem2mix: The chemicals to be mixed and the corresponding test 
        %           tube which it is stored, which are stored in an array 
        %           of pairs eg. {'Bromine', 1}
        % mixingLocation: Where the chemicals are to be mixed
        
        function MixChem(obj, numOfChem, chem2mix, mixingLocation)
            % Define test tube locations (you could adjust this as per your environment)
            testTubeLocation = {[0.2, -0.2, 1.7], ...
                                [0.2, -0.1, 1.7], ...
                                [0.2, 0, 1.7], ...
                                [0.2, 0.1, 1.7], ...
                                [0.2, 0.2, 1.7]};
                            
            % Iterate over the number of chemicals to mix
            for i = 1:numOfChem
                %% Retrieve Chem
                % Extract chemical name and its location from chemicals2mix array
                chemical = chem2mix{i}{1};  % Name of the chemical
                locationIndex = chem2mix{i}{2};  % Index of the test tube location
                fprintf('Picking up %s from test tube %d...\n', chemical, locationIndex);
                fprintf('\n');
                
                % Move to the test tube location to pick up the chemical
                finishPos = testTubeLocation{locationIndex};  % Test tube location 
                
                % Move UR3 to the test tube
                obj.movementController.Move2Global(finishPos, obj.environment.rUR3);
                
                % self.GripperClose();
                fprintf('Gripper closing to pick up %s...\n', chemical);
                fprintf('\n');
                
                % Move to the mixing location
                finishPos = testTubeLocation{mixingLocation};  % Mixing location 
                fprintf('Moving %s to the mixing location at test tube %d...\n', chemical, mixingLocation);
                fprintf('\n');
                
                % Move robot to the mixing location with the chemical
                obj.movementController.Move2Global(finishPos, obj.environment.rUR3);
                
                %% Mix Chem
                % self.PourChem(); 
                fprintf('Pouring %s into test tube at the mixing location...\n', chemical);
                fprintf('\n');
                
                %% Return Chem
                % Move back to the original test tube location to return the tube
                finishPos = testTubeLocation{locationIndex};  % Back to the original location
                fprintf('Returning test tube %d to its original position...\n', locationIndex);
                fprintf('\n');
                
                % Move robot back to return the test tube
                obj.movementController.Move2Global(finishPos, obj.environment.rUR3);
                
                % self.GripperOpen(); 
                fprintf('Gripper opening to release test tube %d...\n', locationIndex);
                fprintf('\n');
            end
        end

        function GUITeach_UR3(obj)
            %% Call Teach Function 
            robot = obj.environment.rUR3;
            obj.GUI_Func.GUITeachUR3(robot);
       
        end

        function GUITeach_CustomBot(obj)
            %% Call Teach Function 
            robot = obj.environment.rCustomBot;
            obj.GUI_Func.GUITeachCustomBot(robot);

        end
    end
end
 