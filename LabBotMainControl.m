  
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

        % E-Stop Properties
        isStopped = false;     
        estopTimer                 
    end 

    %% Constructor method
    methods
        function obj = LabBotMainControl(controlCase) 
            %% Initialise Enviornment
            obj.environment = LabBotEnvironment();
            disp('Initialising environment...');
            obj.environment.InitialiseEnvironment();  
            disp('Enviornment Initialised');

            % Retrieve the Table instance from environment and pass to movement control
            obj.movementController = LabBotMovementControl(obj.environment.Table,obj.environment.rUR3,obj.environment.objects,obj.environment.rCustomBot);
            obj.GUI_Func = GUI_Functions(obj.environment.rUR3,obj.environment.rCustomBot); 

            % Initialise Parent classes
            obj.wrkspaceCalc = LabBotCalculations();
            % obj.GUI_Func = GUI_Functions(); 
            % obj.movementController = LabBotMovementControl();

            % Start the E-Stop listener
            obj.startEstopListener();
            
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
                    obj.GUITeach_UR3();

                case 5 % teach
                    % Call workspace calculation function
                    obj.GUITeach_CustomBot();

                case 6 % teach
                    % Call workspace calculation function
                    obj.forceCollision();

                % case 0 % Default initialization, no action
                %     % Do nothing, simply return the class
                %     disp('LabBotMainControl initialized with no action.');
                % 
    
                otherwise
                    % Debugging Line
                    fprintf('Received controlCase: %d\n', controlCase);

                    % If an unknown case is provided, display an error message
                    error('Unknown control case specified: %d', controlCase);
            end
                    
        end

        function startEstopListener(obj)
            % Timer to monitor E-Stop state
            obj.estopTimer = timer('ExecutionMode', 'fixedRate', ...
                                   'Period', 0.1, ...
                                   'TimerFcn', @(~,~)obj.checkEstop());
            start(obj.estopTimer);
        end

        function checkEstop(obj)
            if obj.isStopped == true
                disp('E-Stop detected - stopping all operations.');
                return
            end
        end

        function delete(obj)
            % Stop the timer
            stop(obj.estopTimer);
            delete(obj.estopTimer);
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

            while 1

                % pendant = GUI;
                % 
                % GUI_Case = pendant.read5;
                
                GUI_Case = 4;

                disp(GUI_Case);

                switch GUI_Case
                    case 1 
                        obj.GUI_Func.GUITeachUR3();
                            
                    case 2
                        obj.GUI_Func.GUITeachCustomBot();
        
                    case 3 
                        obj.GUI_Func.GUICartesianUR3();
                        
                    case 4 
                        obj.GUI_Func.GUICartesianCustomBot();   
                    case 5
                        % nothing to be called
                end


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

            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Errors: have to comment out which one you dont want out 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Call Listening function 
            % obj.GUI_Func.GUITeachUR3();

            % obj.GUI_Func.GUITeachCustomBot();

            % obj.GUI_Func.GUICartesianUR3();

            % obj.GUI_Func.GUICartesianCustomBot();
            
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
        end                

        %% Demonstration of Robot Control

        function DemonstrationControl(obj)
            disp('Demonstration Control Commencing')

            tableHeight = 0.9;
            xOffset = 1;
            yOffset = 0;
            zOffset = 0.075;

            
            %% Initialisation of Enviorment   
            % Initialize the environment
            % disp('Initialising environment...');
            % obj.environment.InitialiseEnvironment();  
            % disp('Enviornment Initialised');
                              
            %% Trasforms
            % UR3 End Effector Goal Destinations 
            % Cheecking for correct orientation
            UR3_Pos1 = [0+xOffset ,0.5,tableHeight+zOffset]; % positive x Q++
            UR3_Pos2 = [-0.3+xOffset,0.2,tableHeight+zOffset]; % negative x Q-+
            UR3_Pos3 = [0.2+xOffset,0.3,tableHeight+zOffset]; % positive y Q++
            UR3_Pos4 = [0.2+xOffset,-0.3,tableHeight+zOffset]; % negative y Q+-
            UR3_Pos5 = [0.3+xOffset,-0.2,tableHeight+zOffset]; % positive x Q+-
            UR3_Pos6 = [-0.3+xOffset,-0.2,tableHeight+zOffset]; % negative x Q--
            UR3_Pos7 = [-0.2+xOffset,0.3,tableHeight+zOffset]; % positive y Q-+
            UR3_Pos8 = [0.6+xOffset,-0.3,tableHeight+zOffset]; % negative y Q--
              
            % LabBot End Effector Goal Destinations 
            % LabBot_Pos1 = [0.2,0.2,0.2];
            % LabBot_End = [0.2,0.2,0.2];

            
            obj.isStopped = false;  % Reset E-Stop flag at the beginning

            % Sample loop (replace this with your actual movement loop)
            for i = 1:100
                % checkEstop(obj);
                % Check for E-Stop
                if obj.isStopped == true
                    disp('E-Stop activated, stopping demonstration.');
                    % pause(0.5);
                    break;
                end

                %% Perform Movements
                % Calling Move2Global using self
                % For UR3
                obj.movementController.Move2Global(UR3_Pos1, 'UR3');
                obj.movementController.Move2Global(UR3_Pos2, 'UR3');
                obj.movementController.Move2Global(UR3_Pos3, 'UR3');
                obj.movementController.Move2Global(UR3_Pos4, 'UR3');
                obj.movementController.Move2Global(UR3_Pos5, 'UR3');
                obj.movementController.Move2Global(UR3_Pos6, 'UR3');
                obj.movementController.Move2Global(UR3_Pos7, 'UR3');
                obj.movementController.Move2Global(UR3_Pos8, 'UR3');
                
                % For LabBot
                % obj.movementController.Move2Global(LabBot_Pos1, 'CustomBot');
                % obj.movementController.Move2Global(LabBot_End, 'CustomBot');             
    
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
                pause(0.1);  % Simulating time-consuming task
            end
                   
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
            testTubeLocation = {[1, 0.5, 0.8], ...
                                [1.2, 0.5, 0.8], ...
                                [1.4, 0.5, 0.8], ...
                                [1.2, 0.1, 0.8], ...
                                [1.2, 0.2, 0.8]};
                            
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
                obj.movementController.Move2Global(finishPos, 'UR3');
                
                % self.GripperClose();
                fprintf('Gripper closing to pick up %s...\n', chemical);
                fprintf('\n');
                
                % Move to the mixing location
                finishPos = testTubeLocation{mixingLocation};  % Mixing location 
                fprintf('Moving %s to the mixing location at test tube %d...\n', chemical, mixingLocation);
                fprintf('\n');
                
                % Move robot to the mixing location with the chemical
                obj.movementController.Move2Global(finishPos, 'UR3');
                
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
                obj.movementController.Move2Global(finishPos, 'UR3');
                
                % self.GripperOpen(); 
                fprintf('Gripper opening to release test tube %d...\n', locationIndex);
                fprintf('\n');
            end
        end

        function GUITeach_UR3(obj)
            %% Call Teach Function 
            % robot = obj.environment.rUR3;
            obj.GUI_Func.GUITeachUR3();
       
        end

        function GUITeach_CustomBot(obj)
            %% Call Teach Function 
            % robot = obj.environment.rCustomBot;
            obj.GUI_Func.GUITeachCustomBot();

        end


        function forceCollision(obj)
            
            disp('Demonstration Control Commencing')

            tableHeight = 1;
            xOffset = 1;
            yOffset = 0;
            zOffset = 0.075;

            
            %% Initialisation of Enviorment   
            % Initialize the environment
            % disp('Initialising environment...');
            % obj.environment.InitialiseEnvironment();  
            % disp('Enviornment Initialised');
                              
            %% Trasforms
            % UR3 End Effector Goal Destinations 
            % Cheecking for correct orientation
            UR3_Pos1 = [1 , 0.5, tableHeight]; 
            UR3_Pos2 = [1.2, 0.5, tableHeight];
            
              
            % LabBot End Effector Goal Destinations 
            % LabBot_Pos1 = [0.2,0.2,0.2];
            % LabBot_End = [0.2,0.2,0.2];

            
            obj.isStopped = false;  % Reset E-Stop flag at the beginning

            % Sample loop (replace this with your actual movement loop)
            for i = 1:100
                % Check for E-Stop
                if obj.isStopped
                    disp('E-Stop activated, stopping demonstration.');
                    break;
                end

                %% Perform Movements
                % Calling Move2Global using self
                % For UR3
                obj.movementController.Move2Global(UR3_Pos1, 'UR3');
                obj.movementController.Move2Global(UR3_Pos2, 'UR3');
                
                
                % For LabBot
                % obj.movementController.Move2Global(LabBot_Pos1, 'CustomBot');
                % obj.movementController.Move2Global(LabBot_End, 'CustomBot');
            end
        end


    end

end
 