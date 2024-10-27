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
        rLabBot

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
                case 1
                    % Call SimultaneousControl function
                    obj.DemonstrationControl();
    
                case 2
                    % Call GUI interface function
                    obj.GUI_InterfaceControl();
    
                case 3
                    % Call workspace calculation function
                    obj.WorkspaceCalculation();
    
                otherwise
                    % If an unknown case is provided, display an error message
                    error('Unknown control case specified: %d', controlCase);
            end
        end
    end


    %% Functions
    methods 
        %% Control Through GUI Interface
        function GUI_InterfaceControl(obj)
            % Start GUI and environment, but wait for the On switch to enable controls
        
            % Create the GUI
            obj.guiApp = GUI();  % Start GUI

                        
            %% Environment  
            % Initialize the environment
            disp('Initializing environment...');
            obj.environment.InitialiseEnvironment();  % Logic to initialize the environment

            %%            

            % Wait for the GUI to be closed
            uiwait(obj.guiApp.UIFigure);
        end
   
   
                

        %% Demonstration of Robot Control
        function DemonstrationControl(obj)
            clf;
                       
            %% Initialisation of Enviorment 
            obj.environment.InitialiseEnvironment();
                              
            %% Trasforms
            % UR3 End Effector Goal Destinations 
            % Cheecking for correct orientation
            UR3_Pos1 = [0.3,0.2,2.2]; % positive x Q++
            UR3_Pos2 = [-0.3,0.2,2.2]; % negative x Q-+
            UR3_Pos3 = [0.2,0.3,2.2]; % positive y Q++
            UR3_Pos4 = [0.2,-0.3,2.2]; % negative y Q+-
            UR3_Pos5 = [0.3,-0.2,2.2]; % positive x Q+-
            UR3_Pos6 = [-0.3,-0.2,2.2]; % negative x Q--
            UR3_Pos7 = [-0.2,0.3,2.2]; % positive y Q-+
            UR3_Pos8 = [-0.2,-0.3,2.2]; % negative y Q--
              
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
            % obj.movementController.Move2Global(LabBot_Pos1, self.rLabBot);
            % obj.movementController.Move2Global(LabBot_End, self.rLabBot);             

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

        function WorkspaceCalculation(obj)
            %% Initialisation of Enviorment 
            obj.environment.InitialiseEnvironment();

            %% Workspace Calculation 
            obj.wrkspaceCalc.WorkspaceCalc();

        end
    end
end
 