classdef LabBotMainControl
    %% Properties
    properties 
        % Objects of Parent classes
        movementController
        enviornment

        % Initialise Robot models variables
        rUR3
        rLabBot
    end 

    %% Constructor method
    methods
        function obj = LabBotMainControl
            % Initialise Parent classes 
            obj.movementController = LabBotMovementControl();
            obj.enviornment = LabBotEnviornment();

            % Call SimultaneousControl
            obj.DemonstrationControl();

        end
    end 

    %% Functions
    methods 
        %% Control Through GUI Interface
        function GUI_InterfaceControl(obj)
            % Implement functions to call 

            % Call Mix Chem

            % Call Cartesian movement

            % Call Joint Movement
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
