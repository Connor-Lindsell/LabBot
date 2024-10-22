classdef GUI_Functions
    %% Properties
    properties
        controlEnabled = false; 

        % Objects of Parent classes
        movementController

        % Initialise Robot models variables
        rUR3
        rLabBot

    end

    %% Constructor method
    methods
        function obj = GUI_Functions
            % Initialise Parent classes 
            obj.movementController = LabBotMovementControl();
        end
    end
   
    %% Functions
    methods 
        % Enable control
        function EnableControl(obj)
            obj.controlEnabled = true;
        end

        % Disable control
        function DisableControl(obj)
            obj.controlEnabled = false;
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
        
        function MixChem(self, numOfChem, chem2mix, mixingLocation)
            % Define test tube locations (you could adjust this as per your environment)
            testTubeLocation = {[0.2, -0.2, 0.2], ...
                                [0.2, -0.1, 0.2], ...
                                [0.2, 0, 0.2], ...
                                [0.2, 0.1, 0.2], ...
                                [0.2, 0.2, 0.2]};
                            
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
                obj.movementController.Move2Global(finishPos, self.rUR3);
                
                % self.GripperClose();
                fprintf('Gripper closing to pick up %s...\n', chemical);
                fprintf('\n');
                
                % Move to the mixing location
                finishPos = testTubeLocation{mixingLocation};  % Mixing location 
                fprintf('Moving %s to the mixing location at test tube %d...\n', chemical, mixingLocation);
                fprintf('\n');
                
                % Move robot to the mixing location with the chemical
                obj.movementController.Move2Global(finishPos, self.rUR3);
                
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
                obj.movementController.Move2Global(finishPos, self.rUR3);
                
                % self.GripperOpen(); 
                fprintf('Gripper opening to release test tube %d...\n', locationIndex);
                fprintf('\n');
            end
        end

    end 


end 