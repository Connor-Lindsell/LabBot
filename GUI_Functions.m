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

         % Enable control method
        function enableControl(obj)
            obj.controlEnabled = true;
            disp('Control enabled in GUI_Functions');

        end

        % Disable control method
        % function disableControl(obj)
        %     obj.controlEnabled = false;
        %     disp('Control disabled in GUI_Functions');
        % end

        %% Joint Movement with Integrated Jogging and DLS
        function JointMovement(obj, robotName, joint, value, isJogging)
            % Move the joint based on either direct position control or jogging (velocity-based control)
            
            % Select the robot based on the robotName argument
            switch robotName
                case 'UR3'
                    robot = obj.rUR3;
                case 'LabBot'
                    robot = obj.rLabBot;
                otherwise
                    error('Unknown robot specified.');
            end

            if obj.controlEnabled

                disp(['Moving joint ', num2str(joint), ' for ', robotName]);

                % Read the current joint configuration
                currentQ = robot.getpos();
                
                if isJogging
                    % Velocity-based movement using Damped Least Squares (DLS)
                    f = zeros(6, 1);  % Zero force vector for 6-DOF
                    f(joint) = value;  % Assign the force (or velocity) for jogging

                    % Admittance scheme to convert force into velocity command
                    Ka = diag([0.3 0.3 0.3 0.5 0.5 0.5]);
                    dx = Ka * f;

                    % DLS Inverse Jacobian to calculate joint velocity
                    J = robot.jacobe(currentQ);
                    lambda = 0.1;
                    Jinv_dls = inv((J' * J) + lambda^2 * eye(6)) * J';

                    % Calculate joint velocity
                    dq = Jinv_dls * dx;

                    % Apply joint velocity to update joint configuration
                    dt = 0.1;  % Time step
                    newQ = currentQ + (dq' * dt);

                    % Animate the robot to the new configuration
                    robot.animate(newQ);
                    fprintf('Jogging robot joint %d with force %.2f, updated joint configuration.\n', joint, value);
                else
                    % Direct position-based control
                    currentQ(joint) = deg2rad(value);
                    robot.animate(currentQ);
                    fprintf('Moved robot joint %d to %.2f degrees.\n', joint, value);
                end
            else
                disp('Control is disabled. Please turn the switch on.');
            end
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