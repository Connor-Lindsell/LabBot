classdef LabBotsControl
    %% Properties    
    properties
        rUR3
        % rLabBot
        
        steps = 50;
        
    end

    %% Constructor method
    methods
        function self = LabBotsControl
            clc;
            self.SimultaneousControl

        end
    end 

    %% Functions
    methods
        function SimultaneousControl(self)
            clf;

            % Initaialising Robot Models
            self.rUR3 = UR3;
            % rLabBot = LabBot;

            self.rUR3.model.plot(zeros(1, self.rUR3.model.n));

            %% Base transforms

            %% Set Up Enviorment 

            % Call Enviorment class here

            % Temorary Enviorment 
            % Configure the axes and labels for the environment
            axis([-1.6 1.6 -1 1 0 1.5]);  % Set the axis limits to fit all objects in the environment
            xlabel('X-axis');  % Label the X-axis
            ylabel('Y-axis');  % Label the Y-axis
            zlabel('Z-axis');  % Label the Z-axis
            grid on;  % Display a grid for better visualization of object positions
            hold on;  % Keep the plot active for additional elements

            
            %% Trasforms
            % UR3 End Effector Goal Destinations 
            UR3_Pos1 = [0.3,0.2,0.2]; % positive x
            UR3_Pos2 = [-0.2,-0.3,0.2]; % negative x
            UR3_Pos3 = [0.3,0.2,0.2]; % positive y
            UR3_Pos4 = [0.3,0.2,0.2]; % negative y
            UR3_Pos5 = [0.3,0.2,0.2]; % positive x
            UR3_Pos6 = [0.3,0.2,0.2]; % positive x
            UR3_Pos7 = [0.3,0.2,0.2]; % positive y
            UR3_Pos8 = [0.3,0.2,0.2]; % positive y
              
            % LabBot End Effector Goal Destinations 
            % LabBot_Pos1 = [0.2,0.2,0.2];
            % LabBot_End = [0.2,0.2,0.2];


            %% Perform Movements
            % Calling Move2Global using self
            % For UR3
            self.Move2Global(UR3_Pos1, self.rUR3);
            self.Move2Global(UR3_Pos2, self.rUR3);
            
            % For LabBot
            % self.Move2Global(LabBot_Start, LabBot_Pos1, rLabBot);
            % self.Move2Global(LabBot_Pos1, LabBot_End, rLabBot);             

            %% Mix Chemicals 

            % Not nessecary while testing optimisation 
            % % Define chemicals to mix and their test tube locations
            % chemicals2mix1 = {{'Bromine', 1}, ...  % Notice the use of curly braces
            %                   {'Iodine', 2}, ... 
            %                   {'Nitrogen', 4} ...
            %                   };
            % 
            % % Call MixChem with 3 chemicals and mixing location 3
            % MixChem(self, 3, chemicals2mix1, 3)
            % 
            % chemicals2mix2 = {{'New Mixture', 3}, ... 
            %                   {'Nitrogen', 4} ...
            %                   };
            % 
            % % Call MixChem with 2 chemicals and mixing location 5
            % MixChem(self, 2, chemicals2mix2, 5)
                    
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
            testTubeLocation = {[0.2, 0.21, 0.2], ...
                                [0.2, 0.22, 0.2], ...
                                [0.2, 0.23, 0.2], ...
                                [0.2, 0.24, 0.2], ...
                                [0.2, 0.25, 0.2]};
                            
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
                self.Move2Global(finishPos, self.rUR3);
                
                % self.GripperClose();
                fprintf('Gripper closing to pick up %s...\n', chemical);
                fprintf('\n');
                
                % Move to the mixing location
                finishPos = testTubeLocation{mixingLocation};  % Mixing location 
                fprintf('Moving %s to the mixing location at test tube %d...\n', chemical, mixingLocation);
                fprintf('\n');
                
                % Move robot to the mixing location with the chemical
                self.Move2Global(finishPos, self.rUR3);
                
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
                self.Move2Global(finishPos, self.rUR3);
                
                % self.GripperOpen(); 
                fprintf('Gripper opening to release test tube %d...\n', locationIndex);
                fprintf('\n');
            end
        end


        %% Move To Global Function
        % Moves the selected Robot Arm From a Start Position to  Finish
        % Position
        % 
        % Inputs -  
        % Finish Transform: the end location of the robot end effector 
        % Robot: calls the robot that is required to move
        

        function Move2Global(self, finishTr, robot)      
            %% Inverse Kinematics with Optimization
            % Calculate the target transform including orientation
            rpy = self.RollPitchYawCalc(finishTr);
            targetTransform = transl(finishTr) * rpy;
            
            % Optimize movement to reach the target transform
            optimizedConfiguration = self.OptimizeMovement(targetTransform, robot);
            
            % If optimization fails, return early
            if isempty(optimizedConfiguration)
                fprintf('Optimization failed. Unable to move to the goal position.\n');
                return;
            end
        
            %% Joint Trajectory
            % Get the current joint configuration of the robot
            startConfiguration = robot.model.getpos();
            
            % Generate joint space trajectory from start to optimized configuration
            qMatrix = jtraj(startConfiguration, optimizedConfiguration, self.steps);
            
        
            %% Animation
            % Animate the robot through the combined trajectory
            for i = 1:size(qMatrix, 1)
                robot.model.animate(qMatrix(i, :));
                drawnow();
                pause(0.05);
            end
            hold on
        
        end

        %% Calculate Roll Pitch Yaw Orientation
        % Calculates the Roll Pitch and Yaw of the end effector in relation
        % to the location of the finsih transform to the base of the Robot
        %
        % Input:
        % FinishTr: the end location of the robot end effector 
        % 
        % Rutput:
        % rpy: the orientation of the end effector

        % TO DO: 
        % The angles aren't pointing in the right direction, have to go through
        % all of them to make sure the z is pointing in the right direction
        % and the y is pointing along the positive z

        function rpy = RollPitchYawCalc(self, finishTr)
            % Extract X and Y coordinates of the finish transform
            x = finishTr(1);
            y = finishTr(2);
            
            % Default orientation: Z points in the positive X direction, Y points up
            rpy = trotz(0) * troty(pi/2);
            
            % Determine the quadrant and set the orientation
            if x >= 0 && y >= 0  % First quadrant (both X and Y are positive)
                if x >= y
                    % Z points in the positive X direction
                    rpy = trotz(pi/2) * troty(pi/2);
                else
                    % Z points in the positive Y direction
                    rpy = trotz(pi/2) * trotx(pi/2);
                end
            elseif x < 0 && y >= 0  % Second quadrant (X negative, Y positive)
                if abs(x) >= y
                    % Z points in the negative X direction
                    rpy = trotz(-pi/2) * troty(-pi/2);
                else
                    % Z points in the positive Y direction
                    rpy = trotz(pi/2) * trotx(pi/2);
                end
            elseif x < 0 && y < 0  % Third quadrant (both X and Y are negative)
                if abs(x) >= abs(y)
                    % Z points in the negative X direction
                    rpy = trotz(pi) * troty(-pi/2);
                else
                    % Z points in the negative Y direction
                    rpy = trotz(-pi/2) * trotx(-pi/2);
                end
            else  % Fourth quadrant (X positive, Y negative)
                if x >= abs(y)
                    % Z points in the positive X direction
                    rpy = trotz(0) * troty(pi/2);
                else
                    % Z points in the negative Y direction
                    rpy = trotz(-pi/2) * trotx(-pi/2);
                end
            end
        end

        %% Collision Checking 
        % collision checking for the Robot arm 
        function isCollision = selfCollisionCheck(self, robot, q)
            % Assume initially no collision
            isCollision = false;
            
            % Get the number of links in the robot
            numLinks = robot.model.n;
            
            % Calculate the transformation for each link
            linkTransforms = cell(1, numLinks);
            for i = 1:numLinks
                linkTransforms{i} = robot.model.A(1:i, q);
            end
            
            % Iterate over pairs of links to check for collisions
            for i = 1:numLinks
                for j = i+2:numLinks  % Skip adjacent links
                    % Get the positions of the links as points (e.g., midpoints)
                    link1Pos = transl(linkTransforms{i}.T);
                    link2Pos = transl(linkTransforms{j}.T);
                    
                    % Use a simple distance check for now
                    distance = norm(link1Pos - link2Pos);
                    if distance < 0.05  % Threshold for collision detection
                        isCollision = true;
                        return;
                    end
                end
            end
        end


        %% Optimised Movement
        function optimizedConfiguration = OptimizeMovement(self, targetTransform, robot)
            % Initial joint configuration (use the current robot position)
            initialGuess = robot.model.getpos();
            
            % Joint limits (adjust these based on your robot specs)
            jointMin = [deg2rad(-180), -pi, deg2rad(-180), deg2rad(-180), deg2rad(-180), deg2rad(-180)];
            jointMax = [deg2rad(180), 0, deg2rad(180), deg2rad(180), deg2rad(180), deg2rad(180)];
            
            % Define the optimization problem using fmincon
            options = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
                'Display', 'iter', 'MaxIterations', 100);
            
            % Define the objective function
            objectiveFunction = @(q) self.ObjectiveFunction(q, targetTransform, robot);
            
            % Set up constraints: joint limits and simple collision detection
            constraints = @(q) self.JointConstraints(q, jointMin, jointMax, robot);
            
            % Run the optimization
            [optimizedConfiguration, fval, exitflag] = fmincon(objectiveFunction, initialGuess, [], [], [], [], jointMin, jointMax, constraints, options);
            
            % Check if optimization succeeded
            if exitflag <= 0
                optimizedConfiguration = [];  % Return empty if optimization failed
                warning('Optimization did not converge to a solution.');
            else
                fprintf('Optimization completed successfully.\n');
            end
        end

        %% Objective Function 
        function cost = ObjectiveFunction(self, q, targetTransform, robot)
            % Forward kinematics of the robot with the current joint configuration
            T = robot.model.fkine(q);
            
            % Calculate the error as the difference between the current and target transforms
            positionError = norm(transl(T) - transl(targetTransform));
            orientationError = norm(t2r(T) - t2r(targetTransform), 'fro');
            
            % Combine errors into a cost (weighted sum)
            cost = positionError + 0.1 * orientationError;  % Adjust weights as needed
        end
        
        %% Joint Constraints 
        function [c, ceq] = JointConstraints(self, q, jointMin, jointMax, robot)
            % No equality constraints
            ceq = [];
            
            % 1. Joint Limit Constraints
            % Ensure joint angles do not exceed their limits
            jointLimitUpper = q - jointMax; % Difference between joint values and upper limits
            jointLimitLower = jointMin - q; % Difference between joint values and lower limits
            
            % Collect joint limit constraints into 'c'
            c = [jointLimitUpper; jointLimitLower];
            
            % Note: No collision or ground clearance constraints for now
        end


    
        %% Get End Effector Pos Function 
        % Gets the end effector position of the robot
        % 
        % Input:
        % robot: The robot you want to find the end effector position 
        % 
        % Output:
        % endEffectorPos: The end effector position of the robot 


        function endEffectorPos = getEndEffectorPos(self, robot)
            % Get the current joint positions of the robot
            qCurrent = robot.model.getpos();
        
            % Calculate the transformation matrix for the end-effector
            T = robot.model.fkine(qCurrent);
        
            % Extract the position of the end-effector (X, Y, Z)
            endEffectorPos = transl(T);
        
        end
        
    end
    
end