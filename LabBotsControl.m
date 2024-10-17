classdef LabBotsControl
    %% Properties    
    properties
        rUR3
        % rLabBot
        
        steps = 100;
        
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

            %% Robot Initialisation
            % Initaialising Robot Models
            self.rUR3 = UR3;
            % rLabBot = LabBot;
           
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
            % LabBot_Pos1 = [0.2,0.2,0.2];
            % LabBot_End = [0.2,0.2,0.2];


            %% Perform Movements
            % Calling Move2Global using self
            % For UR3
            self.Move2Global(UR3_Pos1, self.rUR3);
            self.Move2Global(UR3_Pos2, self.rUR3);
            self.Move2Global(UR3_Pos3, self.rUR3);
            self.Move2Global(UR3_Pos4, self.rUR3);
            self.Move2Global(UR3_Pos5, self.rUR3);
            self.Move2Global(UR3_Pos6, self.rUR3);
            self.Move2Global(UR3_Pos7, self.rUR3);
            self.Move2Global(UR3_Pos8, self.rUR3);

            
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
    % Calculate the target orientation using RollPitchYawCalc
    [rpy, maskS2] = self.RollPitchYawCalc(finishTr);
    targetTransform = transl(finishTr) * rpy;

    % Display the target transform
    fprintf('Target Transform (before Position IK):\n');
    disp(targetTransform);

    % Stage 1: Position IK
    % Get initial guess based on quadrant
    initialGuess = self.GetInitialGuess(finishTr);

    % Define the mask for position only [111000]
    maskS1 = [1 1 1 0 0 0];

    % Solve inverse kinematics for position using the initial guess
    qPos = robot.model.ikine(targetTransform, 'q0', initialGuess, 'mask', maskS1);

    % Handle errors or failures in position IK
    if isempty(qPos)
        warning('Stage 1 IK failed to find a solution.');
        return;
    end

    qPosAngles = rad2deg(qPos);

    fprintf('qPos = \n');
    fprintf('\n [');
    fprintf('  %.5f  ', qPos);  % Display all joint angles in a row
    fprintf(']\n');
    fprintf('\n [');
    fprintf('  %.5f  ', qPosAngles);  % Display all joint angles in a row
    fprintf(']\n');
    fprintf('\n');

    fprintf('Stage 1 complete, starting RMRC for orientation and fine position...\n');

    % Stage 2: RMRC for fine orientation and position adjustment
    % RMRC parameters
    deltaT = 0.05;   % Time step for RMRC
    RMRCsteps = 100; % Number of increments

    % Get the initial configuration and current transform from Stage 1
    currentConfig = qPos;  % Start from qPos from Stage 1
    currentTransform = robot.model.fkine(currentConfig);

    % Extract initial and target translation and rotation
    initialPos = transl(currentTransform).';  % 3x1 vector
    targetPos = transl(targetTransform);      % 3x1 vector
    initialRot = t2r(currentTransform);       % 3x3 matrix
    targetRot = t2r(targetTransform);         % 3x3 matrix

    % Linear interpolation for position increments
    deltaPos = (targetPos - initialPos) / RMRCsteps;

    % Compute the incremental rotation matrix for each step
    deltaRot = (targetRot * initialRot')^(1 / RMRCsteps);  % Rotation increment

    % Initialize the current rotation matrix
    currentRot = initialRot;

    % RMRC loop over the given steps
    for i = 1:RMRCsteps
        % Update the position incrementally
        currentPos = initialPos + i * deltaPos;  % 3x1 vector

        % Update the rotation incrementally using matrix multiplication
        currentRot = deltaRot * currentRot;  % Maintain orthogonality

        % Construct the intermediate target transform
        intermediateTransform = rt2tr(currentRot, currentPos);

        % Check if the matrix becomes singular
        if abs(det(intermediateTransform)) < 1e-10
            warning('Matrix is singular or near-singular at step %d. Skipping this step.', i);
            continue;
        end

        % Solve IK for the intermediate step using ikcon
        qStep = robot.model.ikcon(intermediateTransform, currentConfig);

        % Handle IK failure
        if isempty(qStep)
            fprintf('RMRC failed at step %d\n', i);
            break;
        end

        % Debugging: Log RMRC progress
        fprintf('RMRC Step %d: Current error: %.10f\n', i, norm(targetPos - currentPos));

        % Animate the robot to the new configuration
        robot.model.animate(qStep);
        drawnow();
        pause(deltaT);

        % Update the starting configuration for the next step
        currentConfig = qStep;
    end

    fprintf("Movement Complete\n\n");
end

       


        % function Move2Global(self, finishTr, robot)
        %     % Calculate the target transform including orientation
        %     rpy = self.RollPitchYawCalc(finishTr);
        %     targetTransform = transl(finishTr) * rpy ;
        % 
        %     % Display the target transform
        %     fprintf('Target Transform (before IK):\n');
        %     disp(targetTransform);
        % 
        %     % Get initial guess based on quadrant
        %     initialGuess = self.GetInitialGuess(finishTr);
        % 
        %     % Define the mask [111000] to solve for XYZ position only
        %     mask = [1 1 1 0 0 0];
        % 
        %     % Solve inverse kinematics for position using the initial guess
        %     qPos = robot.model.ikine(targetTransform, 'q0', initialGuess, 'mask', mask);
        % 
        %     qPosAngles = rad2deg (qPos);
        % 
        %     fprintf('qPos = \n');
        %     fprintf('\n [');
        %     fprintf('  %.5f  ', qPos);  % Display all joint angles in a row
        %     fprintf(']\n');
        %     fprintf('\n [');
        %     fprintf('  %.5f  ', qPosAngles);  % Display all joint angles in a row
        %     fprintf(']\n');
        %     fprintf('\n');
        % 
        %     % Calculate and display the resulting end-effector position using qPos
        %     endEffectorPos1 = robot.model.fkine(qPos);
        %     fprintf('End Effector Position after qPos = \n');
        %     disp(transl(endEffectorPos1));
        %     fprintf('\n');
        % 
        %     % Set optimization options for 'ikine'
        %     options = optimset('TolFun', 1e-6, 'TolX', 1e-8, 'MaxIter', 1000);
        % 
        %     % Solve inverse kinematics for position and orientation using qPos as the guess
        %     maskOrientation = [1 1 1 1 1 1];
        %     qSolution = robot.model.ikine(targetTransform, 'q0', qPos, 'mask', maskOrientation, options);
        % 
        %     qSolutionAngles = rad2deg (qSolution);
        % 
        %     % Debugging: Display the end-effector position after qSolution calculation
        %     if ~isempty(qSolution)
        %         fprintf('qSolution = \n');
        %         fprintf('\n [');
        %         fprintf('  %.5f  ', qSolution);  % Display all joint angles in a row
        %         fprintf(']\n');
        %         fprintf('\n [');
        %         fprintf('  %.5f  ', qSolutionAngles);  % Display all joint angles in a row
        %         fprintf(']\n');
        %         fprintf('\n');
        % 
        %         endEffectorPos2 = robot.model.fkine(qSolution);
        %         fprintf('End Effector Position after qSolution = \n');
        %         disp(transl(endEffectorPos2));
        %         fprintf('\n');
        %     else
        %         fprintf('Inverse kinematics failed to find a solution.\n');
        %         return;
        %     end
        % 
        %     % Joint Trajectory
        %     startConfiguration = robot.model.getpos();
        %     qMatrix = jtraj(startConfiguration, qSolution, self.steps);
        % 
        %     % Animation
        %     for i = 1:size(qMatrix, 1)
        %         robot.model.animate(qMatrix(i, :));
        %         drawnow();
        %         pause(0.05);
        %     end
        %     hold on;
        % 
        %     fprintf("Movement Complete\n");
        %     fprintf('\n');
        % end



        %% Calculate Roll Pitch Yaw Orientation
        % Calculates the Roll Pitch and Yaw of the end effector in relation
        % to the location of the finsih transform to the base of the Robot
        %
        % Input:
        % FinishTr: the end location of the robot end effector 
        % 
        % Rutput:
        % rpy: the orientation of the end effector

        function [rpy, maskS2] = RollPitchYawCalc(self, finishTr)
            % Extract X and Y coordinates of the finish transform
            x = finishTr(1);
            y = finishTr(2);
            
            % Default orientation: Z points in the positive X direction, Y remains vertical
            rpy = trotz(0) * troty(0);
            
            % Initialize the maskS2 for orientation and fine position control
            maskS2 = [1 1 1 1 1 1];  % Default mask includes both position and orientation
        
            % Determine the quadrant and set the orientation using yaw (Z rotation) and pitch (Y rotation)
            if x >= 0 && y >= 0  % First quadrant (both X and Y are positive)
                if x >= y
                    % Z points in the positive X direction
                    rpy = trotz(pi/2) * troty(pi/2);  
                    maskS2 = [1 1 1 0 1 1];  % Example: prioritize orientation for yaw and pitch
                    disp("Quadrant 1, 1 - MaskS2: [111011]");
                else
                    % Z points in the positive Y direction
                    rpy = trotx(-pi) * trotz(-pi/2);  
                    maskS2 = [1 1 1 1 0 1];  % Example: prioritize yaw and roll
                    disp("Quadrant 1, 2 - MaskS2: [111101]");
                end
        
            elseif x < 0 && y >= 0  % Second quadrant (X negative, Y positive)
                if abs(x) >= y
                    % Z points in the negative X direction
                    rpy = trotx(-pi/2) * troty(-pi/2);  
                    maskS2 = [1 1 1 1 1 0];  % Example: control position with full orientation
                    disp("Quadrant 2, 1 - MaskS2: [111110]");
                else
                    % Z points in the positive Y direction
                    rpy = trotx(pi) * trotz(-pi/2);  
                    maskS2 = [1 1 1 1 0 1];  % Control yaw, pitch, and roll
                    disp("Quadrant 2, 2 - MaskS2: [111101]");
                end
        
            elseif x < 0 && y < 0  % Third quadrant (both X and Y are negative)
                if abs(x) >= abs(y)
                    % Z points in the negative X direction
                    rpy = trotz(pi) * troty(-pi/2);  
                    maskS2 = [1 1 1 0 1 1];  % Control yaw and roll
                    disp("Quadrant 3, 1 - MaskS2: [111011]");
                else
                    % Z points in the negative Y direction
                    rpy = trotz(pi/2);  
                    maskS2 = [1 1 1 1 1 0];  % Full orientation and position control
                    disp("Quadrant 3, 2 - MaskS2: [111110]");
                end
        
            else  % Fourth quadrant (X positive, Y negative)
                if x >= abs(y)
                    % Z points in the positive X direction
                    rpy = trotz(pi);  
                    maskS2 = [1 1 1 0 1 1];  % Adjust yaw and roll
                    disp("Quadrant 4, 1 - MaskS2: [111011]");
                else
                    % Z points in the negative Y direction
                    rpy = troty(pi/2) * trotz(pi/2);  
                    maskS2 = [1 1 1 1 1 0];  % Full control
                    disp("Quadrant 4, 2 - MaskS2: [111110]");
                end
            end
        
            % Debug: Display the final rpy matrix and maskS2
            disp("Final rpy matrix:");
            disp(rpy);
            disp("MaskS2 for Stage 2:");
            disp(maskS2);
        end


        %% Initial Guess for Ikine
        function initialGuess = GetInitialGuess(self, finishTr)
            % Extract X and Y coordinates of the finish transform
            x = finishTr(1);
            y = finishTr(2);
            
            % Initialize guess
            initialGuess = zeros(1, 6);  % Adjust based on your robot's number of joints
        
            % Set different initial guesses based on the quadrant
             if x >= 0 && y >= 0  % First quadrant (both X and Y are positive)
                if x >= y
                    % First quadrant (both X and Y are positive)
                    initialGuess = [deg2rad(40), deg2rad(-135), -pi/2, deg2rad(225), deg2rad(-50), 0];
                else
                    % First quadrant (both X and Y are positive)
                    initialGuess = [deg2rad(-130), -pi/4, pi/2, -pi/4, deg2rad(50), 0];
                end
            
             elseif x < 0 && y >= 0  % Second quadrant (X negative, Y positive)
                if abs(x) >= y
                    % Second quadrant (X negative, Y positive)
                    initialGuess = [deg2rad(-40), -pi/4, pi/2, -pi/4, deg2rad(50) 0];
                else
                    % Second quadrant (X negative, Y positive)
                    initialGuess = [deg2rad(130), deg2rad(-135), -pi/2, deg2rad(225), deg2rad(-50), 0];
                end
            
             elseif x < 0 && y < 0  % Third quadrant (both X and Y are negative)
                if abs(x) >= abs(y)
                    % Third quadrant (both X and Y are negative)
                    initialGuess = [deg2rad(-140), deg2rad(-135), -pi/2, deg2rad(225), deg2rad(-50), 0];
                else
                    % Third quadrant (both X and Y are negative)
                    initialGuess = [deg2rad(50), -pi/4, pi/2, -pi/4, deg2rad(50), 0];
                end
            
             else  
                 % Fourth quadrant (X positive, Y negative)
                 if x >= abs(y)
                    % Fourth quadrant (X positive, Y negative)
                    initialGuess = [deg2rad(-50), deg2rad(-135), -pi/2, deg2rad(225), deg2rad(-50), 0];
                 else
                    % Fourth quadrant (X positive, Y negative)
                    initialGuess = [deg2rad(140), -pi/4, pi/2, -pi/4, deg2rad(50), 0];
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
