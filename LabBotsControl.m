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
            axis([-1 1 -1 1 0 1]);  % Set the axis limits to fit all objects in the environment
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
            rpy = self.RollPitchYawCalc(finishTr);
            targetTransform = transl(finishTr) * rpy;
        
            % Display the target transform
            fprintf('Target Transform (before Position IK):\n');
            disp(targetTransform);
            
            %% Stage 1: Position IK
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
        
            % Display qPos
            fprintf('Initial qPos (Stage 1 solution) = \n');
            fprintf('\n [');
        
            fprintf('  %.5f  ', qPos);  % Display all joint angles in a row
            fprintf(']\n');
            fprintf('\n [');
        
            fprintf('  %.5f  ', qPosAngles);  % Display all joint angles in a row (degrees)
            fprintf(']\n');
            fprintf('\n');
        
            % Animate from current position to qPos (initial joint configuration)
            currentJointConfig = robot.model.getpos();  % Get current joint configuration
            numSteps = 50;  % Number of steps for smooth animation to qPos
            qMatrix = jtraj(currentJointConfig, qPos, numSteps);  % Trajectory from current position to qPos
        
            % Animate the robot moving to qPos
            for i = 1:numSteps
                % Check for self-collision during the movement
                if self.selfCollisionCheck(robot, qMatrix(i, :))
                    warning('Self-collision detected! Stopping movement.');
                    return;
                end
                
                robot.model.animate(qMatrix(i, :));
                drawnow();
                pause(0.02);  % Adjust pause for speed of animation
            end
        
            fprintf('Stage 1 complete, starting RMRC for orientation and fine position...\n\n');
        
            %% Stage 2: RMRC for fine orientation and position adjustment
            % RMRC parameters
            deltaT = 0.02;  % Control frequency (time step for RMRC)
            RMRCsteps = 100;  % Number of increments
            epsilon = 0.1;  % Threshold for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix for the velocity vector
        
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
        
                % Check for self-collision during RMRC
                if self.selfCollisionCheck(robot, currentConfig)
                    warning('Self-collision detected during RMRC! Stopping movement.');
                    return;
                end
        
                % Check if the matrix becomes singular
                if abs(det(intermediateTransform)) < 1e-10
                    warning('Matrix is singular or near-singular at step %d. Skipping this step.', i);
                    continue;
                end
        
                % Compute the position error and orientation error
                deltaX = targetPos - transl(currentTransform).';
                Rd = targetRot;
                Ra = t2r(currentTransform);
                Rdot = (1 / deltaT) * (Rd - Ra);
                S = Rdot * Ra';
        
                % Compute linear and angular velocities
                linear_velocity = (1 / deltaT) * deltaX;
                angular_velocity = [S(3, 2); S(1, 3); S(2, 1)];
        
                % Compute end-effector velocity
                xdot = W * [linear_velocity; angular_velocity];
        
                % Compute Jacobian and manipulability
                J = robot.model.jacob0(currentConfig);
                m = sqrt(det(J * J'));
                if m < epsilon
                    lambda = (1 - m / epsilon) * 5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J' * J + lambda * eye(6)) * J';
        
                % Solve for the next joint velocities
                qdot = (invJ * xdot).';
        
                % Update joint configuration
                currentConfig = currentConfig + deltaT * qdot;
        
                % Animate the robot to the new configuration
                robot.model.animate(currentConfig);
                drawnow();
                pause(deltaT);
        
                % Update the current transform for the next iteration
                currentTransform = robot.model.fkine(currentConfig);
            end
        
            % Display the final q values after RMRC
            finalQ = currentConfig;
            finalQAngles = rad2deg(finalQ);
        
            fprintf('Final q values (after RMRC) = \n');
            fprintf('\n [');
        
            fprintf('  %.5f  ', finalQ);  % Display all joint angles in a row
            fprintf(']\n');
            fprintf('\n [');
        
            fprintf('  %.5f  ', finalQAngles);  % Display all joint angles in a row (degrees)
            fprintf(']\n');
            fprintf('\n');
        
            % Extract the final roll, pitch, and yaw from the final transform
            finalRPY = tr2rpy(currentTransform);
            finalRPYDegrees = rad2deg(finalRPY);
        
            fprintf('Final Roll, Pitch, Yaw (degrees) = \n');
            fprintf('Roll: %.5f  Pitch: %.5f  Yaw: %.5f\n\n', finalRPYDegrees(1), finalRPYDegrees(2), finalRPYDegrees(3));
        
            % pause(1);
        
            % Display movement complete
            fprintf("*********************************MOVEMENT COMPLETE*********************************\n\n");
        end




        %% Calculate Roll Pitch Yaw Orientation
        % Calculates the Roll Pitch and Yaw of the end effector in relation
        % to the location of the finsih transform to the base of the Robot
        %
        % Input:
        % FinishTr: the end location of the robot end effector 
        % 
        % Output:
        % rpy: the orientation of the end effector

        function rpy = RollPitchYawCalc(self, finishTr)
            % Extract X and Y coordinates of the finish transform
            x = finishTr(1);
            y = finishTr(2);
            
            % Default orientation: Z points in the positive X direction, Y remains vertical
            rpy = trotz(0) * troty(0);
                    
            % Determine the quadrant and set the orientation using yaw (Z rotation) and pitch (Y rotation)
            if x >= 0 && y >= 0  % First quadrant (both X and Y are positive)
                if x >= y
                    % Z points in the positive X direction
                    rpy = trotx(0) * troty(pi/2) * trotz(pi/2);  
                    disp("Quadrant 1 (Positive X Positive Y), Segment 1 (X Direction)\n");
                else
                    % Z points in the positive Y direction

                    % This should be the correct rotation
                    % rpy = trotx(-pi) * troty(0) * trotz(-pi/2);  

                    % This is the correct rotation according to trial and error
                    rpy = trotx(-pi/2) * troty(0) * trotz(pi);

                    disp("Quadrant 1 (Positive X Positive Y), Segment 2 (Y Direction)\n");
                end
        
            elseif x < 0 && y >= 0  % Second quadrant (X negative, Y positive)
                if abs(x) >= y
                    % Z points in the negative X direction

                    % This should be the correct rotation
                    % rpy = trotx(-pi/2) * troty(-pi/2) * trotz(0);  

                    % This is the correct rotation according to trial and error
                    rpy = trotx(-pi/2) * troty(-pi/2) * trotz(pi);

                    disp("Quadrant 2 (Negative X Positive Y), Segment 1 (-X Direction)\n");
                else
                    % Z points in the positive Y direction
                    % This should be the correct rotation
                    % rpy = trotx(pi) * troty(0) * trotz(-pi/2);   

                    % This is the correct rotation according to trial and error
                    rpy = trotx(-pi/2) * troty(0) * trotz(-pi);  
                    disp("Quadrant 2 (Negative X Positive Y), Segment 2 (Y Direction)\n");
                end
        
            elseif x < 0 && y < 0  % Third quadrant (both X and Y are negative)
                if abs(x) >= abs(y)
                    % Z points in the negative X direction

                    % This should be the correct rotation
                    % rpy = trotx(0) * troty(-pi/2) * trotz(pi/2);  

                    % This is the correct rotation according to trial and error
                    rpy = trotx(pi) * troty(-pi/2) * trotz(pi/2);  

                    disp("Quadrant 3 (Negative X Negative Y), Segment 1 (-X Direction)\n");
                else
                    % Z points in the negative Y direction
                    rpy = trotx(pi/2) * troty(0) * trotz(0);  
                    disp("Quadrant 3 (Negative X Negative Y), Segment 2 (-Y Direction)\n");
                end
        
            else  % Fourth quadrant (X positive, Y negative)
                if x >= abs(y)
                    % Z points in the positive X direction
                    rpy = trotx(pi/2) * troty(pi/2) * trotz(0);  
                    disp("Quadrant 4 (Positive X Negative Y), Segment 1 (X Direction)\n");
                else
                    % Z points in the negative Y direction
                    rpy = trotx(pi/2) * troty(0) * trotz(0);  
                    disp("Quadrant 4 (Positive X Negative Y), Segment 2 (-Y Direction)\n");
                end
            end
        
            
        end


        %% Initial Guess for Inverse Kinematics
        % Calculates the Initial guess for the ikine function based of the 
        % end effector position in relation to the origin of the robot
        %
        % Input:
        % FinishTr: the end location of the robot end effector 
        % 
        % Output:
        % initialGuess: the initial guess for the inverse kinematics
        
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
            % Collision checking between non-adjacent links using ellipsoids
            % Assume initially no collision
            isCollision = false;
            
            % Define ellipsoid radii for each link (example values, adjust accordingly)
            ellipsoidRadii = [0.1 0.05 0.05];  % radii for each ellipsoid
            
            % Get the number of links in the robot
            numLinks = robot.model.n;
            
            % Calculate the transformation for each link
            linkTransforms = cell(1, numLinks);
            for i = 1:numLinks
                linkTransforms{i} = robot.model.A(1:i, q);  % Forward kinematics of each link
            end
            
            % Iterate over pairs of non-adjacent links to check for collisions
            for i = 1:numLinks
                for j = i+2:numLinks  % Skip adjacent links
                    % Get the positions of the links as points (e.g., midpoints)
                    link1Pos = transl(linkTransforms{i}.T);
                    link2Pos = transl(linkTransforms{j}.T);
                    
                    % Define ellipsoid center points
                    centerPoint1 = link1Pos;
                    centerPoint2 = link2Pos;
                    
                    % Check for intersection between two ellipsoids using algebraic distance
                    algebraicDist = self.GetAlgebraicDist(centerPoint2, centerPoint1, ellipsoidRadii);
                    
                    if algebraicDist < 1  % If algebraic distance < 1, ellipsoids intersect
                        isCollision = true;
                        return;
                    end
                end
            end
        end
        
        % Helper function to compute algebraic distance for ellipsoid collision
        function algebraicDist = GetAlgebraicDist(self, point, center, radii)
            % Calculate the algebraic distance of a point to the surface of an ellipsoid
            % centered at 'center' with 'radii'
            diff = (point - center) ./ radii;  % Normalize the point relative to ellipsoid axes
            algebraicDist = sum(diff .^ 2);
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
