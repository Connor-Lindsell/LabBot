classdef LabBotMovementControl
    
    properties 
        steps = 25;

        rUR3
        rCustomBot
        table
        objects
    end

    methods 
        function obj = LabBotMovementControl (existingTable, RobotUR3, objects)
            
            obj.table = existingTable;
            obj.rUR3 = RobotUR3;
            obj.objects = objects;
            
            % obj.rUR3 = rUR3;
            % obj.rCustomBot = rCustomBot;
            
        end
    end

    methods
        %% Move To Global Function
        % Moves the selected Robot Arm From a Start Position to  Finish
        % Position
        % 
        % Inputs -  
        % Finish Transform: the end location of the robot end effector 
        % Robot: calls the robot that is required to move
        
        function Move2Global(self, finishTr, robot)

            switch robot
                case 'UR3'
                    robot = self.rUR3;
                case 'LabBot'
                    robot = self.rLabBot;
                otherwise
                    error('Unknown robot specified.');
            end

            % Define home position (initial joint configuration)
            homePosition = zeros(1, robot.model.n);  % Assuming zero configuration as home

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
                % Check for Plane and Ellipsoid collisions
                if i > 1  % Skip for the first step since we have no previous position
                    previousEndEffectorPos = transl(robot.model.fkine(qMatrix(i - 1, :)));
                    currentEndEffectorPos = transl(robot.model.fkine(qMatrix(i, :)));
                    
                    % Plane collision check
                    if self.CheckPlaneCollision(previousEndEffectorPos, currentEndEffectorPos)
                        warning('Collision detected with the table! Stopping movement.');
                        self.returnToHome(robot, currentJointConfig, homePosition); % Return to home position
                        return;
                    end
                end
                                               
                robot.model.animate(qMatrix(i, :));
                drawnow();
                pause(0.02);  % Adjust pause for speed of animation
            end
        
            fprintf('Stage 1 complete, continuing with orientation...\n\n');
        
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

                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Made RMRC failed because took too long
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Check for Table collision during RMRC
                % if i > 1  % Skip for the first step since we have no previous position
                %     previousEndEffectorPos = transl(robot.model.fkine(qMatrix(i - 1, :)));
                %     currentEndEffectorPos = transl(robot.model.fkine(qMatrix(i, :)));
                % 
                %     % Perform collision check
                %     isCollision = self.CheckPlaneCollision(previousEndEffectorPos, currentEndEffectorPos);
                %     if isCollision
                %         warning('Collision detected with the table! Stopping movement.');
                %         return;
                %     end
                % end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
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

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % With collision
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % %% Stage 1: Position IK
            % % Get initial guess based on quadrant
            % initialGuess = self.GetInitialGuess(finishTr);
            % 
            % % Define the mask for position only [111000]
            % maskS1 = [1 1 1 0 0 0];
            % 
            % % Solve inverse kinematics for position using the initial guess
            % qPos = robot.model.ikine(targetTransform, 'q0', initialGuess, 'mask', maskS1);
            % 
            % % Handle errors or failures in position IK
            % if isempty(qPos)
            %     warning('Stage 1 IK failed to find a solution.');
            %     return;
            % end
            % 
            % qPosAngles = rad2deg(qPos);
            % 
            % % Display qPos
            % fprintf('Initial qPos (Stage 1 solution) = \n');
            % fprintf('\n [');
            % 
            % fprintf('  %.5f  ', qPos);  % Display all joint angles in a row
            % fprintf(']\n');
            % fprintf('\n [');
            % 
            % fprintf('  %.5f  ', qPosAngles);  % Display all joint angles in a row (degrees)
            % fprintf(']\n');
            % fprintf('\n');
            % 
            % % Animate from current position to qPos (initial joint configuration)
            % currentJointConfig = robot.model.getpos();  % Get current joint configuration
            % numSteps = 50;  % Number of steps for smooth animation to qPos
            % qMatrix = jtraj(currentJointConfig, qPos, numSteps);  % Trajectory from current position to qPos
            % 
            % % Animate the robot moving to qPos
            % for i = 1:numSteps
            %     % Check for Plane and Ellipsoid collisions
            %     if i > 1  % Skip for the first step since we have no previous position
            %         previousEndEffectorPos = transl(robot.model.fkine(qMatrix(i - 1, :)));
            %         currentEndEffectorPos = transl(robot.model.fkine(qMatrix(i, :)));
            % 
            %         % Plane collision check
            %         if self.CheckPlaneCollision(previousEndEffectorPos, currentEndEffectorPos)
            %             warning('Collision detected with the table! Stopping movement.');
            %             self.returnToHome(robot, currentJointConfig, homePosition); % Return to home position
            %             return;
            %         end
            %     end
            % 
            %     % Ellipsoid collision check
            %     if self.CheckEllipsoidCollision(robot.model.fkine(qMatrix(i, :)).T)
            %         warning('Collision detected with environment object! Stopping movement.');
            %         self.returnToHome(robot, currentJointConfig, homePosition); % Return to home position
            %         return;
            %     end
            % 
            %     robot.model.animate(qMatrix(i, :));
            %     drawnow();
            %     pause(0.02);  % Adjust pause for speed of animation
            % end
            % 
            % fprintf('Stage 1 complete, continuing with orientation...\n\n');
            % 
            % %% Stage 2: RMRC for fine orientation and position adjustment
            % % RMRC parameters
            % deltaT = 0.02;  % Control frequency (time step for RMRC)
            % RMRCsteps = 100;  % Number of increments
            % epsilon = 0.1;  % Threshold for manipulability/Damped Least Squares
            % W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix for the velocity vector
            % 
            % % Get the initial configuration and current transform from Stage 1
            % currentConfig = qPos;  % Start from qPos from Stage 1
            % currentTransform = robot.model.fkine(currentConfig);
            % 
            % % Extract initial and target translation and rotation
            % initialPos = transl(currentTransform).';  % 3x1 vector
            % targetPos = transl(targetTransform);      % 3x1 vector
            % initialRot = t2r(currentTransform);       % 3x3 matrix
            % targetRot = t2r(targetTransform);         % 3x3 matrix
            % 
            % % Linear interpolation for position increments
            % deltaPos = (targetPos - initialPos) / RMRCsteps;
            % 
            % % Compute the incremental rotation matrix for each step
            % deltaRot = (targetRot * initialRot')^(1 / RMRCsteps);  % Rotation increment
            % 
            % % Initialize the current rotation matrix
            % currentRot = initialRot;
            % 
            % % RMRC loop over the given steps
            % for i = 1:RMRCsteps
            %     % Update the position incrementally
            %     currentPos = initialPos + i * deltaPos;  % 3x1 vector
            % 
            %     % Update the rotation incrementally using matrix multiplication
            %     currentRot = deltaRot * currentRot;  % Maintain orthogonality
            % 
            %     % Construct the intermediate target transform
            %     intermediateTransform = rt2tr(currentRot, currentPos);
            % 
            %     % Check for self-collision during RMRC
            %     if self.selfCollisionCheck(robot, currentConfig)
            %         warning('Self-collision detected during RMRC! Stopping movement.');
            %         return;
            %     end
            % 
            %     % Ellipsoid Collision with objects
            %     currentPose = qMatrix(i, :);
            %     if self.CheckEllipsoidCollision(robot.model.fkine(currentPose).T)
            %         warning('Collision detected with environment object. Stopping movement.');
            %         return;
            %     end
            % 
            %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %     % Made RMRC failed because took too long
            %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %     % Check for Table collision during RMRC
            %     % if i > 1  % Skip for the first step since we have no previous position
            %     %     previousEndEffectorPos = transl(robot.model.fkine(qMatrix(i - 1, :)));
            %     %     currentEndEffectorPos = transl(robot.model.fkine(qMatrix(i, :)));
            %     % 
            %     %     % Perform collision check
            %     %     isCollision = self.CheckPlaneCollision(previousEndEffectorPos, currentEndEffectorPos);
            %     %     if isCollision
            %     %         warning('Collision detected with the table! Stopping movement.');
            %     %         return;
            %     %     end
            %     % end
            %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % 
            %     % Check if the matrix becomes singular
            %     if abs(det(intermediateTransform)) < 1e-10
            %         warning('Matrix is singular or near-singular at step %d. Skipping this step.', i);
            %         continue;
            %     end
            % 
            %     % Compute the position error and orientation error
            %     deltaX = targetPos - transl(currentTransform).';
            %     Rd = targetRot;
            %     Ra = t2r(currentTransform);
            %     Rdot = (1 / deltaT) * (Rd - Ra);
            %     S = Rdot * Ra';
            % 
            %     % Compute linear and angular velocities
            %     linear_velocity = (1 / deltaT) * deltaX;
            %     angular_velocity = [S(3, 2); S(1, 3); S(2, 1)];
            % 
            %     % Compute end-effector velocity
            %     xdot = W * [linear_velocity; angular_velocity];
            % 
            %     % Compute Jacobian and manipulability
            %     J = robot.model.jacob0(currentConfig);
            %     m = sqrt(det(J * J'));
            %     if m < epsilon
            %         lambda = (1 - m / epsilon) * 5E-2;
            %     else
            %         lambda = 0;
            %     end
            %     invJ = inv(J' * J + lambda * eye(6)) * J';
            % 
            %     % Solve for the next joint velocities
            %     qdot = (invJ * xdot).';
            % 
            %     % Update joint configuration
            %     currentConfig = currentConfig + deltaT * qdot;
            % 
            %     % Animate the robot to the new configuration
            %     robot.model.animate(currentConfig);
            %     drawnow();
            %     pause(deltaT);
            % 
            %     % Update the current transform for the next iteration
            %     currentTransform = robot.model.fkine(currentConfig);
            % end
            % 
            % % Display the final q values after RMRC
            % finalQ = currentConfig;
            % finalQAngles = rad2deg(finalQ);
            % 
            % fprintf('Final q values (after RMRC) = \n');
            % fprintf('\n [');
            % 
            % fprintf('  %.5f  ', finalQ);  % Display all joint angles in a row
            % fprintf(']\n');
            % fprintf('\n [');
            % 
            % fprintf('  %.5f  ', finalQAngles);  % Display all joint angles in a row (degrees)
            % fprintf(']\n');
            % fprintf('\n');
            % 
            % % Extract the final roll, pitch, and yaw from the final transform
            % finalRPY = tr2rpy(currentTransform);
            % finalRPYDegrees = rad2deg(finalRPY);
            % 
            % fprintf('Final Roll, Pitch, Yaw (degrees) = \n');
            % fprintf('Roll: %.5f  Pitch: %.5f  Yaw: %.5f\n\n', finalRPYDegrees(1), finalRPYDegrees(2), finalRPYDegrees(3));
            % 
            % % pause(1);
            % 
            % % Display movement complete
            % fprintf("*********************************MOVEMENT COMPLETE*********************************\n\n");
            % 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

        %% Self Collision Checking 
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
        
        
        %% Check Plane Collision
        function isCollision = CheckPlaneCollision(self, point1, point2)
            % This method checks for a collision between the line segment
            % from point1 to point2 and the plane defined in the table class.
            [isCollision, ~] = self.table.CheckCollisionWithPlane(point1, point2);
        end

        %% Ellipsoid Collision Check Function
        function isCollision = CheckEllipsoidCollision(self, endEffectorTr)
            isCollision = false;
            endEffectorPos = endEffectorTr(1:3, 4).'; % Extract end-effector position

            % Iterate over all objects and check for collisions
            for objIndex = 1:length(self.objects)
                currentObj = self.objects{objIndex};
                dist = self.GetAlgebraicDist(endEffectorPos, currentObj.Center, currentObj.Radii);

                % Check if within ellipsoid (algebraic distance < 1)
                if dist < 1
                    isCollision = true;
                    return;
                end
            end
        end

        %% Get Algebraic Distance for Ellipsoids
        function algebraicDist = GetAlgebraicDist(self, point, center, radii)
            algebraicDist = ((point(1) - center(1)) / radii(1))^2 ...
                          + ((point(2) - center(2)) / radii(2))^2 ...
                          + ((point(3) - center(3)) / radii(3))^2;
        end

        %% Helper function to return to home position
        function returnToHome(self, robot, currentConfig, homePosition)
            % Define a smooth trajectory back to home position
            returnTrajectory = jtraj(currentConfig, homePosition, self.steps);
        
            % Animate returning to home
            for j = 1:self.steps
                robot.model.animate(returnTrajectory);
                drawnow();
                pause(0.02);
            end
            fprintf("Robot returned to home position due to collision.\n");
        end
    end
end