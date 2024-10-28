classdef LabBotCalculations 
    %% Properties
    properties 
        environment

        % Initialise Robot models variables
        rUR3
        rLabBot
    end

    methods 
        function obj = LabBotCalculations
            obj.environment = LabBotEnvironment();
        end
    end

    methods
        %% WorkspaceCalc: Computes and visualizes the robots' reachable workspaces
        % This function calculates the reachable points for both robots within their joint limits,
        % visualizes their maximum reach in 3D space, and computes the volume of the
        % convex hull of the reachable workspace.
        
        

        %% Function to calculate and visualize workspace for a given robot
        function calculateRobotWorkspace(self, robot, robotName)
            % Define the step size in degrees and convert it to radians
            stepDeg = 5;  % Step size in degrees
            stepRads = deg2rad(stepDeg);  % Convert step size to radians
            
            % fprintf ('Robot: %d\n', robot)

            % Retrieve the joint limits of the robot
            qlim = robot.model.qlim;  % Joint limit matrix
            
            % Initialize an array to store the maximum reachable points
            maxReach = [];  
            
            % Estimate the total number of points in the point cloud based on joint limits and step size
            pointCloudSize = prod(floor((qlim(1:3,2) - qlim(1:3,1)) / stepRads + 1));  
            counter = 0;  % Initialize a counter for progress tracking
            tic;  % Start a timer to measure computation time
            disp(['Starting computation of maximum reach for ', robotName, '...']);
            
            %% Loop through joint limits to compute reachable points
            % Loop through joint 1 (base rotation) limits
            for q1 = qlim(1,1):stepRads:qlim(1,2)
                % Loop through joint 2 (shoulder) limits
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    % Loop through joint 3 (elbow) limits
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        % Set joint angles for q1, q2, q3; other joints fixed at 0
                        q = [q1, q2, q3, 0, 0, 0];  
                        
                        % Perform forward kinematics to get the transformation matrix (position)
                        tr = robot.model.fkine(q).T;  % Compute the forward kinematics
                        
                        % Only keep points where the Z-coordinate (height) is non-negative
                        if tr(3,4) >= 0  
                            % Append the X, Y, Z position of the end effector to maxReach
                            maxReach = [maxReach; tr(1:3,4)'];
                        end
        
                        % Update progress counter and calculate percentage completed
                        counter = counter + 1;
                        percentage = (counter / pointCloudSize) * 100;
                        
                        % Display progress every 25% completion
                        if mod(percentage, 25) == 0
                            elapsedTime = toc;  % Get elapsed time
                            disp(['Completed ', num2str(percentage), '% of ', robotName, ' in ', num2str(elapsedTime), ' seconds.']);
                        end
                    end
                end
            end
            
            %% Plot the maximum reachable points for the robot
            figure;
            plot3(maxReach(:,1), maxReach(:,2), maxReach(:,3), 'r.');  % 3D scatter plot of reachable points
            xlabel('X');  % Label for X-axis
            ylabel('Y');  % Label for Y-axis
            zlabel('Z');  % Label for Z-axis
            title(['Maximum Reach of the ', robotName, ' (Z >= 0)']);  % Plot title
            grid on;  % Enable grid for better visualization
            axis equal;  % Ensure equal scaling on all axes
            
            %% Calculate and display the maximum reach distance
            if ~isempty(maxReach)
                % Compute the Euclidean distance of each reachable point from the origin
                distances = sqrt(sum(maxReach.^2, 2));  
                maxDistance = max(distances);  % Find the maximum reach distance
                disp(['The maximum reach of ', robotName, ' is approximately ', num2str(maxDistance), ' meters.']);
            else
                disp(['No valid points found for ', robotName, ' (Z >= 0).']);  % If no points are found
            end
            
            %% Calculate and visualize the convex hull of the reachable workspace
            if ~isempty(maxReach)
                % Compute the convex hull of the reachable points (3D boundary)
                [K, volume] = convhull(maxReach(:,1), maxReach(:,2), maxReach(:,3));  
                disp(['The approximate volume of ', robotName, ' workspace is ', num2str(volume), ' cubic meters.']);
                
                % Plot the convex hull of the reachable workspace
                figure;
                trisurf(K, maxReach(:,1), maxReach(:,2), maxReach(:,3), 'FaceColor', 'cyan', 'FaceAlpha', 0.5);  % 3D surface plot
                xlabel('X');  % Label for X-axis
                ylabel('Y');  % Label for Y-axis
                zlabel('Z');  % Label for Z-axis
                title(['Convex Hull of ', robotName, ' Workspace (Volume: ', num2str(volume), ' cubic meters)']);  % Plot title
                axis equal;  % Ensure equal scaling on all axes
            else
                disp(['No valid points found for ', robotName, ' (Z >= 0).']);  % If no points are found
            end
        end
    end
end
