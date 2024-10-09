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
            UR3_Start = [0.2,0.3,0.2];
            UR3_Pos1 = [0.3,-0.2,0.1];
            UR3_End = [-0.2,0.1,0.3];
    
            % LabBot End Effector Goal Destinations 
            % LabBot_Start = [0.2,0.2,0.2];
            % LabBot_Pos1 = [0.2,0.2,0.2];
            % LabBot_End = [0.2,0.2,0.2];


            %% Perform Movements
            % Calling Move2Global using self
            % For UR3
            self.Move2Global(UR3_Start, UR3_Pos1, self.rUR3);
            self.Move2Global(UR3_Pos1, UR3_End, self.rUR3);
            
            % For LabBot
            % self.Move2Global(LabBot_Start, LabBot_Pos1, rLabBot);
            % self.Move2Global(LabBot_Pos1, LabBot_End, rLabBot);             

            %% Mix Chemicals 

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
                startPos = self.getEndEffectorPos(self.rUR3); 
                finishPos = testTubeLocation{locationIndex};  % Test tube location
                
                % Move UR3 to the test tube
                self.Move2Global(startPos, finishPos, self.rUR3);
                
                % self.GripperClose();
                fprintf('Gripper closing to pick up %s...\n', chemical);
                fprintf('\n');
                
                % Move to the mixing location
                startPos = finishPos;
                finishPos = testTubeLocation{mixingLocation};  % Mixing location 
                fprintf('Moving %s to the mixing location at test tube %d...\n', chemical, mixingLocation);
                fprintf('\n');
                
                % Move robot to the mixing location with the chemical
                self.Move2Global(startPos, finishPos, self.rUR3);
                
                %% Mix Chem
                % self.PourChem(); 
                fprintf('Pouring %s into test tube at the mixing location...\n', chemical);
                fprintf('\n');
                
                %% Return Chem
                % Move back to the original test tube location to return the tube
                startPos = finishPos;
                finishPos = testTubeLocation{locationIndex};  % Back to the original location
                fprintf('Returning test tube %d to its original position...\n', locationIndex);
                fprintf('\n');
                
                % Move robot back to return the test tube
                self.Move2Global(startPos, finishPos, self.rUR3);
                
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
        % Start Transform: the start lcation of the robot end effector 
        % Finish Transform: the end location of the robot end effector 
        % Robot: calls the robot that is required to move
        

        function Move2Global(self, startTr, finishTr, robot)
            % Convert the start and finish positions to 4x4 homogeneous transformation matrices
            transforms = {transl(startTr), transl(finishTr)};
            
            % Pre-allocate cell array for joint configurations
            q = cell(1, length(transforms));
        
            % Set desired values for q5 (90 degrees) and q6 (0 degrees)
            q5_desired = deg2rad(90);  % q5 = 90 degrees (pi/2 radians)
            q6_desired = deg2rad(0);   % q6 = 0 degrees
        
            % Loop through the transformations to solve IK for each one
            for i = 1:length(transforms)
                % Set the initial guess (current joint configuration)
                q_init = robot.model.getpos();  % Use the robot's current joint positions
                
                % Ensure the target transform is a 4x4 homogeneous matrix
                targetTransform = transforms{i};
        
                % DEBUGGING: Print the inputs for verification
                fprintf('Initial guess (q_init): %.5f\n', q_init);
                fprintf('Target transform (targetTransform):\n');
                disp(targetTransform);
                
                % Call the simplified iterative IK solver
                q_solution = IterativeIKSolver(robot, targetTransform, q_init);
        
                % DEBUGGING: Check the output
                fprintf('IK Solver output (q_solution):\n');
                disp(q_solution);
        
                % Assign the solution to q{i}
                q{i} = q_solution;
            end
        
            % Additional code to animate and move the robot...
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

        %%
        function q_solution = IterativeIKSolver(robot, targetTransform, q_init)
            max_iters = 100;       % Maximum number of iterations
            tolerance = 1e-4;      % Tolerance for stopping condition
            alpha = 0.01;          % Step size for gradient descent
        
            q_solution = q_init;   % Initialize joint angles to the initial guess
        
            for iter = 1:max_iters
                % Compute current end-effector transform using forward kinematics
                T_current = robot.model.fkine(q_solution);
                
                % Compute pose error between current and target transforms
                pose_error = norm(transl(T_current) - transl(targetTransform));
                
                % If the error is below the tolerance, stop the iterations
                if pose_error < tolerance
                    fprintf('Converged after %d iterations.\n', iter);
                    break;
                end
                
                % Compute the gradient of the error with respect to q (using finite differences)
                dq = GradientOfError(robot, q_solution, targetTransform);
                
                % Update joint angles based on the gradient
                q_solution = q_solution - alpha * dq;
            end
            
            if iter == max_iters
                fprintf('Max iterations reached without full convergence.\n');
            end
        end               
       
    end

end