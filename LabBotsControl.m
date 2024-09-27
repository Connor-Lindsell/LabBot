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
            self.SimultaneousControl

        end
    end 

    %% Functions
    methods
        function SimultaneousControl(self)
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
                
                % Move to the test tube location to pick up the chemical
                startPos = self.getEndEffectorPos(self.rUR3); 
                finishPos = testTubeLocation{locationIndex};  % Test tube location
                
                % Move UR3 to the test tube
                self.Move2Global(startPos, finishPos, self.rUR3);
                
                % self.GripperClose();
                fprintf('Gripper closing to pick up %s...\n', chemical);
                
                % Move to the mixing location
                startPos = finishPos;
                finishPos = testTubeLocation{mixingLocation};  % Mixing location 
                fprintf('Moving %s to the mixing location at test tube %d...\n', chemical, mixingLocation);
                
                % Move robot to the mixing location with the chemical
                self.Move2Global(startPos, finishPos, self.rUR3);
                
                %% Mix Chem
                % self.PourChem(); 
                fprintf('Pouring %s into test tube at the mixing location...\n', chemical);
                
                %% Return Chem
                % Move back to the original test tube location to return the tube
                startPos = finishPos;
                finishPos = testTubeLocation{locationIndex};  % Back to the original location
                fprintf('Returning test tube %d to its original position...\n', locationIndex);
                
                % Move robot back to return the test tube
                self.Move2Global(startPos, finishPos, self.rUR3);
                
                % self.GripperOpen(); 
                fprintf('Gripper opening to release test tube %d...\n', locationIndex);
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
            % Define transforms with a downward orientation for the last joint (pointing down)
            transforms = {transl(startTr), transl(finishTr)};
            
            % Pre-allocate cell array for joint configurations
            q = cell(1, length(transforms));
            
            % Solve inverse kinematics for each transformation
            for i = 1:length(transforms)
                % Use a mask that includes Z position and orientation (roll around X-axis)
                q{i} = robot.model.ikine(transforms{i}, 'mask', [1, 1, 1, 0, 0, 0]);
                
                % Display the full joint angles using fprintf
                fprintf('q%d = [', i);
                fprintf(' %.5f', q{i});  % Display all joint angles in a row
                fprintf(' ]\n');
            end
        
            % Get the current end-effector position of the robot
            currentEndEffectorPos = self.getEndEffectorPos(robot);
            
            % Define the start position (Cartesian coordinates) from q{1}
            startPos = transl(startTr);  
            
            % Check if the current end-effector position is equal to the start position
            if isequal(currentEndEffectorPos, startPos)
                fprintf('Robot is at the start position.\n');
            else
                fprintf('Moving robot to the start position...\n');
            
                % If the robot is not at the start position, move it there
                qMatrix = jtraj(robot.model.getpos(), q{1}, self.steps);  % Joint trajectory from current position to start position
                
                % Animate the movement to the start position
                for i = 1:size(qMatrix, 1)
                    robot.model.animate(qMatrix(i, :));
                    drawnow();
                end
            end
                        
            % Pre-allocate matrix for combined joint trajectory
            qMatrixTotal = [];
            
            % Generate joint space trajectories for each consecutive pair of transformations
            for i = 1:(length(q) - 1)
                qMatrix = jtraj(q{i}, q{i + 1}, self.steps);
                qMatrixTotal = [qMatrixTotal; qMatrix];  
            end
        
            % Animate the robot through the combined trajectory
            for i = 1:size(qMatrixTotal, 1)
                robot.model.animate(qMatrixTotal(i, :));
                drawnow();
            end
            hold on
        
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


        %% Grab Chemical Function 
        % Toggled by GUI Button

        % function grabChemical = ChooseChemical(TestTube,Chemical)
        %     % get pos of chemical 
        % 
        %     % move to chemical 
        % 
        %     % grab chemical 
        % 
        %     % move to densition (test tub holder) 
        % 
        %     % place chemical
        % 
        % end 

        %% Move To End Effector Position 
        % Toggled by GUI Button 

        % function move2EndEffector = ManualMovement_Endeffector (Startpos, Endpos)
        %     % get pos of robotic arm
        % 
        %     % move to pos
        % 
        % end 

        
       
    end

end