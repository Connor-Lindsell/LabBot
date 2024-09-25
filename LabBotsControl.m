classdef LabBotsControl
    
    
    properties
        % steps = 100;
        
    end

    methods
        function self = LabBotsControl
            % Constructor method
            self.SimultaneousControl

        end
    end 


    methods
        function SimultaneousControl(self)
            % Initaialising Robot Models
            rUR3 = UR3;
            % rLabBot = LabBot;

            %% Base transforms

            %% Set Up Enviorment 

            % Call Enviorment class

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
            self.Move2Global(UR3_Start, UR3_Pos1, rUR3);
            self.Move2Global(UR3_Pos1, UR3_End, rUR3);
            
            % For LabBot
            % self.Move2Global(LabBot_Start, LabBot_Pos1, rLabBot);
            % self.Move2Global(LabBot_Pos1, LabBot_End, rLabBot);             
            
        end

        %% Move To Global Function
        % Moves the selected Robot Arm From a Start Position to  Finish
        % Position
        % 
        % Inputs - 
        % Start Transform: the start lcation of the robot end effector 
        % Finish Transform: the end location of the robot end effector 
        % Robot: calls the robot that is required to move
        

        function Move2Global(self, start, finish, robot) 

            steps = 100;
            
            % Define transforms with a downward orientation for the last joint (pointing down)
            transforms = {transl(start), transl(finish)};
            
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
        
            
            % Pre-allocate matrix for combined joint trajectory
            qMatrixTotal = [];
            
            % Generate joint space trajectories for each consecutive pair of transformations
            for i = 1:(length(q) - 1)
                qMatrix = jtraj(q{i}, q{i + 1}, steps);
                qMatrixTotal = [qMatrixTotal; qMatrix];  
            end
        
            % Animate the robot through the combined trajectory
            for i = 1:size(qMatrixTotal, 1)
                robot.model.animate(qMatrixTotal(i, :));
                drawnow();
            end
            hold on
        
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