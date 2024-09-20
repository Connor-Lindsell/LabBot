classdef RobotGripper < RobotBaseClass
    
    properties(Access = public)   
        plyFileNameStem = 'RobotGripper';
        % gripperState = 'Closed';  % Keep track of gripper's current state
    end
    
    methods       
        function self = RobotGripper()
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end
        
        function [finger1, finger2] = CreateGripper(self)
            % Create the gripper fingers
            
            % Display message indicating the gripper creation process
            disp('Creating gripper...');
            
            % Define lengths for the gripper fingers
            finger_l1 = 0.1;
            finger_l2 = 0.1;
            
            % Define and create finger 1
            T_offset1 = transl(0, -0.1, 0);  % Offset for finger 1 base
            finger1 = SerialLink([ ...
                Revolute('d', 0, 'a', finger_l1, 'alpha', 0, 'standard'), ...
                Revolute('d', 0, 'a', finger_l2, 'alpha', 0, 'standard')], ...
                'name', 'finger1Gripper', 'base', T_offset1);
            
            % Define and create finger 2
            T_offset2 = transl(0, 0.1, 0);  % Offset for finger 2 base
            finger2 = SerialLink([ ...
                Revolute('d', 0, 'a', finger_l1, 'alpha', 0, 'standard'), ...
                Revolute('d', 0, 'a', finger_l2, 'alpha', 0, 'standard')], ...
                'name', 'finger2Gripper', 'base', T_offset2);
            
            % Display message indicating successful gripper initialization
            disp('Gripper fingers initialized successfully.');
        end        
        
        function [qGripper1_target, qGripper2_target] = controlGripper(self, action)
            % Control the gripper to open or close
            
            % Display the action being performed
            disp(['Controlling gripper: ', action]);
            
            % Determine target joint angles based on the specified action
            switch action
                case 'Open'
                    qGripper1_target = [-pi/4, pi/4];  % Open position for finger 1
                    qGripper2_target = [pi/4, -pi/4];  % Open position for finger 2
                case 'Close'
                    qGripper1_target = [0, 0];  % Closed position for finger 1
                    qGripper2_target = [0, 0];  % Closed position for finger 2
                otherwise
                    % Display error if action is invalid
                    error('Invalid gripper action');
            end
            
            % Display message indicating the gripper state has been set
            disp('Gripper state set.');
        end

        % Function to animate the gripper opening or closing
        function animateGripper(self, finger1, finger2, action)
            % Define number of steps for smooth animation
            steps = 50;
            
            % Define open and close joint targets
            qOpen1 = [-pi/4, pi/4];  % Open position for finger 1
            qOpen2 = [pi/4, -pi/4];  % Open position for finger 2
            qClose1 = [0, 0];        % Closed position for finger 1
            qClose2 = [0, 0];        % Closed position for finger 2
            
            % Generate joint trajectories for opening and closing
            if strcmp(action, 'Open')
                qTraj1 = jtraj(qClose1, qOpen1, steps);
                qTraj2 = jtraj(qClose2, qOpen2, steps);
            else
                qTraj1 = jtraj(qOpen1, qClose1, steps);
                qTraj2 = jtraj(qOpen2, qClose2, steps);
            end
            
            % Animate the gripper based on the calculated trajectory
            for i = 1:steps
                finger1.animate(qTraj1(i, :));
                finger2.animate(qTraj2(i, :));
                pause(0.01);
            end
        end

        % Toggle the gripper state and animate
        function toggleGripperState(self, finger1, finger2)
            % Check the current state of the gripper and toggle
            if strcmp(self.gripperState, 'Closed')
                % If closed, open the gripper
                disp('Gripper is closed. Opening...');
                self.animateGripper(finger1, finger2, 'Open');
                self.gripperState = 'Open';  % Update state
            else
                % If open, close the gripper
                disp('Gripper is open. Closing...');
                self.animateGripper(finger1, finger2, 'Close');
                self.gripperState = 'Closed';  % Update state
            end
        end

        % % TestGripper function to test gripper opening and closing
        % function testGripper(self)
        %     % Set the base transformation for testing
        %     baseTr = transl(0, 0, 0) * trotx(-pi/2);  % Translate to (0, 0, 0) and rotate by -90 degrees on x-axis
        %     self.model.base = baseTr;
        % 
        %     % Create the gripper
        %     [finger1, finger2] = self.CreateGripper();
        % 
        %     % Call the toggle function to open or close the gripper
        %     self.toggleGripperState(finger1, finger2);
        % 
        %     % Display message indicating the test is done
        %     disp('Gripper test completed.');
        % end
    end
end
