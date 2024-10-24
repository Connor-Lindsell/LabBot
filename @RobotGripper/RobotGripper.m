classdef RobotGripper < RobotBaseClass
    
    properties(Access = public)   
        % plyFileNameStem = 'RobotGripper';
        gripperState = 'Closed';  % Keep track of gripper's current state
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
        
        % Implement the abstract CreateModel function from RobotBaseClass
        function CreateModel(self)
            % Define the robot model here
            L1 = Revolute('d', 0.1, 'a', 0, 'alpha', pi/2, 'standard');
            L2 = Revolute('d', 0, 'a', 0.2, 'alpha', 0, 'standard');
            L3 = Revolute('d', 0, 'a', 0.2, 'alpha', 0, 'standard');
            self.model = SerialLink([L1, L2, L3], 'name', 'GripperRobot');
        end
        
        function [finger1, finger2] = CreateGripper(self)
            % Create the gripper fingers
            disp('Creating gripper...');
            finger_l1 = 0.1;
            finger_l2 = 0.1;
            
            T_offset1 = transl(0, -0.1, 0);  % Offset for finger 1 base
            finger1 = SerialLink([ ...
                Revolute('d', 0, 'a', finger_l1, 'alpha', 0, 'standard'), ...
                Revolute('d', 0, 'a', finger_l2, 'alpha', 0, 'standard')], ...
                'name', 'finger1Gripper', 'base', T_offset1);
            
            T_offset2 = transl(0, 0.1, 0);  % Offset for finger 2 base
            finger2 = SerialLink([ ...
                Revolute('d', 0, 'a', finger_l1, 'alpha', 0, 'standard'), ...
                Revolute('d', 0, 'a', finger_l2, 'alpha', 0, 'standard')], ...
                'name', 'finger2Gripper', 'base', T_offset2);
            
            disp('Gripper fingers initialized successfully.');
        end        
        
        function [qGripper1_target, qGripper2_target] = controlGripper(self, action)
            % Control the gripper to open or close
            disp(['Controlling gripper: ', action]);
            switch action
                case 'Open'
                    qGripper1_target = [-pi/4, pi/4];  % Open position for finger 1
                    qGripper2_target = [pi/4, -pi/4];  % Open position for finger 2
                case 'Close'
                    qGripper1_target = [0, 0];  % Closed position for finger 1
                    qGripper2_target = [0, 0];  % Closed position for finger 2
                otherwise
                    error('Invalid gripper action');
            end
            disp('Gripper state set.');
        end

        % Function to animate the gripper opening or closing
        function animateGripper(self, finger1, finger2, action)
            steps = 50;
            qOpen1 = [-pi/4, pi/4];  % Open position for finger 1
            qOpen2 = [pi/4, -pi/4];  % Open position for finger 2
            qClose1 = [0, 0];        % Closed position for finger 1
            qClose2 = [0, 0];        % Closed position for finger 2
            
            if strcmp(action, 'Open')
                qTraj1 = jtraj(qClose1, qOpen1, steps);
                qTraj2 = jtraj(qClose2, qOpen2, steps);
            else
                qTraj1 = jtraj(qOpen1, qClose1, steps);
                qTraj2 = jtraj(qOpen2, qClose2, steps);
            end
            
            for i = 1:steps
                finger1.animate(qTraj1(i, :));
                finger2.animate(qTraj2(i, :));
                pause(0.01);
            end
        end

        % Toggle the gripper state and animate
        function toggleGripperState(self, finger1, finger2)
            if strcmp(self.gripperState, 'Closed')
                disp('Gripper is closed. Opening...');
                self.animateGripper(finger1, finger2, 'Open');
                self.gripperState = 'Open';  % Update state
            else
                disp('Gripper is open. Closing...');
                self.animateGripper(finger1, finger2, 'Close');
                self.gripperState = 'Closed';  % Update state
            end
        end        
    end
end
