classdef SuctionGripper < RobotBaseClass
    
    properties(Access = public)   
        plyFileNameStem = 'SuctionGripper';
        gripperState = 'Closed';  % Keep track of gripper's current state
        % base = eye(4);
    end
    
    methods       
        function self = SuctionGripper()
            % if nargin < 3
            %     if nargin == 2
            %         error('If you set useTool you must pass in the toolFilename as well');
            %     elseif nargin == 0 % Nothing passed
            %         baseTr = transl(0,0,0);  
            %     end             
            % else % All passed in 
            %     self.useTool = useTool;
            %     toolTrData = load([toolFilename,'.mat']);
            %     self.toolTr = toolTrData.tool;
            %     self.toolFilename = [toolFilename,'.ply'];
            % end


             if nargin < 1
                    baseTr = eye(4);  % Origin Point
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
            % L2 = Revolute('d', 0.1, 'a', 0, 'alpha', pi/2, 'standard');
            self.model = SerialLink([L1], 'name', 'SuctionGripper');
        end
    end
end
