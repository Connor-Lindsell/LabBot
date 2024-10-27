classdef OurBot < RobotC
    %% LinearUR10 UR10 on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'OurBot';
    end
    
    methods
%% Constructor
        function self = OurBot(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            %
            self.PlotAndColourRobot();
%             q=[0 0];
%             self.model.teach(q)
            self.model.teach([0 0 0 0])
        end

%% CreateModel
        function CreateModel(self)   
            link(1) = Link([pi/2     0.1012      0.033        pi/2	0]);
            %link(1) = Link([pi/2     0.      0.0        pi/2	0]);
            link(2) = Link([0     0.0      0.155        0	0]);
            link(3) = Link([0     0.0      0.1348        0	0]);
            link(4) = Link([pi/2     0.019      0.0005        -pi/2	0]);
            % model = SerialLink(link,'name','RRRR');
            % model.teach([0 0 0 0])
            self.model = SerialLink(link,'name',self.name);
        end     
    end
end