classdef CustomBot < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'CustomBot';
    end
    
    methods
%% Constructor
        function self = CustomBot(baseTr)
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
            % 
            % self.CreateModel();
			% self.model.base = self.model.base.T * baseTr;
            % self.model.tool = self.toolTr;
            % self.PlotAndColourRobot();


                        
            if nargin < 1
                    baseTr = eye(4);  % Origin Point
            end             
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);  

            self.PlotAndColourRobot();

            % 
            % workspace = [-2 2 -2 2 -0 2];                                       % Set the size of the workspace when drawing the robot        
            % scale = 0.5;        
            % q = zeros(1,5);                                                     % Create a vector of initial joint angles        
            % self.model.plot(q,'workspace',workspace,'scale',scale);
            % self.model.teach();


            % drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            link(2) = Link('d',0.15,'a',0.1,'alpha',pi);
            link(3) = Link('d',-0.1,'a',0,'alpha',-pi/2);
            link(4) = Link('d',0,'a',0.4,'alpha',pi);
            link(5) = Link('d',0,'a',0.5,'alpha',pi/2);
            % link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2);
            % link(7) = Link('d',	0.0921,'a',0,'alpha',0);

         

            % Joint limits

            link(1).qlim = [0.01 0.8];
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-360 360]*pi/180;
            link(4).qlim = [-360 360]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            % link(6).qlim = [-360 360]*pi/180;
            % Link(7).qlim = [-Inf, Inf] %(continous rotation)

            % Link offsets
            link(2).offset = pi/2;
            link(3).offset = pi;
            link(4).offset = pi/2;
            link(5).offset = pi/2;
            % 
            self.model = SerialLink(link,'name', self.name);
        end    
        end      
    end

