classdef Conical < RobotBaseClass
        properties (Access = public)
            plyFileNameStem = 'Conical';
        end
        methods
            function self = Conical(baseTr)
                if nargin < 1
                    baseTr = transl(0,0,0);
                end
                self.CreateModel()

                self.model.base = self.model.base.T * baseTr ;
                
                self.PlotAndColourRobot();

            end

            function CreateModel(self)
                L(1) = Link('d', -10,'a',0,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
                self.model = SerialLink(L,'name',self.name);
            end

        end
end

% 
% classdef Conical < handle
%     %#ok<*TRYNC>
% 
%     properties (Access = public)
%         plyFileNameStem = 'Conical.ply';
%         ConicalModel;
%         %baseTr = eye(4);
%         %basePose = eye(4);
%         workspaceDimensions = [-1,1,-1,1,0,1];
%     end
% 
%     methods
%         function self = Conical(x,y,z)
%             % PlaceObject('Conical.ply');
%             %  axis equal
%             %  camlight
%             self.ConicalModel{1} = self.GetConicalModel(['Conical',num2str(1)]);
% 
%                 % basePose = SE3([1 , 0 , 0 , 0 ;...
%                 %                 0 , 1 , 0 , 0 ;...
%                 %                 0 , 0 , 1 , 0 ;...
%                 %                 0 , 0 , 0 , 1]);
% 
% 
%                 basePose = eye(4);
% 
%                 % baseTr = makehgtform('translate',[x, y, z]);
%                 % baseTr = eye(4) * transl(x,y,z);
% 
%                 self.ConicalModel{1}.base = basePose * transl(x,y,z);
% 
%                  % Plot 3D model
% 
%                  % drawnow();
%                  camlight
% 
%                  plot3d(self.ConicalModel{1},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist', 'notiles');
%                  % Hold on after the first plot (if already on there's no difference)
% 
%                     hold on;
% 
% 
%             end
%     end
% 
%     methods (Static)
% %% Create model
% function model = GetConicalModel(name)
%     if nargin < 1
%                 name = 'Conical';
%     end
%             [faceData,vertexData] = plyread('Conical.ply','tri');
%             link1 = Link('alpha', 0,'a',0,'d',0,'offset',0);
%             model = SerialLink(link1,'name',name);
% 
%             % Changing order of cell array from {faceData, []} to 
%             % {[], faceData} so that data is attributed to Link 1
%             % in plot3d rather than Link 0 (base).
%             model.faces = {[], faceData};
% 
%             % Changing order of cell array from {vertexData, []} to 
%             % {[], vertexData} so that data is attributed to Link 1
%             % in plot3d rather than Link 0 (base).
%             model.points = {[], vertexData};            
% end
%     end
% 
% end