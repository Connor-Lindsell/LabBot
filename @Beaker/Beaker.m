% % Method A 
% classdef Beaker < RobotBaseClass
%         properties (Access = public)
%             plyFileNameStem = 'Bbeaker';
%         end
%         methods
%             function self = Beaker(baseTr)
%                 if nargin < 1
%                     baseTr = transl(0,0,0);
%                 end
%                 self.CreateModel()
% 
%                 self.model.base = self.model.base.T * baseTr ;
%                 self.PlotAndColourRobot();
% 
%             end
% 
%             function CreateModel(self)
%                 L(1) = Link('d', 0.01,'a',0,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
%                 self.model = SerialLink(L,'name',self.name);
%             end
% 
%         end
% end
%% Method B

classdef Beaker < handle
    %#ok<*TRYNC>

    properties (Access = public)
        plyFileNameStem = 'BBeaker.ply';
        BeakerModel;
        baseTr = eye(4);
        %basePose = eye(4);
        workspaceDimensions = [-1,1,-1,1,0,1];
    end

    methods
        function self = Beaker(x,y,z)
            % PlaceObject('Conical.ply');
            %  axis equal
            %  camlight
            self.BeakerModel{1} = self.GetBeakerModel(['Beaker',num2str(1)]);

                % basePose = SE3([1 , 0 , 0 , 0 ;...
                %                 0 , 1 , 0 , 0 ;...
                %                 0 , 0 , 1 , 0 ;...
                %                 0 , 0 , 0 , 1]);


                basePose = eye(4);

                % baseTr = makehgtform('translate',[x, y, z]);
                % baseTr = eye(4) * transl(x,y,z);

                self.BeakerModel{1}.base = basePose * transl(x,y,z);

                 % Plot 3D model

                 % drawnow();
                 % camlight

                 plot3d(self.BeakerModel{1},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist', 'notiles');
                 % Hold on after the first plot (if already on there's no difference)

                    hold on;


            end
    end

    methods (Static)
%% Create model
function model = GetBeakerModel(name)
    if nargin < 1
                name = 'Beaker';
    end
            [faceData,vertexData] = plyread('Bbeaker.ply','tri');
            link1 = Link('alpha', 0,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name',name);

            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};   

            % vertexColours = [0.5, 0.2, 0.1] / 255;
            % 
            % trisurf(faceData,vertexData(:,1),vertexData(:,2), vertexData(:,3) ...
            % ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');






end
    end

end

%% Method C
% 
% classdef Beaker < handle
%     %#ok<*TRYNC>
%     properties
% 
%     end
%     methods
%     function self = Beaker()
%         camlight;
% axis equal;
% view(3);
% hold on;
%     [faceData,vertexData, data] = plyread('Bbeaker.ply','tri');
% 
%     vertexCount =  size(vertexData, 1);
% 
%     % midPoint = sum(vertexData)/vertexCount;
%     beakerVerts = vertexData; %- repmat(midPoint, vertexCount, 1);
%     beakerPose =  eye(4);
% 
%     vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% 
%     beakerMesh_h = trisurf(faceData,beakerVerts(:,1),beakerVerts(:,2), beakerVerts(:,3) ...
%     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
% 
%     end
% end
% end