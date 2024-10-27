% function testerImprt()
% 
% This function use "plyread" function which is available here:
% http://au.mathworks.com/matlabcentral/fileexchange/5355-toolbox-graph/content/toolbox_graph/read_ply.m
% clf
% doCameraSpin = false;
% Although generally I usually try to use more descriptive variables, to make it easier to view here I will use 
% f = faceData;
% v = vertexData;
% % Load the cube created and painted in Blender
% 
% [f,v,data] = plyread('cube.ply','tri');
% 
% 
% 
% Scale the colours to be 0-to-1 (they are originally 0-to-255
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% vertexColours = [0,255,255]/255;
% 
% 
% hold on;
% Plot the cubes at the corners of a 10m square room
% for xOffset = [-5, 5]
%     for yOffset = [-5, 5]
%         Then plot the trisurf with offset verticies
%         trisurf(f,v(:,1)+ xOffset,v(:,2) + yOffset, v(:,3) ...
%             ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%     end
% end
% Turn on a light (only turn on 1, don't keep turning them on), and make axis equal
% camlight;
% axis equal;
% view(3);
% hold on;
classdef Cube < RobotBaseClass
        properties
            plyFileNameStem = 'Cube';
        end
        methods
            function self = Cube(baseTr)
                if nargin < 1
                    baseTr = transl(0,0,0);
                end
                self.CreateModel()
                
                self.model.base = self.model.base.T * baseTr ;
                % axis equal;
                self.PlotAndColourRobot(); 
                % drawnow;
                % hold on
                % 
            end

            function CreateModel(self)
                L(1) = Link('d', 0.1,'a',0,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
                self.model = SerialLink(L,'name',self.name);
            end
            
        end
end