clc
clear all
close all
startup_rvc
homeQ = [];
 for linkIndex = 0:self.model.n
    if self.useTool && linkIndex == self.model.n
        if ~isempty(self.toolFilename)
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.toolFilename],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'Link',num2str(linkIndex),'Tool.ply'],'tri'); %#ok<AGROW>
        end
    else
        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    end

    % Obtain faceData and vertexData for the current link and save to the cell.
    self.model.faces{linkIndex+1} = faceData;
    self.model.points{linkIndex+1} = vertexData;
end
robot=LinearUR10
figure_h = gcf;
axis_h = gca;
[ax,by] = view;
roughMinMax = sum(abs(robot.model.d) + abs(robot.model.a));
workspace = [-roughMinMax roughMinMax -roughMinMax roughMinMax -0.01 roughMinMax]; 

robot.model.plot3d(homeQ,'noarrow','workspace',workspace,'view',[ax,by]);%,'notiles');            

% Check if a single surface has been added by plot3d
if CountTiledFloorSurfaces() - initialSurfaceCount == 1
    surfaceAdded = true;
end

% Check if a light needs to be added
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
    lightAdded = true;
end

model.delay = 0;
handles = findobj('Tag', model.name);
h = get(handles,'UserData');
%
