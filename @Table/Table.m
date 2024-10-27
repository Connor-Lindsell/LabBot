classdef Table < handle
    %#ok<*TRYNC>
   
    properties (Access = public)
        plyFileNameStem = 'BTable.ply';
        TableModel;
        %baseTr = eye(4);
        %basePose = eye(4);
        workspaceDimensions = [-10,10,-10,10,0,10];
    end
    
    methods
        function self = Table 
            PlaceObject('BTable.ply');
            axis equal
            camlight
            self.TableModel{1} = self.GetTableModel(['Table',num2str(1)]);

                % basePose = SE3([1 , 0 , 0 , 0 ;...
                %                 0 , 1 , 0 , 0 ;...
                %                 0 , 0 , 1 , 0 ;...
                %                 0 , 0 , 0 , 1]);


                basePose = eye(4);

                % baseTr = makehgtform('translate',[x, y, z]);
                % baseTr = eye(4) * transl(x,y,z);

                %self.TableModel{1}.base = basePose * transl(x,y,z);

                 % Plot 3D model

                 % drawnow();
                 %camlight

                 plot3d(self.TableModel{1},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist', 'notiles');
                 % Hold on after the first plot (if already on there's no difference)

                    hold on;
                
            
            end
    end

    methods (Static)
%% Create model
function model = GetTableModel(name)
    if nargin < 1
                name = 'BTable';
    end
            [faceData,vertexData,data] = plyread('BTable.ply','tri');
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

            % vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % 
            % tableMesh_h = trisurf(faceData,vertexData(:,1),vertexData(:,2), vertexData(:,3) ...
            % ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
end
    end

end


