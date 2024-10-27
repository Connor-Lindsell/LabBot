classdef Enviroment < handle
   
    properties (Access = public)
        Conical;
        Beaker;
        PlyTable;
        Cube;
    end
    
    methods
        function self = Enviroment(self)
            axis([-5 5 -5 5 0 5]);  % Set the axis limits to fit all objects in the environment
            xlabel('X-axis');  % Label the X-axis
            ylabel('Y-axis');  % Label the Y-axis
            zlabel('Z-axis');  % Label the Z-axis
            grid on;  % Display a grid for better visualization of object positions
            hold on;  % Keep the plot active for additional elements
            
            
    % self.PlyTable = Table;
    % self.Beaker = Beaker;
    % self.Conical = Conical;
    self.Cube = Cube;

    % BeakerbaseTr = transl(0,0,1);
    % ConicalbaseTr = transl(2,2,0);

    % self.Beaker.model.base = self.Beaker.model.base.T * BeakerbaseTr;

    
        end
    end
end

