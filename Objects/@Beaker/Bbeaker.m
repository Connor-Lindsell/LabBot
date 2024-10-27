classdef Bbeaker < RobotBaseClass
        properties
            plyFileNameStem = 'Bbeaker';
        end
        methods
            function self = Bbeaker(baseTr)
                if nargin < 1
                    baseTr = transl(0,0,0);
                end
                self.CreateModel()
                
                self.model.base = self.model.base.T * baseTr ;
                self.PlotAndColourRobot(); 
                
            end

            function CreateModel(self)
                L(1) = Link('d', 0.1506,'a',0,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)], 'offset',0);
                self.model = SerialLink(L,'name',self.name);
            end
            
        end
end

