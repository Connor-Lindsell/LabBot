%% Test the Virtual Teach Pendant functionality
% run this script to test the VTP MATLAB app
%
% To quit, press ctrl-C
%
%% setup virtual teach pentant
pendant = TestTeach;

%%
while(1)
    
    % Read VTP values
    wrench = pendant.read;
    wrench
    
    % Print buttons/axes info to command window
    str = sprintf('--------------\n');
    str = [str sprintf('Joint 1:%01.3f\n',wrench(1))];
    str = [str sprintf('Joint 2:%01.3f\n',wrench(2))];
    str = [str sprintf('Joint 3:%01.3f\n',wrench(3))];
    str = [str sprintf('Joint 4:%01.3f\n',wrench(4))];
    str = [str sprintf('Joint 5:%01.3f\n',wrench(5))];
    str = [str sprintf('Joint 6:%01.3f\n',wrench(6))];
    str = [str sprintf('--------------\n')];
    fprintf('%s',str);
    pause(0.05);  
    
end
    
    
    
    