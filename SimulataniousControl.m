function SimulataniousControl
    % Initaialising Robot Models
    rUR3 = UR3;
    % rLabBot = LabBot;

    %% Properties

    steps = 100;

    % UR3 End Effector Goal Destinations 
    UR3_Start = [0.2,0.2,0.2];
    UR3_Pos1 = [0.2,0.2,0.2];
    UR3_End = [0.2,0.2,0.2];

    % LabBot End Effector Goal Destinations 
    % LabBot_Start = [0.2,0.2,0.2]
    % LabBot_Pos1 = [0.2,0.2,0.2]
    % LabBot_End = [0.2,0.2,0.2]


    %% Transforms 
    % UR3 Transforms
    UR3Transforms = { transl(UR3_Start), ...
                        transl(UR3_Pos1), ...
                        transl(UR3_End) ...
                        };

    % LabBot Transforms 
    % LabBotTransforms = { transl(UR3_Start), ...
                        % transl(UR3_Pos1), ...
                        % transl(UR3_End) ...
                        % };
                        % 
    %% Calculations For q value conversion
    % Pre-allocate cell array for joint configurations
    qUR3 = cell(1, length(UR3Transforms));
    
    % Loop to convert the transforms to the q values using Inverse Kinematics
    for i = 1:length(UR3Transforms)   
        qUR3{i} = rUR3.model.ikine(UR3Transforms{i}, 'mask', [1, 1, 1, 0, 0, 0]);
        
        % Print q values for Logging
        fprintf('q%d = [', i);
        fprintf(' %.5f', qUR3{i});
        fprintf(' ]\n');
    end

    % Pre-allocate cell array for joint configurations
    % qLabBot = cell(1, length(LabBotTransforms));
    % 
    % % Loop to convert the transforms to the q values using Inverse Kinematics
    % for i = 1:length(LabBotTransforms)   
    %     qLabBot{i} = rLabBot.model.ikine(LabBotTransforms{i}, 'mask', [1, 1, 1, 0, 0, 0]);
    % 
    %     % Print q values for Logging
    %     fprintf('q%d = [', i);
    %     fprintf(' %.5f', qLabBot{i});
    %     fprintf(' ]\n');
    % end

    %% Generate Joint Trajectories
    % Pre-allocate matrix for combined joint trajectory
    qMatrixUR3Total = [];
    
    % Generate joint space trajectories for each consecutive pair of transformations
    for i = 1:(length(qUR3) - 1)
        qMatrixUR3 = jtraj(qUR3{i}, qUR3{i + 1}, steps);
        qMatrixUR3Total = [qMatrixUR3Total; qMatrixUR3];  
    end

    % % Pre-allocate matrix for combined joint trajectory
    % qMatrixLabBotTotal = [];
    % 
    % % Generate joint space trajectories for each consecutive pair of transformations
    % for i = 1:(length(qLabBot) - 1)
    %     qMatrixLabBot = jtraj(qLabBot{i}, qLabBot{i + 1}, steps);
    %     qMatrixLabBotTotal = [qMatrixLabBotTotal; qMatrixLabBot];  
    % end

    %% Animate Movement 
    % Animate the robot through the combined trajectory
    for i = 1:size(qMatrixUR3Total, 1)
        disp(['Animating step ', num2str(i), ' out of ', num2str(size(qMatrixUR3Total, 1))]);
        rUR3.model.animate(qMatrixUR3Total(i, :));
        % rLabBot.model.animate(qMatrixLabBotTotal(i, :));
        drawnow();
        
        pause(0.05)
    end
    
end