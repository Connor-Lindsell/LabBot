function Move2Global(start, finish, robot)            
    steps = 100;
    
    % Define transforms with a downward orientation for the last joint (pointing down)
    transforms = {transl(start), transl(finish)};
    
    % Pre-allocate cell array for joint configurations
    q = cell(1, length(transforms));
    
    % Solve inverse kinematics for each transformation
    for i = 1:length(transforms)
        % Use a mask that includes Z position and orientation (roll around X-axis)
        q{i} = robot.model.ikine(transforms{i}, 'mask', [1, 1, 1, 0, 0, 0]);
        
        % Display the full joint angles using fprintf
        fprintf('q%d = [', i);
        fprintf(' %.5f', q{i});  % Display all joint angles in a row
        fprintf(' ]\n');
    end

    
    % Pre-allocate matrix for combined joint trajectory
    qMatrixTotal = [];
    
    % Generate joint space trajectories for each consecutive pair of transformations
    for i = 1:(length(q) - 1)
        qMatrix = jtraj(q{i}, q{i + 1}, steps);
        qMatrixTotal = [qMatrixTotal; qMatrix];  
    end

    % Animate the robot through the combined trajectory
    for i = 1:size(qMatrixTotal, 1)
        robot.model.animate(qMatrixTotal(i, :));
        drawnow();
    end
    hold on

end