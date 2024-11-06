function Move2Global(self, startTr, finishTr, robot)  
    %% Properties
    steps = 100;
    startTr = [0.3,0.3,0.3];
    finishTr = [0.35,-0.25,0.1];
    robot = UR3;
    beaker = Beaker();


    %% Inverse Kinematics
    % Roll Pitch Yaw
    rpy = troty(pi/2) * trotx(pi/2);

    % Define transforms with a downward orientation for the last joint (pointing down)
    transforms = {transl(startTr) * rpy, transl(finishTr) * rpy};
    
    % Pre-allocate cell array for joint configurations
    q = cell(1, length(transforms));
    
    % Solve inverse kinematics for each transformation
    for i = 1:length(transforms)
        
        % qn = [pi, deg2rad(-75), deg2rad(25),deg2rad(-75), deg2rad(90),0];

        % Use a mask that includes Z position and orientation (roll around X-axis)
        q{i} = robot.model.ikcon(transforms{i});
        
        % Display the full joint angles using fprintf
        fprintf('q%d = \n', i);
        fprintf('\n [');
        fprintf('  %.5f  ', q{i});  % Display all joint angles in a row
        fprintf(']\n');
        fprintf('\n');
    end

    %% Not Used for Testing 
    % this part of the function only works in conjunction with the getEnd
    % EffectorPos function, while this script is used for testing the
    % reliability of the move 2 global function we leave this aspect of the
    % code out
    %
    % % Get the current end-effector position of the robot
    % currentEndEffectorPos = self.getEndEffectorPos(robot);
    % 
    % % Define a tolerance for checking positions
    % tolerance = 1e-4;  % You can adjust this value based on your accuracy needs
    % 
    % % Check if the current end-effector position is close enough to the start position
    % if all(abs(currentEndEffectorPos - startTr) < tolerance)
    %     fprintf('Robot is already at the start position.\n');
    %     fprintf('\n');
    % else
    %     fprintf('Moving robot to the start position...\n');
    %     fprintf('\n');
    % 
    %     % If the robot is not at the start position, move it there
    %     qMatrix = jtraj(robot.model.getpos(), q{1}, self.steps);  % Joint trajectory from current position to start position
    % 
    %     % Animate the movement to the start position
    %     for i = 1:size(qMatrix, 1)
    %         robot.model.animate(qMatrix(i, :));
    %         drawnow();
    %     end
    % end
        
    %% Joint Trajectory
    % Pre-allocate matrix for combined joint trajectory
    qMatrixTotal = [];
    
    % Generate joint space trajectories for each consecutive pair of transformations
    for i = 1:(length(q) - 1)
        qMatrix = jtraj(q{i}, q{i + 1}, steps);
        qMatrixTotal = [qMatrixTotal; qMatrix];  
    end

    %% Animation
    % Animate the robot through the combined trajectory
    for i = 1:size(qMatrixTotal, 1)
        robot.model.animate(qMatrixTotal(i, :));
        % 
        % 
        % beaker.model.base = robot.model.fkine(qMatrixTotal(i,:)).T * trotx(pi/2) * troty(-pi/2) * transl(0,0.1,-0.1);
        % beaker.model.animate(qMatrixTotal(i, :));
        drawnow();

        pause(0.05);
    end
    hold on

end