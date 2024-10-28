function GUITeach()
    %% Robot Initialisation
    % Initaialising Robot Model
    robot = UR3;

    %% Scene Initialisation
    axis([-2 2 -2 2 2 4]);  % Set the axis limits to fit all objects in the environment
    xlabel('X-axis');  % Label the X-axis
    ylabel('Y-axis');  % Label the Y-axis
    zlabel('Z-axis');  % Label the Z-axis
    grid on;  % Display a grid for better visualization of object positions
    hold on;  % Keep the plot active for additional elements
 

    %% Setup virtual teach pendant
    pendant = TestTeach;   


    %% Infinite loop for teaching mode
    while 1
        % Read VTP values (joint angles in degrees)
        wrench = pendant.read;
        
        % Convert degrees to radians for each joint
        q = deg2rad(wrench');

        % Display the joint angles in the command window
        str = sprintf('--------------\n');
        for i = 1:6
            str = [str, sprintf('Joint %d: %01.3f rad\n', i, q(i))];
        end
        str = [str, sprintf('--------------\n')];
        fprintf('%s', str);

        % Animate the robot with updated joint angles
        robot.model.animate(q);

        % Pause briefly for real-time update (adjust as needed)
        pause(0.05);

    %%
    

end