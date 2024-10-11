function LabBot_7DOF_TeachFunction()
    clf;

    % Instantiate the robot
    robot = LabBot_7DOF(); 

    % Plot the robot at its home configuration
    robot.model.plot(robot.homeQ); 

    % Pass the current joint state when calling teach
    currentJointState = robot.model.getpos();  % Get the current joint configuration
    robot.model.teach(currentJointState);  % Pass it to teach
end
