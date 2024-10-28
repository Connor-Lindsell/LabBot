function LabBot_7DOF_TeachFunction()
    clf;

    % Instantiate the robot
    robot = CustomBot(); 

    % Pass the current joint state when calling teach
    currentJointState = robot.model.getpos();  % Get the current joint configuration
    robot.model.teach(currentJointState);  % Pass it to teach
    robot.PlotAndColourRobot();

end
