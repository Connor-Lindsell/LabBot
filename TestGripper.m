classdef TestGripper < RobotGripper

    methods
        % TestGripper function to test gripper opening and closing
        function testGripper(self)
            % Set the base transformation for testing
            baseTr = transl(0, 0, 0) * trotx(-pi/2);  % Translate to (0, 0, 0) and rotate by -90 degrees on x-axis
            self.model.base = baseTr;

            % Create the gripper
            [finger1, finger2] = self.CreateGripper();

            % Ensure the gripper starts in the closed position
            self.gripperState = 'Closed';

            % Open the gripper
            disp('Opening the gripper...');
            self.toggleGripperState(finger1, finger2);  % This will open the gripper

            % Pause briefly before closing
            pause(1);

            % Close the gripper
            disp('Closing the gripper...');
            self.toggleGripperState(finger1, finger2);  % This will close the gripper

            % Display message indicating the test is done
            disp('Gripper test completed.');
        end
    end
end
