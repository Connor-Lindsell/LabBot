classdef GUI_Functions
    %% Properties
    properties
        controlEnabled = true; 

        % Objects of Parent classes
        movementController
        environment

        % Initialise Robot models variables
        rUR3
        rCustomBot

    end

    % Constructor method
    methods
        function obj = GUI_Functions(RobotUR3, RobotCustomBot)
           
            obj.rUR3 = RobotUR3;
            obj.rCustomBot = RobotCustomBot;

        end
    end
   
    %% Functions
    methods 

        function DemonstrationControl(obj)
            clf;
            disp('Demonstration Control Commencing')

            % if isempty(obj.environment.rUR3)
            %     error('rUR3 is not initialized. Please check environment initialization.');
            % else
            %     disp('rUR3 initialized successfully.');
            % end
            % Proceed with Move2Global calls

            
                       
            
            %% Transforms
            % UR3 End Effector Goal Destinations 
            % Cheecking for correct orientation
            UR3_Pos1 = [0.3,0.2,1.6]; % positive x Q++
            UR3_Pos2 = [-0.3,0.2,1.6]; % negative x Q-+
            UR3_Pos3 = [0.2,0.3,1.6]; % positive y Q++
            UR3_Pos4 = [0.2,-0.3,1.6]; % negative y Q+-
            UR3_Pos5 = [0.3,-0.2,1.6]; % positive x Q+-
            UR3_Pos6 = [-0.3,-0.2,1.6]; % negative x Q--
            UR3_Pos7 = [-0.2,0.3,1.6]; % positive y Q-+
            UR3_Pos8 = [-0.2,-0.3,1.6]; % negative y Q--
              
            % LabBot End Effector Goal Destinations 
            % LabBot_Pos1 = [0.2,0.2,0.2];
            % LabBot_End = [0.2,0.2,0.2];


            %% Perform Movements
            % Calling Move2Global using self
            % For UR3
            obj.movementController.Move2Global(UR3_Pos1, obj.rUR3);
            obj.movementController.Move2Global(UR3_Pos2, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos3, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos4, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos5, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos6, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos7, obj.environment.rUR3);
            obj.movementController.Move2Global(UR3_Pos8, obj.environment.rUR3);

            
            % For LabBot
            % obj.movementController.Move2Global(LabBot_Pos1, obj.environment.rLabBot);
            % obj.movementController.Move2Global(LabBot_End, obj.environment.rLabBot);             

            %% Mix Chemicals 
            % Not nessecary while testing optimisation 
            % Define chemicals to mix and their test tube locations
            chemicals2mix1 = {{'Bromine', 1}, ...  % Notice the use of curly braces
                              {'Iodine', 2}, ... 
                              {'Nitrogen', 4} ...
                              };

            % Call MixChem with 3 chemicals and mixing location 3
            self.MixChem( 3, chemicals2mix1, 3)

            chemicals2mix2 = {{'New Mixture', 3}, ... 
                              {'Nitrogen', 4} ...
                              };

            % Call MixChem with 2 chemicals and mixing location 5
            self.MixChem(2, chemicals2mix2, 5)
                    
        end

        function GUITeachUR3(self)
            
            robot = self.rUR3;
                    
            %% Setup virtual teach pendant
            pendant = GUI;   
               
            %% Infinite loop for teaching mode
            while 1
                % Read VTP values (joint angles in degrees)
                wrench = pendant.read1;
                
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

            end           
        end

        function GUITeachCustomBot(obj)
            robot = self.rCustomBot;
                    
            %% Setup virtual teach pendant
            pendant = GUI;   
               
            %% Infinite loop for teaching mode
            while 1
                % Read VTP values (joint angles in degrees)
                watch = pendant.read2;

                q(1) = watch(1);
                
                % Convert degrees to radians for each joint
                q(2:7) = deg2rad(watch(2:7)');
        
                disp(q);

                % Display the joint angles in the command window
                str = sprintf('--------------\n');
                for i = 1:7
                    str = [str, sprintf('Joint %d: %01.3f rad\n', i, q(i))];
                end
                str = [str, sprintf('--------------\n')];
                fprintf('%s', str);
        
                % Animate the robot with updated joint angles
                robot.model.animate(q);
        
                % Pause briefly for real-time update (adjust as needed)
                pause(0.05);

            end           
        end

        function GUICartesianUR3(obj)
            robot = self.rUR3;
            
            %% Setup virtual teach pendant
            pendant = TestTeach;   
        
        
            %% Infinite loop for teaching mode
            while 1
            
                % Read VTP values
                rench = pendant.read;
                
                q = [0 0 0 0 0 0];                 % Set initial robot configuration 'q'
            
                duration = 300;  % Set duration of the simulation (seconds)
                dt = 0.5;      % Set time step for simulation (seconds)
                
                n = 0;  % Initialise step count to zero 
                tic;    % recording simulation start time
                while( toc < duration)
                    
                    n=n+1; % increment step count
            
                    
                    
                    % Print buttons/axes info to command window
                    str = sprintf('--------------\n');
                    str = [str sprintf('Force  X:%01.3f\n',rench(1))];
                    str = [str sprintf('Force  Y:%01.3f\n',rench(2))];
                    str = [str sprintf('Force  Z:%01.3f\n',rench(3))];
                    str = [str sprintf('Torque X:%01.3f\n',rench(4))];
                    str = [str sprintf('Torque Y:%01.3f\n',rench(5))];
                    str = [str sprintf('Torque Z:%01.3f\n',rench(6))];
                    str = [str sprintf('--------------\n')];
                    fprintf('%s',str);
                    pause(0.05);  
                
                    fx = rench(1);
                    fy = rench(2);
                    fz = rench(3);
                    
                    tx = rench(4);
                    ty = rench(5);
                    tz = rench(6);
                    
                    f = [fx;fy;fz;tx;ty;tz]; % combined force-torque vector (wwrench)
                    
                    % 2 - use simple admittance scheme to convert force measurement into
                    % velocity command
                    Ka = diag([0.3 0.3 0.3 0.5 0.5 0.5]); % admittance gain matrix  
                    dx = Ka*f; % convert wwrench into end-effector velocity command
                    
                    % 2 - use DLS J inverse to calculate joint velocity
                    J = robot.model.jacobe(q);
                    
                    lambda = 0.1;
                    Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
                    dq = Jinv_dls*dx;
                    
                    % 3 - apply joint velocity to step robot joint angles
                    q = q + dq'*dt;
                    
                    
                    % Update plot
                    robot.model.animate(q);  
                    
                    % wait until loop time elapsed
                    if (toc > dt*n)
                        warning('Loop %i took too much time - consider increating dt',n);
                    end

                    while (toc < dt*n) % wait until loop time (dt) has elapsed 
                    end
                end
            end
        end

        function GUICartesianCustomBot(obj)
            robot = self.rCustomBot;
            
            %% Setup virtual teach pendant
            pendant = TestTeach;   
        
        
            %% Infinite loop for teaching mode
            while 1
            
                % Read VTP values
                atch = pendant.read;
                
                q = [0 0 0 0 0 0];                 % Set initial robot configuration 'q'
            
                                
                
                duration = 300;  % Set duration of the simulation (seconds)
                dt = 0.5;      % Set time step for simulation (seconds)
                
                n = 0;  % Initialise step count to zero 
                tic;    % recording simulation start time

                while( toc < duration)
                    
                    n=n+1; % increment step count
            
                    
                    
                    % Print buttons/axes info to command window
                    str = sprintf('--------------\n');
                    str = [str sprintf('Force  X:%01.3f\n',atch(1))];
                    str = [str sprintf('Force  Y:%01.3f\n',atch(2))];
                    str = [str sprintf('Force  Z:%01.3f\n',atch(3))];
                    str = [str sprintf('Torque X:%01.3f\n',atch(4))];
                    str = [str sprintf('Torque Y:%01.3f\n',atch(5))];
                    str = [str sprintf('Torque Z:%01.3f\n',atch(6))];
                    str = [str sprintf('--------------\n')];
                    fprintf('%s',str);
                    pause(0.05);  
                
                    fx = atch(1);
                    fy = atch(2);
                    fz = atch(3);
                    
                    tx = atch(4);
                    ty = atch(5);
                    tz = atch(6);
                    
                    f = [fx;fy;fz;tx;ty;tz]; % combined force-torque vector (wwrench)
                    
                    % 2 - use simple admittance scheme to convert force measurement into
                    % velocity command
                    Ka = diag([0.3 0.3 0.3 0.5 0.5 0.5]); % admittance gain matrix  
                    dx = Ka*f; % convert wwrench into end-effector velocity command
                    
                    % 2 - use DLS J inverse to calculate joint velocity
                    J = robot.model.jacobe(q);
                    
                    lambda = 0.1;
                    Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
                    dq = Jinv_dls*dx;
                    
                    % 3 - apply joint velocity to step robot joint angles
                    q = q + dq'*dt;
                    
                    
                    % Update plot
                    robot.model.animate(q);  
                    
                    % wait until loop time elapsed
                    if (toc > dt*n)
                        warning('Loop %i took too much time - consider increating dt',n);
                    end
                    
                    while (toc < dt*n) % wait until loop time (dt) has elapsed 
                    end
                end
            end
        end
        
        %%
        % Enable control method
        % function enableControl(obj)
        %     obj.controlEnabled = true;
        %     disp('Control enabled in GUI_Functions');
        % 
        % end

        % Disable control method
        % function disableControl(obj)
        %     obj.controlEnabled = false;
        %     disp('Control disabled in GUI_Functions');
        % end

        %% Joint Movement with Integrated Jogging and DLS
        function JointMovement(obj, robotName, joint, value, isJogging)
            % Move the joint based on either direct position control or jogging (velocity-based control)
            
            % Select the robot based on the robotName argument
            switch robotName
                case 'UR3'
                    robot = obj.rUR3;
                case 'LabBot'
                    robot = obj.rLabBot;
                otherwise
                    error('Unknown robot specified.');
            end

            if obj.controlEnabled

                disp(['Moving joint ', num2str(joint), ' for ', robotName]);

                % Read the current joint configuration
                currentQ = robot.getpos();
                
                if isJogging
                    % Velocity-based movement using Damped Least Squares (DLS)
                    f = zeros(6, 1);  % Zero force vector for 6-DOF
                    f(joint) = value;  % Assign the force (or velocity) for jogging

                    % Admittance scheme to convert force into velocity command
                    Ka = diag([0.3 0.3 0.3 0.5 0.5 0.5]);
                    dx = Ka * f;

                    % DLS Inverse Jacobian to calculate joint velocity
                    J = robot.jacobe(currentQ);
                    lambda = 0.1;
                    Jinv_dls = inv((J' * J) + lambda^2 * eye(6)) * J';

                    % Calculate joint velocity
                    dq = Jinv_dls * dx;

                    % Apply joint velocity to update joint configuration
                    dt = 0.1;  % Time step
                    newQ = currentQ + (dq' * dt);

                    % Animate the robot to the new configuration
                    robot.animate(newQ);
                    fprintf('Jogging robot joint %d with force %.2f, updated joint configuration.\n', joint, value);
                else
                    % Direct position-based control
                    currentQ(joint) = deg2rad(value);
                    robot.animate(currentQ);
                    fprintf('Moved robot joint %d to %.2f degrees.\n', joint, value);
                end
            else
                disp('Control is disabled. Please turn the switch on.');
            end
        end


        %% Mix Chemical Function 
        % Selects Chemicals and then mixes the chemicals by pouring them
        % together
        % 
        % Inputs - 
        % numOfChem: The number of chemicals to be mixed 
        % chem2mix: The chemicals to be mixed and the corresponding test 
        %           tube which it is stored, which are stored in an array 
        %           of pairs eg. {'Bromine', 1}
        % mixingLocation: Where the chemicals are to be mixed
        
        function MixChem(obj, numOfChem, chem2mix, mixingLocation)
            % Define test tube locations (you could adjust this as per your environment)
            testTubeLocation = {[0.2, -0.2, 0.2], ...
                                [0.2, -0.1, 0.2], ...
                                [0.2, 0, 0.2], ...
                                [0.2, 0.1, 0.2], ...
                                [0.2, 0.2, 0.2]};
                            
            % Iterate over the number of chemicals to mix
            for i = 1:numOfChem
                %% Retrieve Chem
                % Extract chemical name and its location from chemicals2mix array
                chemical = chem2mix{i}{1};  % Name of the chemical
                locationIndex = chem2mix{i}{2};  % Index of the test tube location
                fprintf('Picking up %s from test tube %d...\n', chemical, locationIndex);
                fprintf('\n');
                
                % Move to the test tube location to pick up the chemical
                finishPos = testTubeLocation{locationIndex};  % Test tube location 
                
                % Move UR3 to the test tube
                obj.movementController.Move2Global(finishPos, obj.environment.rUR3);
                
                % self.GripperClose();
                fprintf('Gripper closing to pick up %s...\n', chemical);
                fprintf('\n');
                
                % Move to the mixing location
                finishPos = testTubeLocation{mixingLocation};  % Mixing location 
                fprintf('Moving %s to the mixing location at test tube %d...\n', chemical, mixingLocation);
                fprintf('\n');
                
                % Move robot to the mixing location with the chemical
                obj.movementController.Move2Global(finishPos, obj.environment.rUR3);
                
                %% Mix Chem
                % self.PourChem(); 
                fprintf('Pouring %s into test tube at the mixing location...\n', chemical);
                fprintf('\n');
                
                %% Return Chem
                % Move back to the original test tube location to return the tube
                finishPos = testTubeLocation{locationIndex};  % Back to the original location
                fprintf('Returning test tube %d to its original position...\n', locationIndex);
                fprintf('\n');
                
                % Move robot back to return the test tube
                obj.movementController.Move2Global(finishPos, obj.environment.rUR3);
                
                % self.GripperOpen(); 
                fprintf('Gripper opening to release test tube %d...\n', locationIndex);
                fprintf('\n');
            end
        end

        

    end 


end 