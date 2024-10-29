function GUITeachXYZ()
    %% Robot Initialisation
    % Initaialising Robot Model
    robot = UR3;

    %% Scene Initialisation
    axis([-2 2 -2 2 0 4]);  % Set the axis limits to fit all objects in the environment
    xlabel('X-axis');  % Label the X-axis
    ylabel('Y-axis');  % Label the Y-axis
    zlabel('Z-axis');  % Label the Z-axis
    grid on;  % Display a grid for better visualization of object positions
    hold on;  % Keep the plot active for additional elements
 

    %% Setup virtual teach pendant
    pendant = TestTeach;   


    %% Infinite loop for teaching mode
    while 1
    
    % Read VTP values
    wrench = pendant.read;
    
    q = [0 -pi 0 0 0 0];                 % Set initial robot configuration 'q'

    
    robot.model.plot(q);          % Plot robot in initial configuration
    robot.model.delay = 0.001;    % Set smaller delay when animating
    
    
    duration = 300;  % Set duration of the simulation (seconds)
    dt = 0.5;      % Set time step for simulation (seconds)
    
    n = 0;  % Initialise step count to zero 
    tic;    % recording simulation start time
    while( toc < duration)
        
        n=n+1; % increment step count

        
        
        % Print buttons/axes info to command window
        str = sprintf('--------------\n');
        str = [str sprintf('Force  X:%01.3f\n',wrench(1))];
        str = [str sprintf('Force  Y:%01.3f\n',wrench(2))];
        str = [str sprintf('Force  Z:%01.3f\n',wrench(3))];
        str = [str sprintf('Torque X:%01.3f\n',wrench(4))];
        str = [str sprintf('Torque Y:%01.3f\n',wrench(5))];
        str = [str sprintf('Torque Z:%01.3f\n',wrench(6))];
        str = [str sprintf('--------------\n')];
        fprintf('%s',str);
        pause(0.05);  
    
        fx = wrench(1);
        fy = wrench(2);
        fz = wrench(3);
        
        tx = wrench(4);
        ty = wrench(5);
        tz = wrench(6);
        
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


    