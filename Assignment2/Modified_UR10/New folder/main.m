Robot=LinearUR10;
%get Robot's current Joint Positions
q_0 = Robot.model.getpos;
%get the Robot's Target Position and orientation
T = transl([-0.5 2 1])*trotx(pi);
%Compute the Robot's Goal Joint Positions
q_1 = Robot.model.ikcon(T);
steps = 50;
%Compute the Robot's trajectory for its movement from current position to
%goal position
q = jtraj(q_0,q_1,steps); 
%Simulate the response
for i = 1:steps
    Robot.model.animate(q(i,:))                          
    drawnow();
end
