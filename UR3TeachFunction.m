function UR3TeachFunction()
    clf;

    robot = UR3(); 
    
    robot.model.teach();

end
