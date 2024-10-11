function LabBot_7DOF_TeachFunction()
    clf;

    robot = LabBot_7DOF(); 
    
    robot.model.teach();

end